// File Name: tracking.cpp
// Author: Shaoxiong Wang
// Create Time: 2018/12/20 10:11

#include "tracking_class.h"
#include <iostream>
#include <stdio.h>

double dist_sqr(Point_t a, Point_t b){
    return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
}

double sum(Point_t a){
    return (a.x*a.x + a.y*a.y);
}

int Matching::precessor(int i, int j) {
    return (degree[i][j] <= theta 
        && degree[i][j] >= -theta 
        && Dist[i][j] <= dmax 
        && Dist[i][j] >= dmin);
}

// 1st initialization called by m = find_marker.Matching
// The constructor initializes the Matching object with grid dimensions, video frame rate, 
// and marker array parameters (such as initial positions and intervals).
// Constructor for the Matching class, initializing it with specific parameters for marker tracking.
// Defining threshold parameters for a matching configuration to be considered valid.
Matching::Matching(int N_, int M_, int fps_, double x0_, double y0_, double dx_, double dy_){
    // Setting grid dimensions and video frame rate based on constructor arguments.
    N = N_; // Number of rows in the marker grid.
    M = M_; // Number of columns in the marker grid.
    NM = N * M; // Total number of markers expected in the grid.

    fps = fps_; // Frames per second of the video input.

    // Setting the initial position (top-left corner of the grid) and intervals between markers.
    x_0 = x0_; // X-coordinate of the upper-left marker.
    y_0 = y0_; // Y-coordinate of the upper-left marker.
    dx = dx_; // Horizontal interval between adjacent markers.
    dy = dy_; // Vertical interval between adjacent markers.(under default setting dx = dy)

    // Pre-calculate the expected positions of all markers in the grid based on the initial position and intervals.
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < M; j++) {
            O[i][j].x = x_0 + j * dx; // Calculating X-coordinate for each marker.
            O[i][j].y = y_0 + i * dy; // Calculating Y-coordinate for each marker.
            // Note that coordinates are stored from upper left row by row into O
        }
    }
    // Initialization flag, indicating readiness to process frames.
    flag_record = 1;

    // Setting thresholds for matching based on distances and movement.
    dmin = (dx * 0.5) * (dx * 0.5); // Minimum squared distance allowed between markers.
    dmax = (dx * 1.8) * (dx * 1.8); // Maximum squared distance allowed between markers.
    // Represents the minimum/maximum allowable squared distance between two markers for them 
    // to be considered a valid matching pair. This threshold can prevent the algorithm from matching 
    // markers that are too close to each other relative to the expected grid spacing, which might 
    // indicate that they are not true matches (perhaps due to noise or detection errors).
    theta = 70; // Maximum angle deviation in degrees for marker matching.
    //It represents the maximum angle (in degrees) that the line between two matched markers 
    // can deviate from the expected angle. In a grid of markers, if the system expects markers 
    // to be aligned horizontally or vertically, theta ensures that any pair of markers being 
    // matched as part of a sequence must not form an angle with each other that exceeds this limit. 
    // This helps in maintaining the structural integrity of the marker array and avoiding incorrect 
    // matches due to significant angular misalignment.
    moving_max = dx * 2; // Maximum movement allowed for markers between frames.
    // Defines a threshold for the maximum movement a marker can have 
    // between frames or from its initial position. 
    flow_difference_threshold = dx * 0.8; // Threshold for considering flow differences in matching.
    // This parameter is likely used in the context of flow vector consistency between adjacent markers. 
    // In a grid, you would expect neighboring markers to have similar motion vectors (flows). 
    // If the difference between the flows of two adjacent markers exceeds this threshold, 
    // it could indicate an inconsistency in the tracking, such as one marker being incorrectly matched 
    // or an object obscuring part of the scene.
    cost_threshold = 15000 * (dx / 21) * (dx / 21); // Threshold for the cost of matching configuration.
    // cost_threshold is likely the upper limit on the cost that the matching algorithm is willing to 
    // accept for a matching configuration to be considered valid. The cost function is a crucial part 
    // of the algorithm, as it quantifies how good a particular marker configuration is. 
    // This function likely considers factors such as:
    // 1. The distance markers have moved from their original positions.
    // 2. The degree of alignment with neighboring markers.
    // 3. How well the current observed configuration matches the expected grid layout.
    // The cost_threshold value sets a cap to distinguish between acceptable and unacceptable arrangements.
    //  If the calculated cost for a particular arrangement exceeds cost_threshold, 
    //  the algorithm will reject this arrangement and search for another with a lower cost.
    // The use of thresholds allows the algorithm to discard improbable matches quickly, 
    // focusing computational resources on more promising configurations, 
    // leading to more efficient and accurate marker tracking.
}

// 2nd Prepare Marker Centers by m.init(mc). Called once per frame or when new marker centers are available.
// Reads the current frame's marker centers.
// Initializes necessary data structures (C, Dist, degree).
// Calculates pairwise distances and angles between all marker centers.
// void Matching::init(Point_t *centers, int count){
void Matching::init(std::vector<std::vector<double>> centers) {
    int i, j;

    // read points from marker centers (mc)
    n = centers.size();

    for (i = 0; i < n; i++){
        C[i].x = centers[i][0];
        C[i].y = centers[i][1];
        C[i].id = i;
        // Coordinates in C are stored with the same sequence in mc
        // std::cout<<C[i].x<<" "<<C[i].y<<" "<<std::endl;
    }

    // init arrays for search
    memset(done, 0, sizeof(done));
    memset(occupied, -1, sizeof(occupied));
    minf = -1;

    // sort by x-axis, if same by y-axis
    std::sort(C, C+n);

    // calculate distance and angle O(N^2M^2)
    for (i = 0; i < n; i++) {
        for (j = 0; j < i; j++) {
            Dist[i][j] = dist_sqr(C[i], C[j]);
            degree[i][j] = asin(fabs(C[i].y - C[j].y) / sqrt(Dist[i][j])) * 180.0 / PI;
        }
    }

}

// 3rd Execute Matching by m.run() 
// Starts the matching process.
// Calculates the number of missing and spare markers compared to the expected grid (N*M).
// Calls Matching::dfs(...) to explore matching configurations.
void Matching::run(){
    int missing, spare;

    time_st = clock();

    missing = NM - n;
    spare = n - NM;
    missing = missing < 0 ? 0 : missing;
    spare = spare < 0 ? 0 : spare;
    dfs(0, 0, missing, spare);
    for(int t=1;t<=3;t++) {
        if(minf == -1){
            // std::cout<<"TRY AGAIN!!"<<std::endl;
            memset(done, 0, sizeof(done));
            memset(occupied, -1, sizeof(occupied));
            dfs(0, 0, missing + 1, spare + 1);
        }
    }
    int i;
    if (flag_record == 1){
        flag_record = 0;
        for (i = 0; i < n; i++){
            O[MinRow[i]][MinCol[i]].x = C[i].x;
            O[MinRow[i]][MinCol[i]].y = C[i].y;
        }
    }
    // std::cout<<"MINF "<<minf<<"\t\t";
}

// 4th Depth-First Search with Matching::dfs(...)
// Recursively explores all configurations of matching detected markers to grid positions.
// For each marker i, attempts to place it in a valid grid position, 
// updating cost and respecting constraints (dmax, dmin, theta) checked by Matching::precessor(...).
// Calls Matching::calc_cost(i) to evaluate the cost of placing marker i.
// Handles missing and spare markers by allowing placements outside the expected grid positions or skipping markers.
// If minf is unset or improved upon, updates the best configuration found so far.
// Invokes Matching::infer() to finalize the cost by inferring positions for unplaced markers.
void Matching::dfs(int i, double cost, int missing, int spare){
    // if(occupied[6][0] <= -1 && occupied[7][0] <= -1)
    // std::cout<<i<<" "<<"COST: "<<cost<<"fmin: "<< minf<< " missing "<<missing<<" spare "<<spare<<std::endl;
    if (((float)(clock()-time_st))/CLOCKS_PER_SEC >= 1.0 / fps) return;
    if(cost >= minf && minf != -1) return;
    if(cost >= cost_threshold) return;
    int j, k, count = 0, flag, m, same_col;
    double c;
    if (i >= n) {
        cost += infer();
        // printf("\nCOST: %lf\n", cost);
        // for (j=0;j<n;j++){
        //     printf("%d %d \t %lf %lf\n", Row[j], Col[j], C[j].x, C[j].y);
        // }
        // printf("--------------------------------------------\n");
        if (cost < minf || minf == -1) {
            // if (int(cost) == 31535) cost = 0;
            minf = cost;
            for (j=0;j<n;j++){
                // printf("%d %d \t %lf %lf\n", Row[j], Col[j], C[j].x, C[j].y);
                MinRow[j] = Row[j];
                MinCol[j] = Col[j];
                if (Row[j] < 0) continue;
                D[Row[j]][Col[j]].x = C[j].x;
                D[Row[j]][Col[j]].y = C[j].y;
            }
            for (j=0;j<N;j++){
                for (k=0;k<M;k++){
                    MinOccupied[j][k] = occupied[j][k];
                    MinD[j][k].x = D[j][k].x;
                    MinD[j][k].y = D[j][k].y;
                }
            }
        }
        return;
    }


    for (j=0;j<i;j++) {
        // if (i == 45) std::cout<<i<<" "<<j<<std::endl;

        if (precessor(i, j)) {
            Row[i] = Row[j];
            Col[i] = Col[j] + 1;
            count++;
            if (Col[i] >= M) continue;
            if (occupied[Row[i]][Col[i]] > -1) continue;
            if (Row[i] > 0 && occupied[Row[i]-1][Col[i]] > -1 && C[i].y <= C[occupied[Row[i]-1][Col[i]]].y) continue;
            if (Row[i] < N - 1 && occupied[Row[i]+1][Col[i]] > -1 && C[i].y >= C[occupied[Row[i]+1][Col[i]]].y) continue;
            int vflag = 0;
            for (k=0;k<N;k++){
                same_col = occupied[k][Col[i]];
                if(same_col > -1 && ((k < Row[i] && C[same_col].y > C[i].y) || (k > Row[i] && C[same_col].y < C[i].y))){
                    vflag = 1;
                    break;
                }
            }
            if (vflag == 1) continue;
            occupied[Row[i]][Col[i]] = i;

            c = calc_cost(i);
            dfs(i+1, cost+c, missing, spare);
            occupied[Row[i]][Col[i]] = -1;
        }
    }


    // if (count == 0) {
        for (j=0;j<N;j++) {
            if(done[j] == 0){
                flag = 0;
                for (int k = 0;k < N;k++) {
                    // printf("%d %d %d %d\t\t", k, done[k], first[k], C[i].x);
                    if (done[k] && 
                        ((k < j && first[k] > C[i].y) || (k > j && first[k] < C[i].y))
                        ){
                        flag = 1;
                        break;
                    }
                }
                if (flag == 1) continue;
                done[j] = 1;
                first[j] = C[i].y;
                Row[i] = j;
                Col[i] = 0;

                occupied[Row[i]][Col[i]] = i;
                c = calc_cost(i);

                dfs(i+1, cost+c, missing, spare);
                done[j] = 0;
                occupied[Row[i]][Col[i]] = -1;
            }
        }
    // }

    // considering missing points
    // if (C[i].y > dy && C[i].y < O[0][M-1].y - dy / 2) return;
    for(m=1;m<=missing;m++){
        for (j=0;j<N;j++) {
            // if (j >= 1 && j < N - 1) continue;
            if(fabs(C[i].y - O[j][0].y) > moving_max) continue;
            for(k=M-1;k>=0;k--) if(occupied[j][k]>-1) break;
            if(k+m+1>=M) continue;
            if (sqrt(sum(C[i] - O[j][k+m+1])) > moving_max) continue;
            for(int t=1;t<=m;t++) occupied[j][k+t] = -2;
            Row[i] = j;
            Col[i] = k + m + 1;
            c = calc_cost(i);
            occupied[Row[i]][Col[i]] = i;
            dfs(i+1, cost+c, missing - m, spare);

            for(int t=1;t<=m+1;t++) occupied[j][k+t] = -1;
        }
    }

    if (spare > 0){
        Row[i] = -1;
        Col[i] = -1;
        dfs(i+1, cost, missing, spare-1);
    }
}

// 5th Calculate Individual Placement Cost with Matching::calc_cost(int i)
// Calculates the cost of placing a marker based on its displacement 
// and alignment with neighboring markers, using dist_sqr and sum for geometric calculations.
double Matching::calc_cost(int i){
    double c = 0, cost = 0;
    int left, up, down;
    Point_t flow1, flow2;

    cost = cost + K1 * sum(C[i] - O[Row[i]][Col[i]]);
    flow1 = C[i] - O[Row[i]][Col[i]];

    if(Col[i] > 0){
        left = occupied[Row[i]][Col[i]-1];
        if (left > -1){
            flow2 = C[left] - O[Row[i]][Col[i]-1];
            c = sum(flow2 - flow1);
            if (sqrt(c) >= flow_difference_threshold) c = 1e8;
            cost +=  K2 * c;
        }
    }
    if(Row[i] > 0){
        up = occupied[Row[i]-1][Col[i]];
        if (up > -1){
            flow2 = C[up] - O[Row[i]-1][Col[i]];
            c = sum(flow2 - flow1);
            if (sqrt(c) >= flow_difference_threshold) c = 1e8;
            cost +=  K2 * c;
        }
    }
    if(Row[i] < N - 1){
        down = occupied[Row[i] + 1][Col[i]];
        if (down > -1){
            flow2 = C[down] - O[Row[i]+1][Col[i]];
            c = sum(flow2 - flow1);
            if (sqrt(c) >= flow_difference_threshold) c = 1e8;
            cost +=  K2 * c;
        }
    }
    return cost;
}

// 6th Infer Positions for Unmatched Markers with Matching::infer()
// After exploring all configurations via dfs, infer is called to estimate positions 
// for any markers that couldn't be directly matched, adjusting the overall cost accordingly.
double Matching::infer(){
    double cost = 0;
    int boarder_nb = 0;
    int i, j, k, x, y, d=1, cnt, nx, ny, nnx, nny;

    int dir[4][2] = {{0, -1}, {-1, 0}, {0, 1}, {1, 0}};
    Point_t flow1, flow2;

    Point_t moving;

    for(i = 0; i < N; i++){
        for(j = 0;j < M; j++){
            if(occupied[i][j] <= -1){
                moving.x = 0;
                moving.y = 0;
                cnt = 0;
                for (k=0;k<4;k++){
                    nx = i + dir[k][0];
                    ny = j + dir[k][1];
                    nnx = nx + dir[k][0];
                    nny = ny + dir[k][1];
                    if (nnx < 0 || nnx >= N || nny < 0 || nny >= M) continue;
                    if (occupied[nx][ny] <= -1 || occupied[nnx][nny] <= -1) continue;
                    moving = moving + (C[occupied[nx][ny]] - O[nx][ny] + (C[occupied[nx][ny]] - O[nx][ny] - C[occupied[nnx][nny]] + O[nnx][nny]));
                    cnt += 1;
                }
                if(cnt == 0){
                    for(x=i-d;x<=i+d;x++){
                        for(y=j-d;y<=j+d;y++){
                            if (x < 0 || x >= N || y < 0 || y >= M) continue;
                            if (occupied[x][y] <= -1) continue;
                            moving = moving + (C[occupied[x][y]] - O[x][y]);
                            cnt += 1;
                        }
                    }
                }
                if(cnt == 0){
                    for(x=i-d-1;x<=i+d+1;x++){
                        for(y=j-d-1;y<=j+d+1;y++){
                            if (x < 0 || x >= N || y < 0 || y >= M) continue;
                            if (occupied[x][y] <= -1) continue;
                            moving = moving + (C[occupied[x][y]] - O[x][y]);
                            cnt += 1;
                        }
                    }
                }
                D[i][j] = O[i][j] + moving / (cnt + 1e-6);
                if (j == 0 && D[i][j].y >= O[i][j].y - dy / 2.0) boarder_nb++;
                if (j == N-1 && D[i][j].y <= O[i][j].y + dy / 2.0) boarder_nb++;
                cost = cost + K1 * sum(D[i][j] - O[i][j]);
            }
        }
    }

    if(boarder_nb >= N -1 ) cost += 1e7;

    for(i = 0; i < N; i++){
        for(j = 0;j < M; j++){
            if(occupied[i][j] <= -1){
                flow1 = D[i][j] - O[i][j];
                for (k = 0; k < 4; k++){
                    nx = i + dir[k][0];
                    ny = j + dir[k][1];
                    if (nx < 0 || nx > N - 1 || ny < 0 || ny > M -1) continue;
                    if (occupied[nx][ny] > -1){
                        flow2 = (C[occupied[nx][ny]] - O[nx][ny]);
                        cost +=  K2 * sum(flow2 - flow1);
                    }
                    else if (k < 2 && occupied[nx][ny] <= -1){
                        flow2 = (D[nx][ny] - O[nx][ny]);
                        cost +=  K2 * sum(flow2 - flow1);
                    }
                }
            }
        }
    }
    return cost;
}

// 7th Retrieve Matching Results by flow = m.get_flow()
// Once the matching process concludes (run completes), 
// get_flow compiles and returns the final matching configuration, 
// including original and matched positions of markers and their occupancy grid.
std::tuple<vvd, vvd, vvd, vvd, vvd> Matching::get_flow() {
    vvd Ox(N), Oy(N), Cx(N), Cy(N), Occupied(N);

    int i, j;
    for(i = 0; i < N; i++){
        Ox[i] = vd(M); Oy[i] = vd(M); Cx[i] = vd(M); Cy[i] = vd(M); Occupied[i] = vd(M);
        for(j = 0; j < M; j++){
            Ox[i][j] = O[i][j].x;
            Oy[i][j] = O[i][j].y;
            Cx[i][j] = MinD[i][j].x;
            Cy[i][j] = MinD[i][j].y;
            Occupied[i][j] = MinOccupied[i][j];
            // Point a(matcher.O[i][j].x, matcher.O[i][j].y), b(matcher.MinD[i][j].x + 2 * (matcher.MinD[i][j].x - matcher.O[i][j].x), matcher.MinD[i][j].y + 2 * (matcher.MinD[i][j].y - matcher.O[i][j].y));
        }
    }

    return std::make_tuple(Ox, Oy, Cx, Cy, Occupied);
}



std::tuple<double, double> Matching::test() {
    return std::make_tuple(dx, dy);
}

// Define a Python module named "find_marker". This name is used in Python to import the module.
// 'm' is the module handle, used to add classes, functions, or variables to the module.
PYBIND11_MODULE(find_marker, m) {
    // Expose the C++ class 'Matching' to Python, naming it "Matching" in the Python side.
    // The 'm' argument specifies that this class is being added to the 'find_marker' module.
    py::class_<Matching>(m, "Matching")
        // Define the constructor of the 'Matching' class that can be called from Python.
        // The constructor parameters are specified, along with default values for each.
        .def(py::init<int, int, int, double, double, double, double>(),
             py::arg("N_") = 8, py::arg("M_") = 8, py::arg("fps_") = 30, 
             py::arg("x0_") = 80., py::arg("y0_") = 15., py::arg("dx_") = 21., py::arg("dy_") = 21.)
        // Expose the 'run' method of the 'Matching' class to Python.
        // This allows the method to be called on instances of 'Matching' from Python.
        .def("run", &Matching::run)
        // Expose the 'test' method for testing or demonstration.
        .def("test", &Matching::test)
        // Expose the 'init' method, allowing initialization with specific parameters from Python.
        .def("init", &Matching::init)
        // Expose the 'get_flow' method to Python. This method returns some form of data structure
        // representing the flow computed by the 'Matching' algorithm.
        .def("get_flow", &Matching::get_flow);
}

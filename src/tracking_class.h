#include <stdio.h>
#include <math.h>
#include <string.h>
#include <algorithm>
#include <time.h>
#include <thread>
#include <chrono>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
namespace py = pybind11;

// Define convenient types for vectors of doubles and vectors of vector of doubles.
typedef std::vector<double> vd;
typedef std::vector<std::vector<double>> vvd;

// Define constants for maximum allowable markers and grid dimensions.
#define MAXN 30     // Maximum number of rows in the marker grid.
#define MAXM 30     // Maximum number of columns in the marker grid.
#define MAXNM 900   // Maximum number of markers (product of rows and columns).
#define PI 3.14159265 // Define PI for angle calculations.

// Define a Point_t struct to represent a point with x and y coordinates and an identifier.
struct Point_t
{
    double x, y;    // The x and y coordinates of the point.
    int id;     // An identifier for the point.

    Point_t(){  // Default constructor.
    }

     // Constructor with parameters to initialize a point.
    Point_t(double x, double y, double id=0){
        this->x = x;
        this->y = y;
        this->id = id;
    }

    // Comparison operator to sort points. Sorts by x first, then by y if x is the same.
    bool operator<(Point_t other) const
    {
        return (x < other.x || (x == other.x && y < other.y));
    }

    // Subtraction operator to get the vector (difference) between two points.
    Point_t operator-(Point_t other) const
    {
        Point_t ret(x-other.x, y-other.y);
        return ret;
    }

    // Addition operator to get the new point by adding vector to the current point.
    Point_t operator+(Point_t other) const
    {
        Point_t ret(x+other.x, y+other.y);
        return ret;
    }
    
    // Division operator to scale down the vector by a double value.
    Point_t operator/(double other) const
    {
        Point_t ret(x/other, y/other);
        return ret;
    }
};

class Matching{
private:
    // double x_0 = 160, y_0 = 30, dx = 43.0, dy = 43.0; //GelSight Hanjun x1
    double x_0, y_0, dx, dy; //GelSight Hanjun x0.5
    // double x_0 = 34, y_0 = 37, dx = 27.0, dy = 27.0; //34 - 223,  37 - 200 GelSight_SX
    // double x_0 = 6, y_0 = 16, dx = 31.0, dy = 31.0; //6 - 130,  16 - 138 HSR x0.5
    // double x_0 = 12, y_0 = 32, dx = 62.0, dy = 62.0; //6 - 130,  16 - 138 HSR x1
    // double x_0 = 15, y_0 = 15, dx = 23.0, dy = 23.0; //15-195 15-202 HSR blue

    int Row[MAXNM], Col[MAXNM];
    int Dist[MAXNM][MAXNM], done[MAXN], occupied[MAXN][MAXM], first[MAXN];
    int fps;
    double degree[MAXNM][MAXNM];
    double dmin, dmax, theta;
    double moving_max;
    double cost_threshold, flow_difference_threshold;
    clock_t time_st;

public:
    int n;
    int N, M, NM;
    int flag_record = 1;

    int MinRow[MAXNM], MinCol[MAXNM], MinOccupied[MAXN][MAXM];
    double minf = -1;

    Point_t O[MAXN][MAXM], D[MAXN][MAXM], C[MAXNM], MinD[MAXN][MAXM];
    double K1 = 0.1, K2 = 1;

    Matching(int N_=8, int M_=8, int fps_=30, double x_0 = 80., double y_0 = 15., double dx = 21.0, double dy = 21.0);
    // void init(Point_t *centers, int count);
    void init(std::vector<std::vector<double>> input);
    int precessor(int i, int j);
    double calc_cost(int i);
    void dfs(int i, double cost, int missing, int spare);
    void run();
    std::tuple<vvd, vvd, vvd, vvd, vvd> get_flow();
    std::tuple<double, double> test();
    double infer();
};

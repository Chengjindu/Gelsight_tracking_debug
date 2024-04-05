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
// Create the linear operation rule for spatial vetor calculation when point information are known
struct Point_t
{
    double x, y;    // The x and y coordinates of the point.
    int id;     // An identifier for the point.

    // Parameterized constructor with default values, capable of acting as a default constructor.
    Point_t(double x = 0.0, double y = 0.0, int id = 0) : x(x), y(y), id(id) {
        // Initialization is done via the initializer list (after the colon).
    }

    // Comparison operator to sort points. Sorts by x first, then by y if x is the same.
    bool operator<(const Point_t& other) const {
        return x < other.x || (x == other.x && y < other.y);
    }

    // Subtraction operator to get the vector (difference) between two points.
    Point_t operator-(const Point_t& other) const {
        return Point_t(x - other.x, y - other.y);
    }

    // Addition operator to get the new point by adding vector to the current point.
    Point_t operator+(const Point_t& other) const {
        return Point_t(x + other.x, y + other.y);
    }

    // Division operator to scale down the vector by a double value.
    Point_t operator/(double scalar) const {
        return Point_t(x / scalar, y / scalar);
    }
};

// The Matching class that handles the matching of detected markers to a pre-defined grid.
class Matching{
private:
    // double x_0 = 160, y_0 = 30, dx = 43.0, dy = 43.0; //GelSight Hanjun x1
    // double x_0, y_0, dx, dy; //GelSight Hanjun x0.5
    // double x_0 = 34, y_0 = 37, dx = 27.0, dy = 27.0; //34 - 223,  37 - 200 GelSight_SX
    // double x_0 = 6, y_0 = 16, dx = 31.0, dy = 31.0; //6 - 130,  16 - 138 HSR x0.5
    // double x_0 = 12, y_0 = 32, dx = 62.0, dy = 62.0; //6 - 130,  16 - 138 HSR x1
    // double x_0 = 15, y_0 = 15, dx = 23.0, dy = 23.0; //15-195 15-202 HSR blue

    double x_0, y_0; // Initial position (top-left corner) of the grid.
    double dx, dy; // Intervals between markers horizontally and vertically.
    int fps;// Frames per second of the video input for timing considerations.

    // Arrays used in the matching algorithm:
    Point_t O[MAXN][MAXM], D[MAXN][MAXM]; // Expected and inferred marker positions.
    Point_t C[MAXNM], MinD[MAXN][MAXM]; // Detected marker positions and the best inferred positions.

    // Arrays for tracking state during matching:
    int Row[MAXNM], Col[MAXNM]; // Row and column indices for detected markers.
    int MinRow[MAXNM], MinCol[MAXNM], MinOccupied[MAXN][MAXM]; // Best matching row, column indices, and occupancy.

    // Variables for matching calculations:
    double degree[MAXNM][MAXNM]; // Angles between markers.
    int Dist[MAXNM][MAXNM]; // Distances between markers.
    int done[MAXN], occupied[MAXN][MAXM], first[MAXN]; // State flags for matching process.

    // Thresholds for the matching algorithm:
    double dmin, dmax; // Minimum and maximum squared distances between markers.
    double theta; // Maximum angle deviation allowed for a valid match.
    double moving_max; // Maximum allowed movement for markers between frames.
    double minf = -1; // Tracks the minimum cost found during the matching process.
    double cost_threshold, flow_difference_threshold; // Thresholds for cost calculations.
    // 'done' flags each row to indicate whether it has been processed in the current search path.
    // 'occupied' marks grid cells with the index of the marker placed there, or -1 if the cell is empty.
    // 'first' records the y-coordinate of the first marker placed in each row to enforce vertical ordering constraints.

    clock_t time_st; // Timing variable for performance measurement.

    // Weights for cost function components.
    double K1 = 0.1, K2 = 1;

public:
    int n; // Current number of detected markers.
    int N, M, NM; // Dimensions of the grid and the total number of expected markers.
    int flag_record = 1; // Flag indicating whether the current frame is the first record.

    // Constructor declaration.
    Matching(int N_ = 8, int M_ = 8, int fps_ = 30, double x0_ = 80., double y0_ = 15., double dx_ = 21.0, double dy_ = 21.0);

    // Method declarations:
    void init(std::vector<std::vector<double>> input); // Initializes the matching process with detected markers.
    int precessor(int i, int j); // Checks if a marker is a valid predecessor.
    double calc_cost(int i); // Calculates the cost of placing a marker.
    void dfs(int i, double cost, int missing, int spare); // Recursive method for depth-first search in matching.
    void run(); // Runs the matching algorithm.
    std::tuple<vvd, vvd, vvd, vvd, vvd> get_flow(); // Retrieves the flow of markers after matching.
    std::tuple<double, double> test(); // Tests the algorithm with dummy data.
    double infer(); // Infers positions for unmatched markers.
};

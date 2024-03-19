#ifndef PLANNER_H
#define PLANNER_H

#include <vector>
#include <string>
#include <map>
#include <tuple>

// Constants
#define PI 3.141592654
#define LINKLENGTH_CELLS 10

// Function Prototypes

// Generates a random configuration for the arm
std::vector<double> getRandomConfig(int numofDOFs);

// Finds the nearest neighbor in the roadmap to a given configuration
int findNearestNeighbor(const std::vector<std::vector<double>> &roadmap, const std::vector<double> &config);

// Performs A* search on the given roadmap graph
std::vector<int> AStarSearch(const std::map<int, std::vector<int>> &graph, int startIdx, int goalIdx, const std::vector<std::vector<double>> &roadmap, int numofDOFs);

// Splits a string based on a delimiter
std::vector<std::string> split(const std::string &str, const std::string &delim);

// Converts a comma-separated string to an array of doubles
double *doubleArrayFromString(std::string str);

// Compares two arrays of doubles for equality within a certain tolerance
bool equalDoubleArrays(double *v1, double *v2, int size);

// Checks if a line segment is valid (not colliding with obstacles) in the given map
int IsValidLineSegment(double x0, double y0, double x1, double y1, double *map, int x_size, int y_size);

// Checks if a given arm configuration is valid (not colliding with obstacles) in the given map
int IsValidArmConfiguration(double *angles, int numofDOFs, double *map, int x_size, int y_size);

#endif // PLANNER_H

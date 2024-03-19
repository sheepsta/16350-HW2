// Julius Arolovitch, for 16350 HW2, March 2024

#include <math.h>
#include <random>
#include <vector>
#include <array>
#include <algorithm>
#include <tuple>
#include <string>
#include <stdexcept>
#include <regex>	// For regex and split logic
#include <iostream> // cout, endl
#include <fstream>	// For reading/writing files
#include <queue>
#include <unordered_map>

using namespace std;

/* Input Arguments */
#define MAP_IN prhs[0]
#define ARMSTART_IN prhs[1]
#define ARMGOAL_IN prhs[2]
#define PLANNER_ID_IN prhs[3]

/* Planner Ids */
#define RRT 0
#define RRTCONNECT 1
#define RRTSTAR 2
#define PRM 3

/* Output Arguments */
#define PLAN_OUT plhs[0]
#define PLANLENGTH_OUT plhs[1]

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y * XSIZE + X)

#if !defined(MAX)
#define MAX(A, B) ((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define MIN(A, B) ((A) < (B) ? (A) : (B))
#endif

#define PI 3.141592654

// the length of each link in the arm
#define LINKLENGTH_CELLS 10

// Some potentially helpful imports
using std::array;
using std::cout;
using std::endl;
using std::make_tuple;
using std::runtime_error;
using std::string;
using std::tie;
using std::tuple;
using std::vector;

/// @brief
/// @param filepath
/// @return map, x_size, y_size
tuple<double *, int, int> loadMap(string filepath)
{
	std::FILE *f = fopen(filepath.c_str(), "r");
	if (f)
	{
	}
	else
	{
		printf("Opening file failed! \n");
		throw runtime_error("Opening map file failed!");
	}
	int height, width;
	if (fscanf(f, "height %d\nwidth %d\n", &height, &width) != 2)
	{
		throw runtime_error("Invalid loadMap parsing map metadata");
	}
	////// Go through file and add to m_occupancy
	double *map = new double[height * width];

	double cx, cy, cz;
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			char c;
			do
			{
				if (fscanf(f, "%c", &c) != 1)
				{
					throw runtime_error("Invalid parsing individual map data");
				}
			} while (isspace(c));
			if (!(c == '0'))
			{
				map[y + x * width] = 1; // Note transposed from visual
			}
			else
			{
				map[y + x * width] = 0;
			}
		}
	}
	fclose(f);
	return make_tuple(map, width, height);
}

// Splits string based on deliminator
vector<string> split(const string &str, const string &delim)
{
	// https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c/64886763#64886763
	const std::regex ws_re(delim);
	return {std::sregex_token_iterator(str.begin(), str.end(), ws_re, -1), std::sregex_token_iterator()};
}

double *doubleArrayFromString(string str)
{
	vector<string> vals = split(str, ",");
	double *ans = new double[vals.size()];
	for (int i = 0; i < vals.size(); ++i)
	{
		ans[i] = std::stod(vals[i]);
	}
	return ans;
}

bool equalDoubleArrays(double *v1, double *v2, int size)
{
	for (int i = 0; i < size; ++i)
	{
		if (abs(v1[i] - v2[i]) > 1e-3)
		{
			// cout << endl;
			return false;
		}
	}
	return true;
}

typedef struct
{
	int X1, Y1;
	int X2, Y2;
	int Increment;
	int UsingYIndex;
	int DeltaX, DeltaY;
	int DTerm;
	int IncrE, IncrNE;
	int XIndex, YIndex;
	int Flipped;
} bresenham_param_t;

void ContXY2Cell(double x, double y, short unsigned int *pX, short unsigned int *pY, int x_size, int y_size)
{
	// take the nearest cell
	double cellsize = 1.0;
	*pX = (int)(x / (double)(cellsize));
	if (x < 0)
		*pX = 0;
	if (*pX >= x_size)
		*pX = x_size - 1;

	*pY = (int)(y / (double)(cellsize));
	if (y < 0)
		*pY = 0;
	if (*pY >= y_size)
		*pY = y_size - 1;
}

void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params)
{
	params->UsingYIndex = 0;

	if (fabs((double)(p2y - p1y) / (double)(p2x - p1x)) > 1)
		(params->UsingYIndex)++;

	if (params->UsingYIndex)
	{
		params->Y1 = p1x;
		params->X1 = p1y;
		params->Y2 = p2x;
		params->X2 = p2y;
	}
	else
	{
		params->X1 = p1x;
		params->Y1 = p1y;
		params->X2 = p2x;
		params->Y2 = p2y;
	}

	if ((p2x - p1x) * (p2y - p1y) < 0)
	{
		params->Flipped = 1;
		params->Y1 = -params->Y1;
		params->Y2 = -params->Y2;
	}
	else
		params->Flipped = 0;

	if (params->X2 > params->X1)
		params->Increment = 1;
	else
		params->Increment = -1;

	params->DeltaX = params->X2 - params->X1;
	params->DeltaY = params->Y2 - params->Y1;

	params->IncrE = 2 * params->DeltaY * params->Increment;
	params->IncrNE = 2 * (params->DeltaY - params->DeltaX) * params->Increment;
	params->DTerm = (2 * params->DeltaY - params->DeltaX) * params->Increment;

	params->XIndex = params->X1;
	params->YIndex = params->Y1;
}

void get_current_point(bresenham_param_t *params, int *x, int *y)
{
	if (params->UsingYIndex)
	{
		*y = params->XIndex;
		*x = params->YIndex;
		if (params->Flipped)
			*x = -*x;
	}
	else
	{
		*x = params->XIndex;
		*y = params->YIndex;
		if (params->Flipped)
			*y = -*y;
	}
}

int get_next_point(bresenham_param_t *params)
{
	if (params->XIndex == params->X2)
	{
		return 0;
	}
	params->XIndex += params->Increment;
	if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
		params->DTerm += params->IncrE;
	else
	{
		params->DTerm += params->IncrNE;
		params->YIndex += params->Increment;
	}
	return 1;
}

int IsValidLineSegment(double x0, double y0, double x1, double y1, double *map,
					   int x_size, int y_size)
{
	bresenham_param_t params;
	int nX, nY;
	short unsigned int nX0, nY0, nX1, nY1;

	// printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);

	// make sure the line segment is inside the environment
	if (x0 < 0 || x0 >= x_size ||
		x1 < 0 || x1 >= x_size ||
		y0 < 0 || y0 >= y_size ||
		y1 < 0 || y1 >= y_size)
		return 0;

	ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
	ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);

	// printf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

	// iterate through the points on the segment
	get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
	do
	{
		get_current_point(&params, &nX, &nY);
		if (map[GETMAPINDEX(nX, nY, x_size, y_size)] == 1)
			return 0;
	} while (get_next_point(&params));

	return 1;
}

int IsValidArmConfiguration(double *angles, int numofDOFs, double *map,
							int x_size, int y_size)
{
	double x0, y0, x1, y1;
	int i;

	// iterate through all the links starting with the base
	x1 = ((double)x_size) / 2.0;
	y1 = 0;
	for (i = 0; i < numofDOFs; i++)
	{
		// compute the corresponding line segment
		x0 = x1;
		y0 = y1;
		x1 = x0 + LINKLENGTH_CELLS * cos(2 * PI - angles[i]);
		y1 = y0 - LINKLENGTH_CELLS * sin(2 * PI - angles[i]);

		// check the validity of the corresponding line segment
		if (!IsValidLineSegment(x0, y0, x1, y1, map, x_size, y_size))
			return 0;
	}
	return 1;
}

struct Node
{
	double cost;
	double *config;
	Node *parent;

	Node(double cost, double *config, Node *parent) : cost(cost), config(config), parent(parent) {}

	bool operator<(const Node &other) const
	{
		return cost > other.cost;
	}
};

double distance(double *config1, double *config2, int numofDOFs)
{
	double dist = 0;
	for (int i = 0; i < numofDOFs; ++i)
	{
		double diff = config1[i] - config2[i];
		dist += diff * diff;
	}
	return sqrt(dist);
}

void PRMPlanner(double *map, int x_size, int y_size, double *armstart_anglesV_rad,
				double *armgoal_anglesV_rad, int numofDOFs, double ***plan, int *planlength)
{
	auto start_time = std::chrono::high_resolution_clock::now();

	*plan = nullptr;
	*planlength = 0;

	int numSamples = 500;
	double connectionDistance = 10.0;
	int kNBors = 10;

	vector<double *> roadmap;
	while (true)
	{

		for (int i = 0; i < numSamples; ++i)
		{
			double *sampleConfig = new double[numofDOFs];
			for (int j = 0; j < numofDOFs; ++j)
			{
				sampleConfig[j] = (double)rand() / RAND_MAX * 2 * PI;
			}
			if (IsValidArmConfiguration(sampleConfig, numofDOFs, map, x_size, y_size))
			{
				roadmap.push_back(sampleConfig);
			}
			else
			{
				delete[] sampleConfig;
			}
		}

		roadmap.push_back(armstart_anglesV_rad);
		roadmap.push_back(armgoal_anglesV_rad);

		unordered_map<double *, vector<double *>> graph;
		for (int i = 0; i < roadmap.size(); ++i)
		{
			double *node = roadmap[i];
			vector<pair<double, double *>> nearestNborConnects;
			for (int j = 0; j < roadmap.size(); ++j)
			{
				if (i != j)
				{
					double dist = distance(node, roadmap[j], numofDOFs);
					nearestNborConnects.push_back({dist, roadmap[j]});
				}
			}
			sort(nearestNborConnects.begin(), nearestNborConnects.end());
			for (int j = 0; j < min(kNBors, (int)nearestNborConnects.size()); ++j)
			{
				graph[node].push_back(nearestNborConnects[j].second);
			}
		}

		priority_queue<Node> openSet;
		unordered_map<double *, double> costMap;
		openSet.push(Node(0.0, armstart_anglesV_rad, nullptr));
		costMap[armstart_anglesV_rad] = 0.0;
		while (!openSet.empty())
		{
			Node current = openSet.top();
			openSet.pop();
			if (equalDoubleArrays(current.config, armgoal_anglesV_rad, numofDOFs))
			{
				vector<double *> path;
				while (current.parent != nullptr)
				{
					path.push_back(current.config);
					current = *(current.parent);
				}
				path.push_back(armstart_anglesV_rad);
				reverse(path.begin(), path.end());
				*plan = new double *[path.size()];
				for (int i = 0; i < path.size(); ++i)
				{
					(*plan)[i] = path[i];
				}
				*planlength = path.size();
				auto end_time = std::chrono::high_resolution_clock::now();
				std::chrono::duration<double> elapsed_seconds = end_time - start_time;
				double planning_time = elapsed_seconds.count();

				double path_cost = 0.0;
				for (int i = 0; i < *planlength - 1; ++i)
				{
					path_cost += distance((*plan)[i], (*plan)[i + 1], numofDOFs);
				}
				std::cout << "number of samples:" << numSamples << std::endl;
				std::cout << "time taken to generate plan with PRM: " << planning_time << " secs" << std::endl;
				std::cout << "path cost with PRM: " << path_cost << std::endl;
				std::cout << "# of vertices generated w/ PRM: " << roadmap.size() << std::endl;
				return;
			}
			vector<double *> &neighbors = graph[current.config];
			for (double *neighbor : neighbors)
			{
				double newCost = current.cost + distance(current.config, neighbor, numofDOFs);
				if (!costMap.count(neighbor) || newCost < costMap[neighbor])
				{
					costMap[neighbor] = newCost;
					double priority = newCost + distance(neighbor, armgoal_anglesV_rad, numofDOFs);
					openSet.push(Node(priority, neighbor, new Node(current)));
				}
			}
		}
	}
}

bool extendRRTConnectTree(vector<double *> &RRTTree, double *randomConfig,
						  double *map, int x_size, int y_size, int numofDOFs);

double *nearestNbor(const vector<double *> &RRTTree, double *config, int numofDOFs);

void constructRRTConnectPath(const vector<double *> &RRTTreeStart, const vector<double *> &RRTTreeGoal,
							 vector<double *> &path);

void RRTConnectPlanner(double *map, int x_size, int y_size, double *armstart_anglesV_rad,
					   double *armgoal_anglesV_rad, int numofDOFs, double ***plan, int *planlength)
{
	auto start_time = std::chrono::high_resolution_clock::now();

	*plan = nullptr;
	*planlength = 0;

	int maxIterations = 100000;

	vector<double *> RRTTreeStart;
	vector<double *> RRTTreeGoal;

	RRTTreeStart.push_back(armstart_anglesV_rad);
	RRTTreeGoal.push_back(armgoal_anglesV_rad);

	for (int iter = 0; iter < maxIterations; ++iter)
	{
		double *randomConfig = new double[numofDOFs];
		for (int j = 0; j < numofDOFs; ++j)
		{
			randomConfig[j] = (double)rand() / RAND_MAX * 2 * PI;
		}

		extendRRTConnectTree(RRTTreeStart, randomConfig, map, x_size, y_size, numofDOFs);
		delete[] randomConfig;

		if (extendRRTConnectTree(RRTTreeGoal, RRTTreeStart.back(), map, x_size, y_size, numofDOFs))
		{
			vector<double *> path;
			constructRRTConnectPath(RRTTreeStart, RRTTreeGoal, path);
			if (!path.empty())
			{
				*plan = new double *[path.size()];
				for (int i = 0; i < path.size(); ++i)
				{
					(*plan)[i] = path[i];
				}
				*planlength = path.size();
				auto end_time = std::chrono::high_resolution_clock::now();
				std::chrono::duration<double> elapsed_seconds = end_time - start_time;
				double planning_time = elapsed_seconds.count();

				double path_cost = 0.0;
				for (int i = 0; i < *planlength - 1; ++i)
				{
					path_cost += distance((*plan)[i], (*plan)[i + 1], numofDOFs);
				}

				std::cout << "time taken for plan generation (RRT-Connect): " << planning_time << " s" << std::endl;
				std::cout << "path cost (RRT-Connect): " << path_cost << std::endl;
				std::cout << "# of vertices generated (RRT-Connect): " << RRTTreeStart.size() + RRTTreeGoal.size() << std::endl;
				return;
			}
		}
	}

	cout << "Failed to find a valid path within maximum iteration limit. This is an error." << endl;
}

bool extendRRTConnectTree(vector<double *> &RRTTree, double *randomConfig,
						  double *map, int x_size, int y_size, int numofDOFs)
{
	double *nearestConfig = nearestNbor(RRTTree, randomConfig, numofDOFs);

	double *newConfig = new double[numofDOFs];
	double dist = distance(nearestConfig, randomConfig, numofDOFs);
	for (int i = 0; i < numofDOFs; ++i)
	{
		newConfig[i] = nearestConfig[i] + (randomConfig[i] - nearestConfig[i]) / dist;
	}

	if (IsValidArmConfiguration(newConfig, numofDOFs, map, x_size, y_size))
	{
		RRTTree.push_back(newConfig);
		return true;
	}
	else
	{
		delete[] newConfig;
		return false;
	}
}

double *nearestNborConnect(const vector<double *> &RRTTree, double *config, int numofDOFs)
{
	double minDist = numeric_limits<double>::max();
	double *nearest = nullptr;
	for (double *node : RRTTree)
	{
		double dist = distance(node, config, numofDOFs);
		if (dist < minDist)
		{
			minDist = dist;
			nearest = node;
		}
	}
	return nearest;
}

void constructRRTConnectPath(const vector<double *> &RRTTreeStart, const vector<double *> &RRTTreeGoal,
							 vector<double *> &path)
{
	path.clear();
	for (double *node : RRTTreeStart)
	{
		path.push_back(node);
	}
	for (int i = RRTTreeGoal.size() - 1; i >= 0; --i)
	{
		path.push_back(RRTTreeGoal[i]);
	}
}

void constructRRTPath(const vector<double *> &RRTTree, double ***plan, int *planlength);
bool extendRRTTree(vector<double *> &RRTTree, double *randomConfig, double stepsize,
				   double *map, int x_size, int y_size, int numofDOFs);
double *nearestNeighborRRT(const vector<double *> &RRTTree, double *config, int numofDOFs);

void RRTPlanner(double *map, int x_size, int y_size, double *armstart_anglesV_rad,
				double *armgoal_anglesV_rad, int numofDOFs, double ***plan, int *planlength)
{
	auto start_time = std::chrono::high_resolution_clock::now();

	*plan = nullptr;
	*planlength = 0;

	double stepsize = 10.0;
	int maxIterations = 10000000;
	int goalBias = 100;

	vector<double *> RRTTree;

	RRTTree.push_back(armstart_anglesV_rad);

	for (int iter = 0; iter < maxIterations; ++iter)
	{
		double *randomConfig;
		if (iter % goalBias == 0)
		{
			randomConfig = armgoal_anglesV_rad;
		}
		else
		{
			randomConfig = new double[numofDOFs];
			for (int j = 0; j < numofDOFs; ++j)
			{
				randomConfig[j] = (double)rand() / RAND_MAX * 2 * PI;
			}
		}

		if (extendRRTTree(RRTTree, randomConfig, stepsize, map, x_size, y_size, numofDOFs))
		{
			if (equalDoubleArrays(RRTTree.back(), armgoal_anglesV_rad, numofDOFs))
			{
				constructRRTPath(RRTTree, plan, planlength);
				constructRRTPath(RRTTree, plan, planlength);
				auto end_time = std::chrono::high_resolution_clock::now();
				std::chrono::duration<double> elapsed_seconds = end_time - start_time;
				double planning_time = elapsed_seconds.count();

				double path_cost = 0.0;
				for (int i = 0; i < *planlength - 1; ++i)
				{
					path_cost += distance((*plan)[i], (*plan)[i + 1], numofDOFs);
				}
				std::cout << "time taken for plan generation (RRT): " << planning_time << " secs" << std::endl;
				std::cout << "path cost (RRT): " << path_cost << std::endl;
				std::cout << "# of vertices generated (RRT): " << RRTTree.size() << std::endl;
				return;
			}
		}

		if (randomConfig != armgoal_anglesV_rad)
		{
			delete[] randomConfig;
		}
	}

	cout << "Failed to find a valid path within maximum iteration limit. This is an error." << endl;
}

bool extendRRTTree(vector<double *> &RRTTree, double *randomConfig, double stepsize,
				   double *map, int x_size, int y_size, int numofDOFs)
{
	double *nearestConfig = nearestNbor(RRTTree, randomConfig, numofDOFs);

	double *newConfig = new double[numofDOFs];
	double dist = distance(nearestConfig, randomConfig, numofDOFs);
	if (dist > stepsize)
	{
		for (int i = 0; i < numofDOFs; ++i)
		{
			newConfig[i] = nearestConfig[i] + (randomConfig[i] - nearestConfig[i]) * stepsize / dist;
		}
	}
	else
	{
		for (int i = 0; i < numofDOFs; ++i)
		{
			newConfig[i] = randomConfig[i];
		}
	}

	bool added = false;
	if (IsValidArmConfiguration(newConfig, numofDOFs, map, x_size, y_size))
	{
		RRTTree.push_back(newConfig);
		added = true;
	}
	else
	{
		delete[] newConfig;
	}
	return added;
}

void constructRRTPath(const vector<double *> &RRTTree, double ***plan, int *planlength)
{
	vector<double *> path;
	for (double *node : RRTTree)
	{
		path.push_back(node);
	}
	*plan = new double *[path.size()];
	for (int i = 0; i < path.size(); ++i)
	{
		(*plan)[i] = path[i];
	}
	*planlength = path.size();
}

double *nearestNbor(const vector<double *> &RRTTree, double *config, int numofDOFs)
{
	double minDist = numeric_limits<double>::max();
	double *nearest = nullptr;
	for (double *node : RRTTree)
	{
		double dist = distance(node, config, numofDOFs);
		if (dist < minDist)
		{
			minDist = dist;
			nearest = node;
		}
	}
	return nearest;
}

// This needs to be updated, currently rudimentary
void ShortcutPath(double **&plan, int &planlength, double *map, int x_size, int y_size)
{
	vector<double *> shortcutPath;
	shortcutPath.push_back(plan[0]);

	double totalDistance = 0.0;

	for (int i = 1; i < planlength - 1; ++i)
	{
		if (IsValidLineSegment(shortcutPath.back()[0], shortcutPath.back()[1],
							   plan[i + 1][0], plan[i + 1][1], map, x_size, y_size))
		{
			continue;
		}
		totalDistance += distance(shortcutPath.back(), plan[i], 2);
		shortcutPath.push_back(plan[i]);
	}

	shortcutPath.push_back(plan[planlength - 1]);

	totalDistance += distance(shortcutPath.back(), plan[planlength - 1], 2);

	cout << "new cost after shortcutting: " << totalDistance << endl;

	for (int i = 0; i < planlength; ++i)
	{
		delete[] plan[i];
	}
	delete[] plan;

	plan = new double *[shortcutPath.size()];
	for (int i = 0; i < shortcutPath.size(); ++i)
	{
		plan[i] = shortcutPath[i];
	}

	planlength = shortcutPath.size();
}

int main(int argc, char **argv)
{
	double *map;
	int x_size, y_size;
	tie(map, x_size, y_size) = loadMap(argv[1]);
	const int numOfDOFs = std::stoi(argv[2]);
	double *startPos = doubleArrayFromString(argv[3]);
	double *goalPos = doubleArrayFromString(argv[4]);
	int whichPlanner = std::stoi(argv[5]);
	string outputFile = argv[6];

	if (!IsValidArmConfiguration(startPos, numOfDOFs, map, x_size, y_size))
	{
		throw runtime_error("Invalid start configuration!\n");
	}
	else if (!IsValidArmConfiguration(goalPos, numOfDOFs, map, x_size, y_size))
	{
		throw runtime_error("Invalid goal configuration!\n");
	}
	std::srand(static_cast<unsigned int>(std::time(nullptr)));

	double **plan = NULL;
	int planlength = 0;

	if (whichPlanner == PRM)
	{
		PRMPlanner(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
	}
	else if (whichPlanner == RRTCONNECT)
	{
		RRTConnectPlanner(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
	}
	else if (whichPlanner == RRT)
	{
		RRTPlanner(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
	}
	else
	{
		runtime_error("This planner hasn't been implenented yet!\n");
	}

	// ShortcutPath(plan, planlength, map, x_size, y_size);

	// for (int i = 0; i < planlength - 1; ++i)
	// {
	// 	if (!IsValidLineSegment(plan[i][0], plan[i][1], plan[i + 1][0], plan[i + 1][1], map, x_size, y_size))
	// 	{
	// 		throw runtime_error("Invalid generated plan");
	// 	}
	// }

	// Your solution's path should start with startPos and end with goalPos
	if (!equalDoubleArrays(plan[0], startPos, numOfDOFs) ||
		!equalDoubleArrays(plan[planlength - 1], goalPos, numOfDOFs))
	{
		throw std::runtime_error("Start or goal position not matching");
	}

	/** Saves the solution to output file
	 * Do not modify the output log file output format as it is required for visualization
	 * and for grading.
	 */
	std::ofstream m_log_fstream;
	m_log_fstream.open(outputFile, std::ios::trunc); // Creates new or replaces existing file
	if (!m_log_fstream.is_open())
	{
		throw std::runtime_error("Cannot open file");
	}
	m_log_fstream << argv[1] << endl; // Write out map name first
	/// Then write out all the joint angles in the plan sequentially
	for (int i = 0; i < planlength; ++i)
	{
		for (int k = 0; k < numOfDOFs; ++k)
		{
			m_log_fstream << plan[i][k] << ",";
		}
		m_log_fstream << endl;
	}
	return 0;
}

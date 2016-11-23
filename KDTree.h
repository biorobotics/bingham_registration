/*
 * File Header:
 * This file contains functions for performing KDTree Search.
 */

#ifndef KDTREE
#define KDTREE

using namespace std;
using namespace arma;

struct KDNode;
typedef struct KDNode *KDTree;

struct KDNode{
	rowvec value;
	KDTree left;
	KDTree right;
};

struct triple1{
	mat pc;
	mat pr;
	double res;
};

// call_error prints out error message and exit the program
void call_error(string msg);

/* find_distance returns the distance between two points */
double find_distance(rowvec point1, rowvec point2);

/* insert is a function that inserts a point into the KDTree */
KDTree insert(rowvec point, KDTree T);

/* find_nearest is a function that finds the point in the cloud that is nearest to the target point */
struct KDNode find_nearest(vec target, KDTree T, int size);

/*
 * kd_search returns pair<pc, pr, res>, where pc = set of all closest points
 * pr = set of all target points in corresponding order with pc
 * res = mean of all the distances calculated
 */
struct triple1 kd_search(mat targets, int numtargets, KDTree T, int size, float inlierRatio, rowvec Xreg);

#endif
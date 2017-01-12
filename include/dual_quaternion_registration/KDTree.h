/*
 * File Header:
 * This file contains functions for performing KDTree Search.
 */

#ifndef KDTREE
#define KDTREE

#include <Eigen/Dense>
#include <compute_transformed_points.h>
#include <long_double_def.h>

using std::string;

struct KDNode;
typedef struct KDNode *KDTree;

struct KDNode{
	Vector3ld value;
	KDTree left;
	KDTree right;
};

// For return type
struct KdResult{
	PointCloud pc;
	PointCloud pr;
	long double res;
};

// call_error prints out error message and exit the program
void call_error(string msg);

// insert is a function that inserts a point into the KDTree
void insert(Vector3ld point, KDTree *T);

vector<size_t> sort_indexes(const vector<long double> &v, bool ascending);

/* kd_search returns pair<pc, pr, res>, where pc = set of all closest points
 * pr = set of all target points in corresponding order with pc
 * res = mean of all the distances calculated
 */
struct KdResult* kd_search(PointCloud targets, int numtargets, KDTree T, int size, long double inlierRatio, ArrayXld Xreg);

void free_tree(KDTree T);
#endif
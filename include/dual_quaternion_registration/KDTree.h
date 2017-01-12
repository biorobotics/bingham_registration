/*
 * File Header:
 * This file contains functions for performing KDTree Search.
 */

#ifndef KDTREE
#define KDTREE

#include <Eigen/Dense>
#include <compute_transformed_points.h>

using std::string;
using Eigen::Vector3d;
using Eigen::ArrayXd;

struct KDNode;
typedef struct KDNode *KDTree;

struct KDNode{
	Vector3d value;
	KDTree left;
	KDTree right;
};

// For return type
struct KdResult{
	PointCloud pc;
	PointCloud pr;
	double res;
};

// call_error prints out error message and exit the program
void call_error(string msg);

// insert is a function that inserts a point into the KDTree
KDTree insert(Vector3d point, KDTree T);

/* kd_search returns pair<pc, pr, res>, where pc = set of all closest points
 * pr = set of all target points in corresponding order with pc
 * res = mean of all the distances calculated
 */
struct KdResult kd_search(PointCloud targets, int numtargets, KDTree T, int size, double inlierRatio, ArrayXd Xreg);

#endif
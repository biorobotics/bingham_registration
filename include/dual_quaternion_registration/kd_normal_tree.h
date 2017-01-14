/*
 * File Header:
 * This file contains functions for performing KDTree Search with the consideration
 * of normals.
 */

#ifndef KD_NORMAL_TREE
#define KD_NORMAL_TREE

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
	int index;	// Index of the value where it was originally in the ptcldFixed
};

// For return type
struct KDNormalResult{
	long double res1;  // res1 = mean of all the distances calculated
	long double res2;  // res2 = mean of 
	PointCloud pc;
	PointCloud pr;
	PointCloud normalc;
	PointCloud normalr;
};

// call_error prints out error message and exit the program
void call_error(string msg);

// insert is a function that inserts a point into the KDTree, where index is where the point 
// was in the original ptcldFixed
void insert(Vector3ld point, int index, KDTree *T);

vector<size_t> sort_indexes(const vector<long double> &v, bool ascending);

/* kd_normal_search returns the result of serach struct KDNormalResult (each filed is commented above)
 */
struct KDNormalResult* kd_search_normals(PointCloud targets, int numtargets, KDTree T, int size, 
										long double inlierRatio, VectorXld Xreg, 
										PointCloud normalMoving, PointCloud normalFixed);

void free_tree(KDTree T);
#endif
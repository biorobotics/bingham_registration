/*
 * File Header:
 * This file contains functions for performing KDTree Search.
 */

#ifndef KD_TREE
#define KD_TREE

#include "type_defs.h"

struct KDNode;
typedef struct KDNode *KDTree;
typedef struct KDNormalNode *KDNormalTree;

struct KDNode{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Vector3ld value;
	KDTree left;
	KDTree right;
	int index;	// Index of the value where it was originally in the ptcldFixed
};

struct KdResult{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	PointCloud pc;
	PointCloud pr;
	long double res;
};

struct KDNormalResult{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	long double resPoints;  // resPoints = mean of all the point distances calculated
	long double resNormals;  // resNormals = mean of all the normal distances calculated
	PointCloud pc;
	PointCloud pr;
	PointCloud normalc;
	PointCloud normalr;
};

/* insert:
 * 		Input: point (to be inserted into the tree), kd-tree (can't be NULL), 
 		Return: None. Modify the tree in place by inserting the point into the tree
 */
void insert(const Vector3ld& point, int index, KDTree *T);

/*
 * kd_search:
 		Input: target point cloud, kd-tree, inlier ratio, Xreg from last iteration
 			   to transform points 
		Return: pc = set of all closest points
				pr = set of all target points in corresponding order with pc
 				res = mean of the sum of all the distances calculated
 */
KdResult kd_search(const PointCloud& targets, const KDTree& T, long double inlierRatio, const VectorXld& Xreg);

/*
 * kd_search_normals:
 		Input: target point cloud, kd-tree, inlier ratio, Xreg from last iteration
 			   to transform points, normal moving, normal fixed
		Return: pc = set of all closest points
				pr = set of all target points in corresponding order with pc
				normalr = set of the moving normals in corresponding order with searched result
				normalc = set of the fixed normals in original order
 				resPoints = mean of the sum of all the point distances calculated
				resNormals = mean of the sum of all the normal distances calculated
 */
KDNormalResult kd_search_normals(const PointCloud& targets, const KDTree& T, 
								 long double inlierRatio, const VectorXld& Xreg, 
								 const PointCloud& normalMoving, const PointCloud& normalFixed);


KDTree tree_from_point_cloud(const PointCloud& ptcld);

long count_leaves(const KDTree& T);

// This function frees the tree
void free_tree(const KDTree& T);

#endif
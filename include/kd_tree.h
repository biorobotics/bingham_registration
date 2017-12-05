/*
 * File Header:
 * This file contains functions for performing KDTree Search.
 */

#ifndef KD_TREE
#define KD_TREE

#include "type_defs.h"
#include "search_result.h"

struct KDNode;
typedef struct KDNode *KDTree;
typedef struct KDNormalNode *KDNormalTree;

struct KDNode{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Eigen::Vector3f value;
	KDTree left;
	KDTree right;
	int index;	// Index of the value where it was originally in the ptcldFixed
};

/* insert:
 * 		Input: point (to be inserted into the tree), kd-tree (can't be NULL), 
 		Return: None. Modify the tree in place by inserting the point into the tree
 */
void insert(const Eigen::Vector3f& point, int index, KDTree& T);

/*
 * kd_search:
 		Input: target point cloud, kd-tree, inlier ratio, regParams from last iteration
 			   to transform points 
		Return: pc = set of all closest points
				pr = set of all target points in corresponding order with pc
 				res = mean of the sum of all the distances calculated
 */
SearchResult kd_search(const PointCloud& targets, const KDTree& T, double inlierRatio, const VectorXld& regParams);

KDTree tree_from_point_cloud(const PointCloud& ptcld);

long count_leaves(const KDTree& T);

// This function frees the tree
void free_tree(const KDTree& T);

#endif
/*
 * File Header:
 * This file contains functions for performing SearchTree Search.
 */

#ifndef KD_TREE
#define KD_TREE

#include "type_defs.h"
#include "search_result.h"

struct KDNode;
typedef struct KDNode *SearchTree;
typedef struct KDNormalNode *KDNormalTree;

struct KDNode{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Eigen::Vector3f value;
	SearchTree left;
	SearchTree right;
	int index;	// Index of the value where it was originally in the ptcldFixed
};

/* insert:
 * 		Input: point (to be inserted into the tree), kd-tree (can't be NULL), 
 		Return: None. Modify the tree in place by inserting the point into the tree
 */
void insert(const Eigen::Vector3f& point, int index, SearchTree& T);

/*
 * tree_search:
 		Input: target point cloud, kd-tree, inlier ratio, regParams from last iteration
 			   to transform points 
		Return: pc = set of all closest points
				pr = set of all target points in corresponding order with pc
 				res = mean of the sum of all the distances calculated
 */
SearchResult tree_search(const PointCloud& targets, const SearchTree& T);

SearchTree tree_from_point_cloud(const PointCloud& ptcld);

long count_leaves(const SearchTree& T);

// This function frees the tree
void free_tree(const SearchTree& T);

#endif
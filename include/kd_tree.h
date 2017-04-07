/*
 * File Header:
 * This file contains functions for performing KDTree Search.
 */

#ifndef KD_TREE
#define KD_TREE

#include <Eigen/Dense>
#include <type_defs.h>
#include <vector>

using std::string;
using std::vector;

struct KDNode;
typedef struct KDNode *KDTree;
typedef struct KDNormalNode *KDNormalTree;

struct KDNode{
	Vector3ld value;
	KDTree left;
	KDTree right;
};

struct KDNormalNode{
	Vector3ld value;
	KDNormalTree left;
	KDNormalTree right;
	int index;	// Index of the value where it was originally in the ptcldFixed
};

struct KdResult{
	PointCloud pc;
	PointCloud pr;
	long double res;
};

struct KDNormalResult{
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
void insert(Vector3ld point, KDTree *T);

/* insert_normal: similar to insert but insert additional information 
 * 		"index" into the tree to keep track of where the point was in original 
 *		pointcloud.
 */
void insert_normal(Vector3ld point, int index, KDNormalTree *T);

/*
 * kd_search:
 		Input: target point cloud, kd-tree, inlier ratio, Xreg from last iteration
 			   to transform points 
		Return: pc = set of all closest points
				pr = set of all target points in corresponding order with pc
 				res = mean of the sum of all the distances calculated
 */
KdResult kd_search(PointCloud *targets, KDTree T, long double inlierRatio, VectorXld *Xreg);

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
KDNormalResult kd_search_normals(PointCloud *targets, KDNormalTree T, 
								 long double inlierRatio, VectorXld *Xreg, 
								 PointCloud *normalMoving, PointCloud *normalFixed);


// This function frees the tree
void free_tree(KDTree T);

// This function frees the tree
void free_normal_tree(KDNormalTree T);

#endif
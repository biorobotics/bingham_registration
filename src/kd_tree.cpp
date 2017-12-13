/*
 * File Header for kd_tree.cpp:
 * 		This file contains functions for performing kdtree search on 3-D points 
 		with or without the aid of normals. 

 		Search steps:
 			transform the moving dataset
			kd-search for closest points in fixed dataset 
			return search result

 */
#include <limits>
#include <iostream>
#include "kd_tree.h"

KDTree tree_from_point_cloud(const PointCloud& ptcld) {
	KDTree cloudTree = NULL;
    int size = ptcld.cols();
    // Construct the kdtree from ptcldFixed
    for (int i = 0; i < size; i++) 
        insert(ptcld.col(i), i, cloudTree);
    return cloudTree;
}

/* find_distance:
 * 		Input: two points in Vector3ld type
 *      Return: distance between two points
 */
double find_distance(const Eigen::Vector3f& point1, const Eigen::Vector3f& point2) {
	return (point1 - point2).norm();
}

/* insert_helper:
 * 		Input: point (to be inserted into the tree), kd-tree (can't be NULL), 
 			   level that the point should be sorted on
 		Return: None. Modify the tree in place by inserting the point into the tree
 */
void insert_helper(const Eigen::Vector3f& point, int index, KDTree& T, int level) {
	// Right now the tree only works for x, y, z point
	if (level < 0 || level > 2){
		std::cerr << "Invalid search level";
		exit(1);
	}

	if (T == NULL)
	{
		T = new KDNode();
		T->left = NULL;
		T->right = NULL;
		(T->value)(0) = point(0);
		(T->value)(1) = point(1);
		(T->value)(2) = point(2);
		T->index = index;
	}
	else {
		if (point(level) < T->value(level)) 
			insert_helper(point, index, T->left, (level+1) % 3);
		else 
			insert_helper(point, index, T->right, (level+1) % 3);
	}
}

/* insert:
 * 		Input: point (to be inserted into the tree), kd-tree (can't be NULL), 
 		Return: None. Modify the tree in place by inserting the point into the tree
 */
void insert(const Eigen::Vector3f& point, int index, KDTree& T) {
	return insert_helper(point, index, T, 0);
}

/* find_nearest_helper:
 * 		Input: kd-tree, point (whose closest match needs to be searched in kd-tree), 
 			   the level to search, a storage for current best found, a storage for
 			   current distance
 		Return: None. Modify the found storages in place
 */
void find_nearest_helper(const KDTree& T, const Eigen::Vector3f& target, int level, const KDTree& bestN, 
						 double *bestDistance) {

	double distance, diff;
	// If reaches the leaf of the tree, end search
	if (T == NULL)
		return;


	distance = find_distance(T->value, target);
	diff = (T->value)(level) - target(level);
	
	if (distance <= *bestDistance) {
		*bestDistance = distance;
		*bestN = *T;
	}
	//If find exact match, exit early
	if (!*bestDistance) 
		return;
	
	// Our kd-tree search is for x, y, z points only
	level = (level+1) % 3;
	find_nearest_helper(diff > 0 ? T->left : T->right, target, level, bestN, bestDistance);
	//If the candidate hypersphere crosses this splitting plane, look on the
    // other side of the plane by examining the other subtree.
    if (fabs(diff) >= *bestDistance) 
    	return;
    find_nearest_helper(diff > 0 ? T->right : T->left, target, level, bestN, bestDistance);
}

/* find_nearest:
 * 		Input: point (whose closest match needs to be searched in kd-tree), kd-tree
 		Return: The sub-tree whose node is the best match
 */
KDNode *find_nearest(const Eigen::Vector3f& target, KDNode *T) {
	KDNode *bestN = new KDNode();
	double *distanceResult = (double*)malloc(sizeof(double));
	*distanceResult = std::numeric_limits<long double>::max();

	find_nearest_helper(T, target, 0, bestN, distanceResult);

	free(distanceResult);
	return bestN;
}

/*
 * kd_search:
 		Input: target point cloud, kd-tree, inlier ratio, regParams from last iteration
 			   to transform points 
		Return: pc = set of all closest points
				pr = set of all target points in corresponding order with pc
 				res = mean of the sum of all the distances calculated
 */
SearchResult kd_search(const PointCloud& targetPoints, const KDTree& T) {
	int numTargets = targetPoints.cols();
	PointCloud resultMatches = PointCloud(3, numTargets);	// First 3 rows = point, 
															// 4th row = distance

	// Find numTargets cloest points together with corresponding targets
	for (int count = 0; count < numTargets; count++) {
		KDTree nearestPoint = find_nearest(targetPoints.col(count), T);
		resultMatches.col(count) = nearestPoint->value;
		free(nearestPoint);
	}

	// Get distance row and turn into vector for sorting
	Eigen::VectorXf distances = (resultMatches - targetPoints).colwise().norm();
	
	SearchResult result;
	// When return, ignore the last column which stores individual distances
	result.matches = resultMatches;
	result.distances = distances;
	return result;
}

long count_leaves(const KDTree& T) {
	if(!T){
		return 0;
	}
	long count = 1;
	count += count_leaves(T->left);
	count += count_leaves(T->right);
	return count;
}

// This function frees the tree
void free_tree(const KDTree& T) {
	if (T) {
		free_tree(T->left);
		free_tree(T->right);
		free(T);
	}
	return;
}
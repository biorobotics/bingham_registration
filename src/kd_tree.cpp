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
#include "sort_indexes.h"
#include "conversions.h"

/* compute_transformed_points:
 *		Input: ptcld moving, regParams from previous iteration
 		Output: ptcld moving after being transformed 
 */
PointCloud compute_transformed_points(const PointCloud& ptcldMoving, const ArrayXld& regParams) {
	Matrix4ld testimated = reg_params_to_transformation_matrix (regParams.segment(0,6));
	Affine3ld t(testimated);
	return t*ptcldMoving;
}

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
long double find_distance(const Vector3ld& point1, const Vector3ld& point2) {
	return (point1 - point2).norm();
}

/* insert_helper:
 * 		Input: point (to be inserted into the tree), kd-tree (can't be NULL), 
 			   level that the point should be sorted on
 		Return: None. Modify the tree in place by inserting the point into the tree
 */
void insert_helper(const Vector3ld& point, int index, KDTree& T, int level) {
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
void insert(const Vector3ld& point, int index, KDTree& T) {
	return insert_helper(point, index, T, 0);
}

/* find_nearest_helper:
 * 		Input: kd-tree, point (whose closest match needs to be searched in kd-tree), 
 			   the level to search, a storage for current best found, a storage for
 			   current distance
 		Return: None. Modify the found storages in place
 */
void find_nearest_helper(const KDTree& T, const Vector3ld& target, int level, const KDTree& bestN, 
						 long double *bestDistance) {

	long double distance, diff;
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
KDNode *find_nearest(const Vector3ld& target, KDNode *T) {
	KDNode *bestN = new KDNode();
	long double *distanceResult = (long double*)malloc(sizeof(long double));
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
SearchResult kd_search(const PointCloud& targets_p, const KDTree& T, double inlierRatio, const VectorXld& regParams) {
	int numTargets = targets_p.cols();
	int inlierSize = trunc(numTargets * inlierRatio);	// Round down to int
	PointCloud resultTargets = PointCloud(3, numTargets);
	MatrixXld resultMatches = MatrixXld(4, numTargets);	// First 3 rows = point, 
														// 4th row = distance 
	PointCloud filtered_resultTargets = PointCloud(3, inlierSize);
	MatrixXld filtered_resultMatches = MatrixXld(4, inlierSize);
	long double totalDistance = 0;
	PointCloud targetsNew = PointCloud(3, numTargets);

	// Transform the target points before searching
	targetsNew = compute_transformed_points(targets_p, regParams);

	if (targets_p.cols() != numTargets) {
		std::cerr << "Pointcloud (" << targets_p.cols()<< ") doesn't match target size (" 
				  << numTargets << ")\n";
		exit(1);
	}

	// Find numTargets cloest points together with corresponding targets
	for (int count = 0; count < numTargets; count++) {
		KDTree nearestPoint = find_nearest(targetsNew.col(count), T);

		(resultMatches.col(count))(0) = (nearestPoint->value)(0);
		(resultMatches.col(count))(1) = (nearestPoint->value)(1);
		(resultMatches.col(count))(2) = (nearestPoint->value)(2);
		(resultMatches.col(count))(3) = find_distance(nearestPoint->value, targetsNew.col(count));
		// resultTargets.col(count) = targets_p.col(count);	// We want to return the original targets

		free(nearestPoint);
	}

	// Get distance row and turn into vector for sorting
	VectorXld distances = resultMatches.row(3);
	std::vector<long double> distancesVector;
	distancesVector.resize(distances.size());
	VectorXld::Map(&distancesVector[0], distances.size()) = distances;

	// Get indexes sorted by distance
	std::vector<unsigned int> sortIndex = sort_indexes(distancesVector, true);
	
	for (int count = 0; count < inlierSize; count++) {
		filtered_resultMatches.col(count) = resultMatches.col(sortIndex[count]);
		filtered_resultTargets.col(count) = targets_p.col(sortIndex[count]);
		totalDistance += filtered_resultMatches(3, count);
	}
	
	SearchResult result;
	// When return, ignore the last column which stores individual distances
	result.pc = filtered_resultMatches.topLeftCorner(3,filtered_resultMatches.cols());
	result.pr = filtered_resultTargets;
	result.res = totalDistance / inlierSize;
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
/*
 * File Header for kd_tree.cpp:
 * 		This file contains functions for performing kdtree search on 3-D points 
 		with or without the aid of normals. 

 		Search steps:
 			transform the moving dataset
			kd-search for closest points in fixed dataset 
			return search result

 */
#include <iostream>
#include <vector>
#include <kd_tree.h>
#include <iomanip>

using namespace Eigen;
using namespace std;

/* call_error: 
 * 		Input: error message
 *      Return: none. Prints out error message and exit the program
 */
void call_error(string msg) {
	cerr << "Error: " << msg << endl;
	exit(1);
}

/* find_distance:
 * 		Input: two points in Vector3ld type
 *      Return: distance between two points
 */
long double find_distance(Vector3ld point1, Vector3ld point2) {
	return (point1 - point2).norm();
}

/* insert_helper:
 * 		Input: point (to be inserted into the tree), kd-tree (can't be NULL), 
 			   level that the point should be sorted on
 		Return: None. Modify the tree in place by inserting the point into the tree
 */
void insert_helper(Vector3ld point, KDTree *T, int level) {
	// Right now the tree only works for x, y, z point
	if (level < 0 || level > 2) 
		call_error("Invalid search level");
	if (*T == NULL) {
		*T = (KDTree)malloc(sizeof(struct KDNode));
		if (*T == NULL)
			call_error("Malloc failed in insert_helper");
		(*T)->left = NULL;
		(*T)->right = NULL;
		((*T)->value)(0) = point(0);
		((*T)->value)(1) = point(1);
		((*T)->value)(2) = point(2);
	}
	else {
		if (point(level) < (*T)->value(level)) 
			insert_helper(point, &((*T)->left), (level+1) % 3);
		else 
			insert_helper(point, &((*T)->right), (level+1) % 3);
	}
}

/* insert:
 * 		Input: point (to be inserted into the tree), kd-tree (can't be NULL), 
 		Return: None. Modify the tree in place by inserting the point into the tree
 */
void insert(Vector3ld point, KDTree *T) {
	if (T == NULL)
		call_error("Invalid pointer for kd-tree in insert");
	return insert_helper(point, T, 0);
}

/* insert_normal_helper: similar to insert_helper but insert additional information 
 * 		"index" into the tree to keep track of where the point was in original 
 *		pointcloud.
 */
void insert_normal_helper(Vector3ld point, int index, KDNormalTree *T, int level) {
	// Right now the tree only works for x, y, z point
	if (level < 0 || level > 2) 
		call_error("Invalid search level");

	if (*T == NULL)
	{
		*T = (KDNormalTree)malloc(sizeof(struct KDNormalNode));
		if (*T == NULL)
			call_error("Malloc failed in insert_normal_helper");
		(*T)->left = NULL;
		(*T)->right = NULL;
		((*T)->value)(0) = point(0);
		((*T)->value)(1) = point(1);
		((*T)->value)(2) = point(2);
		(*T)->index = index;
	}
	else {
		if (point(level) < (*T)->value(level)) 
			insert_normal_helper(point, index, &((*T)->left), (level+1) % 3);
		else 
			insert_normal_helper(point, index, &((*T)->right), (level+1) % 3);
	}
}

/* insert_normal: similar to insert but insert additional information 
 * 		"index" into the tree to keep track of where the point was in original 
 *		pointcloud.
 */
void insert_normal(Vector3ld point, int index, KDNormalTree *T) {
	if (T == NULL)
		call_error("Invalid pointer for kd-tree in insert");
	return insert_normal_helper(point, index, T, 0);
}

/* find_nearest_helper:
 * 		Input: kd-tree, point (whose closest match needs to be searched in kd-tree), 
 			   the level to search, a storage for current best found, a storage for
 			   current distance
		Requires TreeType to be KDTree or KDNormalTree
 		Return: None. Modify the found storages in place
 */
template <class TreeType>
void find_nearest_helper(TreeType T, Vector3ld target, int level, TreeType bestN, 
						 long double *bestDistance) {
	long double distance, diff, diffSq;
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
template <class NodeType>
NodeType* find_nearest(Vector3ld target, NodeType *T) {
	NodeType *bestN = (NodeType*)malloc(sizeof(NodeType));
	
	if (!bestN)
		call_error("Malloc failed in find_nearest");
	long double *distanceResult = (long double*)malloc(sizeof(long double));
	*distanceResult = numeric_limits<long double>::max();

	find_nearest_helper(T, target, 0, bestN, distanceResult);

	//free(distanceResult);
	return bestN;
}



/* sort_indexes:
 *		Input: vector to be sorted, sorting order option (true for ascending, vice versa)
 		Return: the sorted results' index in the original vector

 * Taken from http://stackoverflow.com/questions/1577475/c-sorting-and-keeping-track-of-indexes
 */
vector<size_t> sort_indexes(const vector<long double> &v, bool ascending) {
	// initialize original index locations
	vector<size_t> idx(v.size());
	for (size_t i = 0; i < idx.size(); i++) idx[i] = i;

	// sort indexes based on comparing values in v
 	if (ascending)
  		sort(idx.begin(), idx.end(), [&v](size_t i1, size_t i2) {return v[i1] < v[i2];});
  	else
  		sort(idx.begin(), idx.end(), [&v](size_t i1, size_t i2) {return v[i1] > v[i2];});
  	
  	return idx;
}

/*
 * kd_search:
 		Input: target point cloud, kd-tree, inlier ratio, Xreg from last iteration
 			   to transform points 
		Return: pc = set of all closest points
				pr = set of all target points in corresponding order with pc
 				res = mean of the sum of all the distances calculated
 */
struct KdResult* kd_search(PointCloud *targets_p, KDTree T, long double inlierRatio, VectorXld *Xreg) {
	int numTargets = (*targets_p).cols();
	int inlierSize = trunc(numTargets * inlierRatio);	// Round down to int
	PointCloud resultTargets = PointCloud(3, numTargets);
	MatrixXld resultMatches = MatrixXld(4, numTargets);	// First 3 rows = point, 
														// 4th row = distance 
	PointCloud filtered_resultTargets = PointCloud(3, inlierSize);
	MatrixXld filtered_resultMatches = MatrixXld(4, inlierSize);
	long double totalDistance = 0;
	PointCloud targetsNew = PointCloud(3, numTargets);

	// Transform the target points before searching
	targetsNew = compute_transformed_points(*targets_p, *Xreg);

	if ((*targets_p).cols() != numTargets) {
		ostringstream errorString;
		errorString << "Pointcloud (" << (*targets_p).cols()<< ") doesn't match target size (" 
					<< numTargets << ")\n";
		call_error(errorString.str());
	}

	// Find numTargets cloest points together with corresponding targets
	for (int count = 0; count < numTargets; count++) {
		KDTree nearestPoint = find_nearest(targetsNew.col(count), T);

		(resultMatches.col(count))(0) = (nearestPoint->value)(0);
		(resultMatches.col(count))(1) = (nearestPoint->value)(1);
		(resultMatches.col(count))(2) = (nearestPoint->value)(2);
		(resultMatches.col(count))(3) = find_distance(nearestPoint->value, targetsNew.col(count));
		resultTargets.col(count) = (*targets_p).col(count);	// We want to return the original targets
	}

	// Get distance row and turn into vector for sorting
	VectorXld distances = resultMatches.row(3);
	vector<long double> distancesVector;
	distancesVector.resize(distances.size());
	VectorXld::Map(&distancesVector[0], distances.size()) = distances;

	// Get indexes sorted by distance
	vector<long unsigned int> sortIndex = sort_indexes(distancesVector, true);
	
	for (int count = 0; count < inlierSize; count++) {
		filtered_resultMatches.col(count) = resultMatches.col(sortIndex[count]);
		filtered_resultTargets.col(count) = resultTargets.col(sortIndex[count]);
		totalDistance += filtered_resultMatches(3, count);
	}
	
	struct KdResult *result = (struct KdResult*)calloc(1,sizeof(struct KdResult));
	// When return, ignore the last column which stores individual distances
	result->pc = filtered_resultMatches.topLeftCorner(3,filtered_resultMatches.cols());
	result->pr = filtered_resultTargets;
	result->res = totalDistance / inlierSize;

	return result;
}

/*
 * kd_search_normals:
 		Input: target point cloud, kd-tree, inlier ratio, Xreg from last iteration
 			   to transform points, normal moving, normal fixed
		Return: pc = set of all closest points
				pr = set of all target points in corresponding order with pc
				normalr = set of the moving normals in corresponding order with searched result
				normalc = set of the fixed normals in original order
 				res1 = mean of the sum of all the point distances calculated
				res2 = mean of the sum of all the normal distances calculated
 */
struct KDNormalResult* kd_search_normals(PointCloud *targets, KDNormalTree T, 
									long double inlierRatio, VectorXld *Xreg, 
									PointCloud *normalMoving, PointCloud *normalFixed) {
	int numTargets = (*targets).cols();
	int inlierSize = trunc(numTargets * inlierRatio);	// Round down to int
	MatrixXld resultMatches = MatrixXld(4, numTargets);	// First 3 rows = point, 
														// 4th row = distance 
	

	MatrixXld normalMatches = MatrixXld(4, numTargets);	// The corrsponding normals in 
														// normalFixed with results in resultMatches  

	PointCloud filtered_resultTargets = PointCloud(3, inlierSize);
	MatrixXld filtered_resultMatches = MatrixXld(4, inlierSize);
	MatrixXld filtered_normalMatches = MatrixXld(4, inlierSize);
	MatrixXld filtered_normalTargets = MatrixXld(3, inlierSize);

	long double totalPointDistance = 0;
	long double totalNormalDistance = 0;

	PointCloud targetsNew = PointCloud(3, numTargets);
	PointCloud normalrNew = PointCloud(3, numTargets);

	
	// Transform the target points before searching
	targetsNew = compute_transformed_points(*targets, *Xreg);
	VectorXld normalrNewTemp = VectorXld::Zero(6);

	for (int i = 3; i < 6; i++) {
		normalrNewTemp(i) = (*Xreg)(i);
	}

	normalrNew = compute_transformed_points(*normalMoving, normalrNewTemp);

	if ((*targets).cols() != numTargets){
		ostringstream errorString;
		errorString << "Pointcloud (" << (*targets).cols()<< ") doesn't match target size (" 
				    << numTargets << ")\n";
		call_error(errorString.str());
	}

	// Find numTargets closet points together with corresponding targets
	for (int count = 0; count < numTargets; count++) {
		KDNormalTree nearestPoint = find_nearest(targetsNew.col(count), T);

		(resultMatches.col(count))(0) = nearestPoint->value(0);
		(resultMatches.col(count))(1) = nearestPoint->value(1);
		(resultMatches.col(count))(2) = nearestPoint->value(2);
		(resultMatches.col(count))(3) = find_distance(nearestPoint->value, 
													  targetsNew.col(count));

		normalMatches.col(count)(0) = (*normalFixed)(0, nearestPoint->index);
		normalMatches.col(count)(1) = (*normalFixed)(1, nearestPoint->index);
		normalMatches.col(count)(2) = (*normalFixed)(2, nearestPoint->index);
		(normalMatches.col(count))(3) = find_distance((*normalFixed).col(nearestPoint->index), 
														normalrNew.col(count));
	}

	// Get distance row and turn into vector for sorting
	VectorXld distances = resultMatches.row(3);
	vector<long double> distancesVector;
	distancesVector.resize(distances.size());
	VectorXld::Map(&distancesVector[0], distances.size()) = distances;
	// Get indexes sorted by distance
	vector<long unsigned int> sortIndex = sort_indexes(distancesVector, true);
	
	
	for (int count = 0; count < inlierSize; count++) {
		filtered_resultMatches.col(count) = resultMatches.col(sortIndex[count]);
		filtered_resultTargets.col(count) = (*targets).col(sortIndex[count]);
		filtered_normalMatches.col(count) = normalMatches.col(sortIndex[count]);
		filtered_normalTargets.col(count) = (*normalMoving).col(sortIndex[count]);
		totalPointDistance += filtered_resultMatches(3, count);
		totalNormalDistance += filtered_normalMatches(3, count);
	}


	struct KDNormalResult *result = (struct KDNormalResult*)calloc(1,sizeof(struct KDNormalResult));
	// When return, ignore the last column which stores individual distances
	result->pc = filtered_resultMatches.topLeftCorner(3,filtered_resultMatches.cols());
	result->normalc = filtered_normalMatches.topLeftCorner(3,filtered_resultMatches.cols());
	result->pr = filtered_resultTargets;
	result->normalr = filtered_normalTargets;
	result->res1 = totalPointDistance / inlierSize;
	result->res2 = totalNormalDistance / inlierSize;

	return result;
}

// This function frees the tree
void free_tree(KDTree T) {
	if (T) {
		free_tree(T->left);
		free_tree(T->right);
		free(T);
	}
	return;
}
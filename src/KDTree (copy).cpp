/*
 * File Header:
 * This file contains functions for performing KDTree Search. (updated
    on 01/07/2017 for a better memory utilization)
 */
#include <iostream>
#include <vector>
#include <KDTree.h>
#include <iomanip>

using namespace Eigen;
using namespace std;

// call_error prints out error message and exit the program
void call_error(string msg) {
	cerr << "Error: " << msg << endl;
	exit(-1);
}

// find_distance returns the distance between two points
long double find_distance(Vector3ld point1, Vector3ld point2) {
	return (point1 - point2).norm();
}

// Insert is a function that inserts a point into the KDTree (which is 
// modified in place in T)
void insert_helper(Vector3ld point, KDTree *T, int level) {
	// If creating a new tree 
	if (level < 0 || level > 2) 
		call_error("Invalid component access");
	
	if (T == NULL)
		call_error("Invalid initialization");
	if (*T == NULL)
	{
		*T = (KDTree)malloc(sizeof(struct KDNode));
		if (*T == NULL)
			call_error("Failure in creating a new tree");
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

void insert(Vector3ld point, KDTree *T) {
	return insert_helper(point, T, 0);
}

/* find_nearest is a function that finds the point in the cloud that is nearest to the target point
 * (the best point is modified in place in bestPP)
 * 
 * Requirement: T be a non-empty tree, bestPP be a non-null pointer to Vector3ld * (which
   is allowed to be NULL)
 */
void find_nearest_helper(KDTree T, Vector3ld target, int level, Vector3ld **bestPP, long double *bestDistance) {
	long double distance, diff, diffSq;

	// If reaches the leaf of the tree, end search
	if (T == NULL)
		return;
	
	distance = find_distance(T->value, target);
	diff = (T->value)(level) - target(level);
	
	if (!bestPP)
		call_error("Invalid initialization");
	if (!(*bestPP) || distance < *bestDistance) {
		*bestDistance = distance;
		*bestPP = &(T->value);
	}
	//If find exact match
	if (!*bestDistance) 
		return;
	
	level = (level+1) % 3;
	find_nearest_helper(diff > 0 ? T->left : T->right, target, level, bestPP, bestDistance);
	/* If the candidate hypersphere crosses this splitting plane, look on the
    * other side of the plane by examining the other subtree.
    */
    if (fabs(diff) >= *bestDistance) 
    	return;
    
    find_nearest_helper(diff > 0 ? T->right : T->left, target, level, bestPP, bestDistance);
}

Vector3ld find_nearest(Vector3ld target, KDTree T, int size) {
	Vector3ld *bestP = NULL;
	Vector3ld **bestPP = &bestP;
	long double *distanceResult = (long double*)malloc(sizeof(long double));
	*distanceResult = numeric_limits<long double>::max();

	find_nearest_helper(T, target, 0, bestPP, distanceResult);
	free(distanceResult);
 	if (!bestPP || !bestP)
 		call_error("find_nearest didn't find result.");
 	Vector3ld result = **bestPP;
	return result;
}

/* Sort that returns the indexes in order (set ascending true for ascending order, 
 * false for descending order). Requires c++11 for lambda functions
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
 * kd_search returns pair<pc, pr, res>, where pc = set of all closest points
 * pr = set of all target points in corresponding order with pc
 * res = mean of the sum of all the distances calculated
 */
struct KdResult* kd_search(PointCloud targets, int numtargets, KDTree T, int size, long double inlierRatio, ArrayXld Xreg) {

	int inlierSize = trunc(numtargets * inlierRatio);	// Round down to int
	PointCloud resultTargets = PointCloud(3, numtargets);
	MatrixXld resultMatches = MatrixXld(4, numtargets);	// First 3 rows = point, 4th row = distance 
	PointCloud filtered_resultTargets = PointCloud(3, inlierSize);
	MatrixXld filtered_resultMatches = MatrixXld(4, inlierSize);
	long double totalDistance = 0;
	PointCloud targetsNew = PointCloud(3, numtargets);
	
	// Transform the target points before searching
	targetsNew = compute_transformed_points(targets, Xreg);

	if (targets.cols() != numtargets){
		ostringstream errorString;
		errorString << "Pointcloud (" << targets.cols()<< ") doesn't match target size (" << numtargets << ")\n";
		call_error(errorString.str());
	}

	// Find numtargets cloest points together with corresponding targets
	for (int count = 0; count < numtargets; count++) {
		Vector3ld nearestPoint = find_nearest(targetsNew.col(count), T, size);

		(resultMatches.col(count))(0) = nearestPoint(0);
		(resultMatches.col(count))(1) = nearestPoint(1);
		(resultMatches.col(count))(2) = nearestPoint(2);
		(resultMatches.col(count))(3) = find_distance(nearestPoint, targetsNew.col(count));
		resultTargets.col(count) = targets.col(count);	// We want to return the original targets
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
		//cout << "totalDistance at count: " << count << " is: " << setprecision(18) << totalDistance << endl;
	}
	
	struct KdResult *result = (struct KdResult*)calloc(1,sizeof(struct KdResult));
	// When return, ignore the last column which stores individual distances
	result->pc = filtered_resultMatches.topLeftCorner(3,filtered_resultMatches.cols());
	result->pr = filtered_resultTargets;
	//cout << "totalDistance is: " << totalDistance << endl;
	//cout << "inlierSize: " << inlierSize << endl;
	result->res = totalDistance / inlierSize;

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
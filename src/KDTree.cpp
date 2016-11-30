/*
 * File Header:
 * This file contains functions for performing KDTree Search.
 */
#include <iostream>
#include <cstdlib>
#include <cmath>
//#include <armadillo>
#include <limits>
#include <Eigen/Dense>
#include <vector>
#include <compute_transformed_points.h>

using namespace Eigen;
using namespace std;

struct KDNode;
typedef struct KDNode *KDTree;

struct KDNode{
	Vector3f value;
	KDTree left;
	KDTree right;
};

// For return type
struct triple1{
	PointCloud pc;
	PointCloud pr;
	double res;
};

// call_error prints out error message and exit the program
void call_error(string msg) {
	cerr << "Error: " << msg << endl;
	exit(-1);
}

// find_distance returns the distance between two points
double find_distance(Vector3f point1, Vector3f point2) {
	return (point1 - point2).norm();
}

// Insert is a function that inserts a point into the KDTree
KDTree insert_helper(Vector3f point, KDTree T, int level) {
	// If creating a new tree 
	if (level < 0 || level > 2)
		call_error("Invalid component access");
	if (T == NULL) {
		T = (KDTree)malloc(sizeof(struct KDNode));
		if (T == NULL)
			call_error("Failure in creating a new tree");
		T->left = T->right = NULL;
		(T->value)(0) = point(0);
		(T->value)(1) = point(1);
		(T->value)(2) = point(2);
	}
	else {
		if (point(level) < T->value(level))
			T->left = insert_helper(point, T->left, (level+1) % 3);
		else
			T->right = insert_helper(point, T->right, (level+1) % 3);
	}
	return T;
}

KDTree insert(Vector3f point, KDTree T) {
	return insert_helper(point, T, 0);
}

// find_nearest is a function that finds the point in the cloud that is nearest to the target point
void find_nearest_helper(KDTree T, Vector3f target, int level, KDTree *best, double *bestDistance) {
	double distance, diff, diffSq;

	// If reaches the leaf of the tree, end search
	if (T == NULL){
		return;
	}
	distance = find_distance(T->value, target);
	diff = (T->value)(level) - target(level);
	if (!best)
		call_error("Best pointer should not be null");
	if (!*best || distance < *bestDistance) {
		*bestDistance = distance;
		*best = T;
	}

	//If find exact match
	if (!*bestDistance)
		return;

	level = (level+1) % 3;
	find_nearest_helper(diff > 0 ? T->left : T->right, target, level, best, bestDistance);
	/* If the candidate hypersphere crosses this splitting plane, look on the
    * other side of the plane by examining the other subtree.
    */
    if (fabs(diff) >= *bestDistance)
    	return;
    find_nearest_helper(diff > 0 ? T->right : T->left, target, level, best, bestDistance);
}

struct KDNode find_nearest(Vector3f target, KDTree T, int size) {
	KDTree temp = NULL;
	KDTree *bestResult = &temp;
	double *distanceResult = (double*)malloc(sizeof(double));
	*distanceResult = numeric_limits<double>::max();
	find_nearest_helper(T, target, 0, bestResult, distanceResult);
	return **bestResult;
}

// Sort that returns the indexes in order. Requires c++11 for lambda functions
// Taken from http://stackoverflow.com/questions/1577475/c-sorting-and-keeping-track-of-indexes
template <typename T>
vector<size_t> sort_indexes(const vector<T> &v) {

  // initialize original index locations
  vector<size_t> idx(v.size());
  for (size_t i = 0; i != idx.size(); ++i) idx[i] = i;

  // sort indexes based on comparing values in v
  sort(idx.begin(), idx.end(), [&v](size_t i1, size_t i2) {return v[i1] < v[i2];});

  return idx;
}

/*
 * kd_search returns pair<pc, pr, res>, where pc = set of all closest points
 * pr = set of all target points in corresponding order with pc
 * res = mean of all the distances calculated
 */
struct triple1 kd_search(PointCloud targets, int numtargets, KDTree T, int size, float inlierRatio, ArrayXf Xreg) {

	int inlierSize = trunc(numtargets * inlierRatio);	// Round down to int
	PointCloud resultTargets = PointCloud(3, numtargets);
	MatrixXf resultMatches = MatrixXf(4, numtargets);	// First 3 rows = point, 4th row = distance 
	PointCloud filtered_resultTargets = PointCloud(3, inlierSize);
	MatrixXf filtered_resultMatches = MatrixXf(4, inlierSize);
	double totalDistance;
	PointCloud targetsNew = PointCloud(3, numtargets);
	
	// Transform the target points before searching
	targetsNew = compute_transformed_points(targets, Xreg);

	if (targets.size()/3 != numtargets)
		call_error("target doesn't match target size");
	// Find numtargets cloest points together with corresponding targets
	for (int count = 0; count < numtargets; count++) {
		struct KDNode nearestPoint = find_nearest(targetsNew.col(count), T, size);
		(resultMatches.col(count))(0) = (nearestPoint.value)(0);
		(resultMatches.col(count))(1) = (nearestPoint.value)(1);
		(resultMatches.col(count))(2) = (nearestPoint.value)(2);
		(resultMatches.col(count))(3) = find_distance(nearestPoint.value, targetsNew.col(count));
		resultTargets.col(count) = targets.col(count);	// We want to return the original targets
		}
	
	// Get distance row and turn into vector for sorting
	VectorXf distances = resultMatches.row(3);
	vector<float> distancesVector;
	distancesVector.resize(distances.size());
	VectorXf::Map(&distancesVector[0], distances.size()) = distances;
	// Get indexes sorted by distance
	vector<long unsigned int> sortIndex = sort_indexes(distancesVector);
	
	for (int count = 0; count < inlierSize; count++) {
		filtered_resultMatches.col(count) = resultMatches.col(sortIndex[count]);
		filtered_resultTargets.col(count) = resultTargets.col(sortIndex[count]);
		totalDistance += filtered_resultMatches(count, 3);
	}
	
	struct triple1 result;
	// When return, ignore the last column which stores individual distances
	result.pc = filtered_resultMatches.topLeftCorner(3, 2);
	result.pr = filtered_resultTargets;
	result.res = totalDistance / inlierSize;
	return result;
}
/*
 * File Header:
 * This file contains functions for performing KDTree Search.
 */

#include <iostream>
#include <cstdlib>
#include <cmath>
#include <armadillo>
#include <limits>
#include "compute_transformed_points.h"

using namespace std;
using namespace arma;

struct KDNode;
typedef struct KDNode *KDTree;

struct KDNode{
	rowvec value;
	KDTree left;
	KDTree right;
};

// For return type
struct triple1{
	mat pc;
	mat pr;
	double res;
};

// call_error prints out error message and exit the program
void call_error(string msg) {
	cerr << "Error: " << msg << endl;
	exit(-1);
}

/* find_distance returns the distance between two points */
double find_distance(rowvec point1, rowvec point2) {
	double sum = 0;
	for (int i = 0; i < 3; i++) {
		sum += pow(point1(i) - point2(i), 2);
	}
	return sqrt(sum);
}

/* Insert is a function that inserts a point into the KDTree */
KDTree insert_helper(rowvec point, KDTree T, int level) {
	/* If creating a new tree */
	if (level < 0 || level > 2)
		call_error("Invalid component access");
	if (T == NULL) {
		T = (KDTree)malloc(sizeof(struct KDNode));
		if (T == NULL)
			call_error("Failure in creating a new tree");
		T->left = T->right = NULL;
		T->value = rowvec(3);
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

KDTree insert(rowvec point, KDTree T) {
	return insert_helper(point, T, 0);
}

/* find_nearest is a function that finds the point in the cloud that is nearest to the target point */
void find_nearest_helper(KDTree T, rowvec target, int level, KDTree *best, double *bestDistance) {
	double distance, diff, diffSq;

	/* If reaches the leaf of the tree, end search */
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

	/* If find exact match */
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

struct KDNode find_nearest(rowvec target, KDTree T, int size) {
	KDTree temp = NULL;
	KDTree *bestResult = &temp;
	double *distanceResult = (double*)malloc(sizeof(double));
	*distanceResult = numeric_limits<double>::max();
	find_nearest_helper(T, target, 0, bestResult, distanceResult);
	return **bestResult;
}

/*
 * kd_search returns pair<pc, pr, res>, where pc = set of all closest points
 * pr = set of all target points in corresponding order with pc
 * res = mean of all the distances calculated
 */
struct triple1 kd_search(mat targets, int numtargets, KDTree T, int size, float inlierRatio, rowvec Xreg) {

	int inlierSize = trunc(numtargets * inlierRatio);	// Round down to int
	mat resultTargets = mat(numtargets, 3);
	mat resultMatches = mat(numtargets, 4);	// First 3 cols = point, 4th col = distance 
	mat filtered_resultTargets = mat(inlierSize, 3);
	mat filtered_resultMatches = mat(inlierSize, 4);
	double totalDistance;
	mat targetsNew = mat(numtargets, 3);

	// Transform the target points before searching
	targetsNew = compute_transformed_points(targets, Xreg);

	if (targets.n_rows != numtargets)
		call_error("target doesn't match target size");
	// Find numtargets cloest points together with corresponding targets
	for (int count = 0; count < numtargets; count++) {
		struct KDNode nearestPoint = find_nearest(targetsNew.row(count), T, size);
		(resultMatches.row(count))(0) = (nearestPoint.value)(0);
		(resultMatches.row(count))(1) = (nearestPoint.value)(1);
		(resultMatches.row(count))(2) = (nearestPoint.value)(2);
		(resultMatches.row(count))(3) = find_distance(nearestPoint.value, targetsNew.row(count));
		resultTargets.row(count) = targets.row(count);	// We want to return the original targets
		}

	// Sort from shortest to longest distance
	uvec sortIndex = sort_index(resultMatches.col(3));

	for (int count = 0; count < inlierSize; count++) {
		filtered_resultMatches.row(count) = resultMatches.row(sortIndex(count));
		filtered_resultTargets.row(count) = resultTargets.row(sortIndex(count));
		totalDistance += filtered_resultMatches(count, 3);
	}

	struct triple1 result;
	// When return, ignore the last column which stores individual distances
	result.pc = filtered_resultMatches.submat(0, 0, filtered_resultMatches.n_rows-1, 2);
	result.pr = filtered_resultTargets;
	result.res = totalDistance / inlierSize;
	return result;
}
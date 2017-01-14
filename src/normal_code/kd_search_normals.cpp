/*
 * File Header:
 * This file contains functions for performing KDTree Search with consideration
   of normal. (updated on 01/13/2017 for normal implementation)
 */
#include <iostream>
#include <vector>
#include <kd_normal_tree.h>
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

/* Insert is a function that inserts a point into the KDTree (which is 
 * modified in place in T). Also record the index where the point is
 * in the original ptcldFixed for the matching normal extraction in kd_search_normals
 */
void insert_helper(Vector3ld point, int index, KDTree *T, int level) {
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
		(*T)->index = index;
	}
	else {
		if (point(level) < (*T)->value(level)) 
			insert_helper(point, index, &((*T)->left), (level+1) % 3);
		else 
			insert_helper(point, index, &((*T)->right), (level+1) % 3);
	}
}

void insert(Vector3ld point, int index, KDTree *T) {
	return insert_helper(point, index, T, 0);
}

/* find_nearest is a function that finds the point in the cloud that is nearest to the target point
 * (the best point is modified in place in bestPP)
 * 
 * Requirement: T be a non-empty tree, bestPP be a non-null pointer to Vector3ld * (which
   is allowed to be NULL)
 */
void find_nearest_helper(KDTree T, Vector3ld target, int level, KDTree bestN, long double *bestDistance) {
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
	//If find exact match
	if (!*bestDistance) 
		return;
	
	level = (level+1) % 3;
	find_nearest_helper(diff > 0 ? T->left : T->right, target, level, bestN, bestDistance);
	/* If the candidate hypersphere crosses this splitting plane, look on the
    * other side of the plane by examining the other subtree.
    */
    if (fabs(diff) >= *bestDistance) 
    	return;
    
    find_nearest_helper(diff > 0 ? T->right : T->left, target, level, bestN, bestDistance);
}

struct KDNode find_nearest(Vector3ld target, KDTree T, int size) {
	KDTree bestN = (KDTree)malloc(sizeof(struct KDNode));
	if (!bestN)
		call_error("malloc failed.");
	long double *distanceResult = (long double*)malloc(sizeof(long double));
	*distanceResult = numeric_limits<long double>::max();

	find_nearest_helper(T, target, 0, bestN, distanceResult);
	free(distanceResult);

	return *bestN;
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
struct KDNormalResult* kd_search_normals(PointCloud targets, int numtargets, KDTree T, int size, 
									long double inlierRatio, VectorXld Xreg, 
									PointCloud normalMoving, PointCloud normalFixed) {
clock_t start_14 = clock();
	int inlierSize = trunc(numtargets * inlierRatio);	// Round down to int
	MatrixXld resultMatches = MatrixXld(4, numtargets);	// First 3 rows = point, 4th row = distance 
	MatrixXld normalMatches = MatrixXld(4, numtargets);	// The corrsponding normals in normalFixed with results
														// in resultMatches  

	PointCloud filtered_resultTargets = PointCloud(3, inlierSize);
	MatrixXld filtered_resultMatches = MatrixXld(4, inlierSize);
	MatrixXld filtered_normalMatches = MatrixXld(4, inlierSize);
	MatrixXld filtered_normalTargets = MatrixXld(3, inlierSize);

	long double totalPointDistance = 0;
	long double totalNormalDistance = 0;

	PointCloud targetsNew = PointCloud(3, numtargets);
	PointCloud normalrNew = PointCloud(3, numtargets);

	
	// Transform the target points before searching

	targetsNew = compute_transformed_points(targets, Xreg);
	/*VectorXld normalrNewTemp = VectorXld::Zero(6);

	for (int i = 3; i < 6; i++) {
		normalrNewTemp(i) = Xreg(i);
	}*/


	clock_t normal_compute_start = clock();
	long double crz = cos(Xreg(3));
	long double cry = cos(Xreg(4));
	long double crx = cos(Xreg(5));

	long double srz = sin(Xreg(3));
	long double sry = sin(Xreg(4));
	long double srx = sin(Xreg(5));

	Matrix3ld TComb;
	TComb << cry*crz, -crx*srz+srx*sry*crz, srx*srz+crx*sry*crz,
	         cry*srz, crx*crz+srx*sry*srz, -srx*crz+crx*sry*srz,
	         -sry, srx*cry, crx*cry;

	//cout << "normalMoving size is: " << normalMoving.rows() << " x " << normalMoving.cols() << endl;
	normalrNew = (TComb * normalMoving);
	clock_t normal_compute_end = clock();
	long double elapsed_secs_normal = (long double)(normal_compute_end - normal_compute_start) / CLOCKS_PER_SEC;
	//cout << "Normal compute part takes: " << elapsed_secs_normal << " seconds." << endl;
	//cout << "Here" << endl;
	//normalrNew = compute_transformed_points(normalMoving, normalrNewTemp);



	if (targets.cols() != numtargets){
		ostringstream errorString;
		errorString << "Pointcloud (" << targets.cols()<< ") doesn't match target size (" << numtargets << ")\n";
		call_error(errorString.str());
	}


	// Find numtargets closet points together with corresponding targets
	for (int count = 0; count < numtargets; count++) {
		struct KDNode nearestPoint = find_nearest(targetsNew.col(count), T, size);

		(resultMatches.col(count))(0) = nearestPoint.value(0);
		(resultMatches.col(count))(1) = nearestPoint.value(1);
		(resultMatches.col(count))(2) = nearestPoint.value(2);
		(resultMatches.col(count))(3) = find_distance(nearestPoint.value, targetsNew.col(count));
		
		normalMatches.col(count)(0) = normalFixed(0, nearestPoint.index);
		normalMatches.col(count)(1) = normalFixed(1, nearestPoint.index);
		normalMatches.col(count)(2) = normalFixed(2, nearestPoint.index);
		(normalMatches.col(count))(3) = find_distance(normalFixed.col(nearestPoint.index), normalrNew.col(count));
	}
	
	/*clock_t end = clock();
	long double elapsed_secs = (long double)(end - start) / CLOCKS_PER_SEC; 
	cout << "First part of kd_search takes: " << elapsed_secs << endl;*/

	// Get distance row and turn into vector for sorting

	VectorXld distances = resultMatches.row(3);
	vector<long double> distancesVector;
	distancesVector.resize(distances.size());
	VectorXld::Map(&distancesVector[0], distances.size()) = distances;
	// Get indexes sorted by distance
	vector<long unsigned int> sortIndex = sort_indexes(distancesVector, true);
	
	
	for (int count = 0; count < inlierSize; count++) {
		filtered_resultMatches.col(count) = resultMatches.col(sortIndex[count]);
		filtered_resultTargets.col(count) = targets.col(sortIndex[count]);

		filtered_normalMatches.col(count) = normalMatches.col(sortIndex[count]);
		filtered_normalTargets.col(count) = normalMoving.col(sortIndex[count]);
		totalPointDistance += filtered_resultMatches(3, count);
		totalNormalDistance += filtered_normalMatches(3, count);
		//cout << "totalPointDistance at count: " << count << " is: " << setprecision(18) << totalPointDistance << endl;
	}

	struct KDNormalResult *result = (struct KDNormalResult*)calloc(1,sizeof(struct KDNormalResult));
	// When return, ignore the last column which stores individual distances
	result->pc = filtered_resultMatches.topLeftCorner(3,filtered_resultMatches.cols());
	result->normalc = filtered_normalMatches.topLeftCorner(3,filtered_resultMatches.cols());
	result->pr = filtered_resultTargets.topLeftCorner(3,filtered_resultMatches.cols());
	result->normalr = filtered_normalTargets.topLeftCorner(3,filtered_resultMatches.cols());
	//cout << "totalPointDistance is: " << totalPointDistance << endl;
	//cout << "inlierSize: " << inlierSize << endl;
	result->res1 = totalPointDistance / inlierSize;
	//result->res2 = totalNormalDistance / inlierSize;
	result->res2 = totalNormalDistance / inlierSize;

		clock_t end_14= clock();
	long double elapsed_secs_14 = (long double)(end_14 - start_14) / CLOCKS_PER_SEC; 
	//cout << "This entire chunk takes: " << elapsed_secs_14 << endl;
	//cout << "Size of result is: " << sizeof(result) << endl;
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
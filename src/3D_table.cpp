/*
 * File Header for 3d_Table.cpp:
 * 		This file contains functions for performing 3DTable search on 3-D points 
 		Search steps:
            store the fixed dataset into a KDTree first
            store values of the KDTree into a PointCloud that represents a 3DTable to allow for constant-time access
 			transform the moving dataset
			search for closest points in 3DTable
			return search result
 */

#define NOMINMAX
#include <limits>
#include <3D_table.h>
#include <registration_tools.h>
#include <compute_transformed_points.h>
#include <kd_tree.h>
#include <cmath>

using namespace Eigen;
using namespace std;


/* compute_boundary:
 * 		Input: target pointcloud, ratio for the boundary
 * 		Return: 3D boundary of the pointcloud magnified by a factor of the ratio
 */

KDBoundary compute_boundary(PointCloud *targets, int ratio){
	long double XL = 0, XR = 1, YL = 0, YR = 1, ZL = 0, ZR = 1;
	KDBoundary result = (KDBoundary)malloc(sizeof(struct bound3D));
	result->XL = XL, result->XR = XR;
	result->YL = YL, result->YR = YR;
	result->ZL = ZL, result->ZR = ZR;
	int numTargets = (*targets).cols();

	if (numTargets == 0){
		return result;
	}

	//initialize
	XL = ((*targets).col(0))(0), XR = ((*targets).col(0))(0);
	YL = ((*targets).col(0))(1), YR = ((*targets).col(0))(1);
	ZL = ((*targets).col(0))(2), ZR = ((*targets).col(0))(2);

	for (int i = 0; i<numTargets; i++){
		Vector3ld temp = (*targets).col(i);
		if (XL > temp(0)){
			XL = temp(0);
		}
		else if(XR < temp(0)){
			XR = temp(0); 
		}
		if (YL > temp(1)){
			YL = temp(1);
		}
		else if(YR < temp(1)){
			YR = temp(1); 
		}
		if (ZL > temp(2)){
			ZL = temp(2);
		}
		else if(ZR < temp(2)){
			ZR = temp(2); 
		}
	}

	long double xWidth = max(abs(XL), abs(XR));
	long double yWidth = max(abs(YL), abs(YR));
	long double zWidth = max(abs(ZL), abs(ZR));
	/*long double Xmid = (XR + XL)/2;
	long double Ymid = (YR + YL)/2;
	long double Zmid = (ZR + ZL)/2;*/

	//result boundary
	result->XL = 0 - xWidth * ratio;
	result->XR = 0 + xWidth * ratio;

	result->YL = 0 - yWidth * ratio;
	result->YR = 0 + yWidth * ratio;

	result->ZL = 0 - zWidth * ratio;
	result->ZR = 0 + zWidth * ratio;

	return result;
}



/* accessTable:
 * 		Input: index i, j, k, and the dimension of the table (PointCloud)
 * 		Output: the index in the table corresponding to i, j, k values.
 */

long long accessTable(int i, int j, int k, int dim){
	long long result = 0;
	result += k*dim*dim;
	result += i*dim;
	result += j;
	return result;
}

/* generateTable:
 * 		Input: 3D boundary, and the specified dimension of the table
 * 		Output: a PointCloud corresponding to a 3D table, with each column initialized to the corresponding (x,y,z) coordinates
 */

PointCloud generateTable(KDBoundary boundary, int dim){

	long long totalDim = (dim*dim*dim);
	PointCloud result (3, totalDim);
	for (int k = 0; k<dim; k++){
		for (int i = 0; i<dim; i++){
			for (int j = 0; j<dim; j++){
				long long a = accessTable(i, j, k, dim);
				result(0, a) = (boundary->XL) + ((boundary->XR - boundary->XL)/dim) * (i+0.5);
				result(1, a) = (boundary->YL) + ((boundary->YR - boundary->YL)/dim) * (j+0.5);
				result(2, a) = (boundary->ZL) + ((boundary->ZR - boundary->ZL)/dim) * (k+0.5);
			}
		}
		
	}
	return result;
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


/* updateTable:
 * 		Input: an initalized PointCloud table, KDTree of the points that need to be mapped
 * 		Output: a PointCloud that maps the indexes in the tree correspondingly
 */

PointCloud fillTable(PointCloud* table, KDTree T) {
	int numTargets = (*table).cols();	// Round down to int
	
	MatrixXld resultMatches = MatrixXld(3, numTargets);	// First 3 rows = point,
	
	// Transform the target points before searching

	if ((*table).cols() != numTargets) {
		ostringstream errorString;
		errorString << "Pointcloud (" << (*table).cols()<< ") doesn't match target size (" 
					<< numTargets << ")\n";
		call_error(errorString.str());
	}
	long double inlierRatio = 1;
	VectorXld Xreg = VectorXld::Zero(6);  //Xreg: 6x1

	struct KdResult* searchResult = kd_search(table, T, inlierRatio, &Xreg);
	PointCloud pc = searchResult->pcUnsorted;

	// Find numTargets cloest points together with corresponding targets
	/*for (int count = 0; count < numTargets; count++) {
		KDTree nearestPoint = find_nearest((*table).col(count), T);
		(resultMatches.col(count))(0) = (nearestPoint->value)(0);
		(resultMatches.col(count))(1) = (nearestPoint->value)(1);
		(resultMatches.col(count))(2) = (nearestPoint->value)(2);
	}
	return resultMatches;*/
	return pc;
}


//Finding Stage

/* finder:
 * 		Input: a point in 3D, PointCloud table, boundary of the table, dimension of the table
 * 		Output: the closest point in the PointCloud table that corresponds to the point
 */

Vector3ld finder(Vector3ld target, PointCloud *T, KDBoundary boundary, int dim){
	int indX = 0;
	int indY = 0;
	int indZ = 0;

	if (target(0) <= boundary->XL){
		indX = 0;
	}
	else if(target(0) >= boundary->XR){
		indX = dim-1;
	} 
	else{
		indX = (int)(((target(0) - boundary->XL) * dim / (boundary->XR - boundary->XL)));
		//add 0.5 for rounding purposes
	}
	if (target(1) <= boundary->YL){
		indY = 0;
	}
	else if(target(1) >= boundary->YR){
		indY = dim-1;
	}
	else{
		indY = (int)(((target(1) - boundary->YL) * dim / (boundary->YR - boundary->YL)));
	}
	if (target(2) <= boundary->ZL){
		indZ = 0;
	}
	else if(target(2) >= boundary->ZR){
		indZ = dim-1;
	}
	else{
		indZ = (int)(((target(2) - boundary->ZL) * dim / (boundary->ZR - boundary->ZL)));
	}
	return (*T).col(accessTable(indX, indY, indZ, dim));
}

/*
 * table_search:
 		Input: target point cloud, 3D table, inlier ratio, Xreg from last iteration
 			   to transform points, boundary of the point cloud, dimension of the point cloud
		Return: pc = set of all closest points
				pr = set of all target points in corresponding order with pc
 				res = mean of the sum of all the distances calculated
 */

struct TableResult* table_search(PointCloud *targets_p, PointCloud *T, long double inlierRatio, VectorXld *Xreg, KDBoundary boundary, int dim) {
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
		Vector3ld nearestPoint = finder(targetsNew.col(count), T, boundary, dim);

		(resultMatches.col(count))(0) = (nearestPoint)(0);
		(resultMatches.col(count))(1) = (nearestPoint)(1);
		(resultMatches.col(count))(2) = (nearestPoint)(2);
		(resultMatches.col(count))(3) = find_distance(nearestPoint, targetsNew.col(count));
		resultTargets.col(count) = (*targets_p).col(count);	// We want to return the original targets
	}

	// Get distance row and turn into vector for sorting
	VectorXld distances = resultMatches.row(3);
	vector<long double> distancesVector;
	distancesVector.resize(distances.size());
	VectorXld::Map(&distancesVector[0], distances.size()) = distances;

	// Get indexes sorted by distance
	vector<unsigned int> sortIndex = sort_indexes(distancesVector, true);
	
	for (int count = 0; count < inlierSize; count++) {
		filtered_resultMatches.col(count) = resultMatches.col(sortIndex[count]);
		filtered_resultTargets.col(count) = resultTargets.col(sortIndex[count]);
		totalDistance += filtered_resultMatches(3, count);
	}
	
	struct TableResult *result = (struct TableResult*)calloc(1,sizeof(struct TableResult));
	// When return, ignore the last column which stores individual distances
	result->pc = filtered_resultMatches.topLeftCorner(3,filtered_resultMatches.cols());
	result->pr = filtered_resultTargets;
	result->res = totalDistance / inlierSize;

	return result;
}



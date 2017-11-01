/*
 * File Header:
 * This file contains functions for performing 3DTable Search.
 */

#ifndef TABLE
#define TABLE

#include <Eigen/Dense>
#include <type_defs.h>
#include <vector>
#include <kd_tree.h>

using std::string;
using std::vector;

typedef struct bound3D *KDBoundary;

//struct for a 3-dimensional boundary
struct bound3D{
	long double XL;
	long double XR; //has to be greater than or equal to XL
	long double YL;
	long double YR; //has to be greater than or equal to YL
	long double ZL;
	long double ZR; //has to be greater than or equal to ZL
};


struct TableResult{
	PointCloud pc;
	PointCloud pr;
	PointCloud pcUnsorted;
	PointCloud prUnsorted;
	long double res;
};



/* compute_boundary:
 * Input: target pointcloud, ratio for the boundary
 * Return: 3D boundary of the pointcloud magnified by a factor of the ratio
 */

KDBoundary compute_boundary(PointCloud *targets, int ratio);


/* generateTable:
 * 		Input: 3D boundary, and the specified dimension of the table
 * 		Output: a PointCloud corresponding to a 3D table, with each column initialized to the corresponding (x,y,z) coordinates
 */

PointCloud generateTable(KDBoundary boundary, int dim);

/* fillTable:
 * 		Input: an initalized PointCloud table, KDTree of the points that need to be mapped
 * 		Output: a PointCloud that maps the indexes in the tree correspondingly
 */

PointCloud fillTable(PointCloud* table, KDTree T);

/*
 * table_search:
 		Input: target point cloud, 3D table, inlier ratio, Xreg from last iteration
 			   to transform points, boundary of the point cloud, dimension of the point cloud
		Return: pc = set of all closest points
				pr = set of all target points in corresponding order with pc
 				res = mean of the sum of all the distances calculated
 */

struct TableResult* table_search(PointCloud *targets_p, PointCloud *T, long double inlierRatio, VectorXld *Xreg, KDBoundary boundary, int dim);

#endif

/*
 * File Header:
 * This file contains functions for performing KDTree Search.
 */

#ifndef KDTREE
#define KDTREE

#include <Eigen/Dense>
#include <compute_transformed_points.h>

using std::string;
using Eigen::Vector3d;
using Eigen::ArrayXd;

struct KDNode;
typedef struct KDNode *KDTree;

struct triple1;

// call_error prints out error message and exit the program
void call_error(string msg);

/* find_distance returns the distance between two points */
double find_distance(Vector3d point1, Vector3d point2);

/* insert is a function that inserts a point into the KDTree */
KDTree insert(Vector3d point, KDTree T);

/* find_nearest is a function that finds the point in the cloud that is nearest to the target point */
struct KDNode find_nearest(Vector3d target, KDTree T, int size);

/*
 * kd_search returns pair<pc, pr, res>, where pc = set of all closest points
 * pr = set of all target points in corresponding order with pc
 * res = mean of all the distances calculated
 */
struct triple1 kd_search(PointCloud targets, int numtargets, KDTree T, int size, double inlierRatio, ArrayXd Xreg);

#endif
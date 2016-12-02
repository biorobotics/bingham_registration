#include <iostream>
#include <math.h>
#include <compute_transformed_points.h>
#include <KDTree.h>
#include <chrono>
#include <qr_kf.h>
using namespace std;
using namespace std::chrono;

int main() {
	cout << "REGISTRATION TEST FUNCTIONS" << ".\n\n";

	cout << "Testing euler to rotation matrix conversion" << "\n";
	Array3d e(M_PI,0,0);
	Matrix4d testEuler = eul2rotm(e);
	Affine3d t;
	t.matrix() = testEuler;
	AngleAxisd a;
	a.fromRotationMatrix(t.rotation());
	cout << "Angle: "<< a.angle() << ".\n";
	cout << "Axis: " << a.axis().transpose() << "\n\n";

	cout << "Testing regular params to rotation matrix" << ".\n";

	ArrayXd regParams(6);
	regParams << 1, 0, 0, 0, M_PI, 0;
	cout <<  reg_params_to_transformation_matrix(regParams) << "\n\n";

	cout << "Testing point cloud transforms" << ".\n";
	PointCloud testPointCloud(3,5);
	testPointCloud.row(0) << 1, 0,   0,  .5, -10;
	testPointCloud.row(1) << 0, 1,   0,   0,   0;
	testPointCloud.row(2) << 0, 0, 150, -.1,   0;

	cout << testPointCloud << "\n";
	cout << "   =====================" << "\n";
	cout << compute_transformed_points(testPointCloud,regParams) << "\n\n";

	PointCloud testPointCloudMoving(3,1);
	testPointCloud.row(0) << 1;
	testPointCloud.row(1) << 0;
	testPointCloud.row(2) << 0;

	cout << "Testing kdTree" << ".\n";
	int sizePtcldMoving = testPointCloudMoving.size()/3;
	int sizePtcldFixed = testPointCloud.size()/3;
	int treeSize = sizePtcldMoving;
	KDTree cloudTree = NULL;	// Generated kd tree from ptcldFixed
	ArrayXd tolerance;

	// Construct the kdtree from ptcldFixed
	for (int i = 0; i < treeSize; i++) {
		cloudTree = insert(testPointCloud.row(i), cloudTree);
	}

	cout << "Testing quaternion to euler conversion" << ".\n";
	Quaterniond quat = Quaterniond(.924, 0, .382, 0);
	cout << quat2eul(quat) << "\n\n";
	
	/*
	Array3d eul;
	Matrix3d mat;
	Quaterniond quat = Quaterniond(.924, 0, .382, 0);
	high_resolution_clock::time_point t1 = high_resolution_clock::now();
    for(int idx = 0; idx<5000000; idx++) mat = quat2rotm(quat);
    high_resolution_clock::time_point t2 = high_resolution_clock::now();

    auto duration = duration_cast<microseconds>( t2 - t1 ).count();

    cout << duration << "\n";

    t1 = high_resolution_clock::now();
    for(int idx = 0; idx<5000000; idx++) mat = quat.matrix();
    t2 = high_resolution_clock::now();

    duration = duration_cast<microseconds>( t2 - t1 ).count();

    cout << duration << "\n\n";

    cout << eul << "\n";
    cout << mat << "\n";
	*/
    return 0;
}
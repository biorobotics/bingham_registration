#include <iostream>
#include <math.h>
#include <compute_transformed_points.h>
#include <chrono>
using namespace std;
using namespace std::chrono;

int main() {
	cout << "REGISTRATION TEST FUNCTIONS" << ".\n\n";

	cout << "Testing euler to rotation matrix conversion" << "\n";
	Array3f e(M_PI,0,0);
	Matrix4f testEuler = eul2rotm(e);
	Affine3f t;
	t.matrix() = testEuler;
	AngleAxisf a;
	a.fromRotationMatrix(t.rotation());
	cout << "Angle: "<< a.angle() << ".\n";
	cout << "Axis: " << a.axis().transpose() << "\n\n";

	cout << "Testing regular params to rotation matrix" << ".\n";

	ArrayXf regParams(5);
	regParams << 1, 0, 0, 0, M_PI;
	cout <<  reg_params_to_transformation_matrix(regParams) << "\n\n";

	cout << "Testing point cloud transforms" << ".\n";
	PointCloud testPointCloud(3,5);
	testPointCloud.row(0) << 1, 0,   0,  .5, -10;
	testPointCloud.row(1) << 0, 1,   0,   0,   0;
	testPointCloud.row(2) << 0, 0, 150, -.1,   0;

	cout << testPointCloud << "\n";
	cout << "   =====================" << "\n";
	cout << compute_transformed_points(testPointCloud,regParams) << "\n\n";
	/*
	high_resolution_clock::time_point t1 = high_resolution_clock::now();
    for(int idx = 0; idx<50000; idx++) compute_transformed_points(testPointCloud,regParams);
    high_resolution_clock::time_point t2 = high_resolution_clock::now();

    auto duration = duration_cast<microseconds>( t2 - t1 ).count();

    cout << duration << "\n";

    t1 = high_resolution_clock::now();
    for(int idx = 0; idx<50000; idx++) compute_transformed_points2(testPointCloud,regParams);
    t2 = high_resolution_clock::now();

    duration = duration_cast<microseconds>( t2 - t1 ).count();

    cout << duration << "\n";
	*/
    return 0;
}
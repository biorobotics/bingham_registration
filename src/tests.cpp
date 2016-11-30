#include <iostream>
#include <math.h>
#include <compute_transformed_points.h>
using namespace std;

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
	pointCloud testPointCloud(3,5);
	testPointCloud.row(0) << 1, 0,   0,  .5, -10;
	testPointCloud.row(1) << 0, 1,   0,   0,   0;
	testPointCloud.row(2) << 0, 0, 150, -.1,   0;

	cout << testPointCloud << "\n";
	cout << "   =====================" << "\n";
	cout << compute_transformed_points(testPointCloud,regParams) << "\n\n";
}
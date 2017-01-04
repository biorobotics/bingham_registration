#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <algorithm>
#include <iterator>

using namespace Eigen;
using namespace std;

int main()
{
	Matrix3d m;
	m << 1, 2, 3,
		 4, 5, 6,
		 7, 8, 9;

	EigenSolver<MatrixXd> es;
	es.compute(m, true);
	// This part might have problem
	VectorXd s = es.eigenvalues().real();  // A vector whose entries are eigen values of RTmp
	cout << "The eigen value vector is in the form of " << s << endl;
	cout << "s actually has the size of: " << s.size() << endl;
	MatrixXd U = es.eigenvectors().real();	// A matrix whose column vectors are eigen vectors of 
	cout << "The eigen vector is in the form of " << U << endl;
	MatrixXd RInvTmp = U * ((s.array().pow(-1)).matrix().asDiagonal()) * U.transpose();
	cout << "RInvTmp is in the form of " << RInvTmp << endl;
	// Test for converting VectorXd to std::vector
	vector<double> path(s.data(), s.data() + s.size());
	std::copy(path.begin(), path.end(), std::ostream_iterator<double>(std::cout, " "));
	//cout << "The eigen value vector after convertion is in the form of " << test << endl;
}
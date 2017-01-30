#include "pch.h"
#include <registration_tools.h>
/* call_error:
* 		Input: error message
*      Return: none. Prints out error message and exit the program
*/
void call_error(string msg) {
	cerr << "Error: " << msg << endl;
	exit(1);
}

/* quat2eul:
*		Input: quaternion
Output: euler angle in vector after conversion
*/
Vector3ld quat2eul(Quaternionld q) {
	// Normalize the quaternions
	Quaternionld temp = q.normalized();
	q = temp;

	long double qw = q.w();
	long double qx = q.x();
	long double qy = q.y();
	long double qz = q.z();

	Vector3ld eul(3);

	eul(0) = atan2(2 * (qx * qy + qw * qz), pow(qw, 2) + pow(qx, 2) - pow(qy, 2) - pow(qz, 2));
	eul(1) = asin(-2 * (qx * qz - qw * qy));
	eul(2) = atan2(2 * (qy * qz + qw * qx), pow(qw, 2) - pow(qx, 2) - pow(qy, 2) + pow(qz, 2));

	return eul;
}


/* sort_indexes:
*		Input: vector to be sorted, sorting order option (true for ascending, vice versa)
Return: the sorted results' index in the original vector

* Taken from http://stackoverflow.com/questions/1577475/c-sorting-and-keeping-track-of-indexes
*/
vector<unsigned int> sort_indexes(const vector<long double> &v, bool ascending) {
	// initialize original index locations
	vector<unsigned int> idx(v.size());
	for (size_t i = 0; i < idx.size(); i++) idx[i] = i;

	// sort indexes based on comparing values in v
	if (ascending)
		sort(idx.begin(), idx.end(), [&v](unsigned int i1, unsigned int i2) {return v[i1] < v[i2]; });
	else
		sort(idx.begin(), idx.end(), [&v](unsigned int i1, unsigned int i2) {return v[i1] > v[i2]; });

	return idx;
}
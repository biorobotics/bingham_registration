#ifndef REGISTRATION_TOOLS
#define REGISTRATION_TOOLS

#include <Eigen/Dense>
#include <type_defs.h>
#include <iostream>
#include <vector>
using namespace std;

/* sort_indexes:
*		Input: vector to be sorted, sorting order option (true for ascending, vice versa)
Return: the sorted results' index in the original vector

* Taken from http://stackoverflow.com/questions/1577475/c-sorting-and-keeping-track-of-indexes
*/
vector<unsigned int> sort_indexes(const vector<long double> &v, bool ascending);

/*  quat2eul:
*		Input: quaternion
Output: euler angle in vector after conversion
*/
Vector3ld quat2eul(Quaternionld q);

/* call_error:
* 		Input: error message
*      Return: none. Prints out error message and exit the program
*/
void call_error(string msg);

#endif

#ifndef REGISTRATION_TOOLS
#define REGISTRATION_TOOLS

#include "type_defs.h"

/* sort_indexes:
*		Input: vector to be sorted, sorting order option (true for ascending, vice versa)
Return: the sorted results' index in the original vector

* Taken from http://stackoverflow.com/questions/1577475/c-sorting-and-keeping-track-of-indexes
*/
Eigen::VectorXi sort_indexes(const VectorXld  &v, bool ascending);

#endif

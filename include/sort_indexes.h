#ifndef REGISTRATION_TOOLS
#define REGISTRATION_TOOLS

#include "type_defs.h"

/* sort_indexes:
*		Input: vector to be sorted, sorting order option (true for ascending, vice versa)
Return: the sorted results' index in the original vector

* Taken from http://stackoverflow.com/questions/1577475/c-sorting-and-keeping-track-of-indexes
*/
template <typename Type>
Eigen::VectorXi sort_indexes(const Type& v, bool ascending) {
	// initialize original index locations
	Eigen::VectorXi  idx(v.size());
	for (unsigned int i = 0; i < idx.size(); i++) idx[i] = i;

	// sort indexes based on comparing values in v
	if (ascending)
		std::sort(idx.data(), idx.data() + idx.size(), [&v](unsigned int i1, unsigned int i2) {return v[i1] < v[i2]; });
	else
		std::sort(idx.data(), idx.data() + idx.size(), [&v](unsigned int i1, unsigned int i2) {return v[i1] > v[i2]; });

	return idx;
}

#endif

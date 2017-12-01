#ifndef SEARCH_RESULT
#define SEARCH_RESULT

#include "type_defs.h"

struct SearchResult{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	PointCloud pc;
	PointCloud pr;
	long double res;
};

#endif
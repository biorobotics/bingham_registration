/*
 *	File Header:
 *  get_changes_in_transformation_estimate takes in the updated Xreg and previous
 *  Xreg, outputs dT and dR for checking if convergence is met.
 */
#ifndef GET_CHANGES_IN_TRANSFORMATION_ESTIMATE
#define GET_CHANGES_IN_TRANSFORMATION_ESTIMATE

#include "type_defs.h"

struct DeltaTransform{
	long double dR;
	long double dT;
};

/* get_changes_in_transformation_estimate:
 *		Input: pose from last iteration, a record of the poses from earlier iterations
 		Output: dR and dT to check wheter to stop the iteration
 */
extern "C" DeltaTransform get_changes_in_transformation_estimate(const VectorXld& Xreg, const VectorXld& Xregprev);

#endif
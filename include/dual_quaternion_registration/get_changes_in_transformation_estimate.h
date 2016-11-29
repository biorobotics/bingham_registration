/*
 *	File Header:
 *  get_changes_in_transformation_estimate takes in the updated Xreg and previous
 *  Xreg, outputs dT and dR for checking if convergence is met.
 */
#ifndef GET_CHANGES_IN_TRANSFORMATION_ESTIMATE
#define GET_CHANGES_IN_TRANSFORMATION_ESTIMATE

struct tuple1{
	double dR;
	double dT;
};

rowvec eul2quat(mat eul);

struct tuple1 get_changes_in_transformation_estimate(rowvec Xreg, rowvec Xregprev);

#endif
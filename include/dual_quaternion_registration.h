#ifndef DUAL_QUATERNION_REGISTRATION
	#define DUAL_QUATERNION_REGISTRATION
	#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
		#define EXPORT extern "C" __declspec(dllexport)
	#else
		#define EXPORT extern "C"
	#endif
	EXPORT long double* qf_register(char const * movingData, char const * fixedData,
									int inlierRatio, int maxIterations, int windowSize,
									double toleranceT, double toleranceR);

	int main(int argc, char *argv[]);
#endif
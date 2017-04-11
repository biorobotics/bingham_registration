#ifndef DUAL_QUATERNION_REGISTRATION
	#define DUAL_QUATERNION_REGISTRATION
	#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
		#define EXPORT extern "C" __declspec(dllexport)
	#else
		#define EXPORT extern "C"
	#endif
	EXPORT long double* combined_register(int registerOption, char const * movingData, char const * fixedData, char const * saveDest,
									double inlierRatio, int maxIterations, int windowSize,
									double toleranceT, double toleranceR, double uncertaintyR);

	int main(int argc, char *argv[]);
#endif
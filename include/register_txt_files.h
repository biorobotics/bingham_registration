#ifndef BINGHAM_REGISTRATION
	#define BINGHAM_REGISTRATION
	#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
		#define EXPORT extern "C" __declspec(dllexport)
	#else
		#define EXPORT extern "C"
	#endif
	EXPORT void free_result(long double *ptr);

	EXPORT long double* register_txt(char const * movingData, char const * fixedData,
									 double inlierRatio, int maxIterations, int windowSize,
									 double toleranceT, double toleranceR);
	
#endif
#include <registration_tools.h>
#include <registration_est_kf_rgbd.h>
#include <dual_quaternion_registration.h>

int main(int argc, char *argv[]) {

    string movingPointsString;
    string fixedPointsString;
    string movingNormalsString;
    string fixedNormalsString;
    int useNormal = 0;
    int normalFileProvided = 0;
    int pointFileProvided = 0;

    // Replace filenames by the arguments
    for (int i = 1; i < argc; ++i) {
        string arg = argv[i];

        // Parse ptcld_moving from .txt
        if (arg == "-pm") {
            if (i + 1 < argc) { // Make sure we aren't at the end of argv!
                i++; // Increment 'i' so we don't get the argument as the next argv[i].
                movingPointsString = argv[i];
                pointFileProvided++;
            } else { // If there was no argument to the destination option.
                std::cerr << "-pm option requires filepath for moving pointcloud." 
                << std::endl;
                return 1;
            }
        }
        // Parse ptcld_fixed from .txt
        else if (arg == "-pf") {
            if (i + 1 < argc) { // Make sure we aren't at the end of argv!
                i++; // Increment 'i' so we don't get the argument as the next argv[i].
                fixedPointsString = argv[i];
                pointFileProvided++;
            } else { // If there was no argument to the destination option.
                std::cerr << "-pf option requires filepath for fixed pointcloud." 
                << std::endl;
                return 1;
            }
        }
        // Parse normal moving from .txt
        else if (arg == "-nm") {
            if (i + 1 < argc) { // Make sure we aren't at the end of argv!
                i++; // Increment 'i' so we don't get the argument as the next argv[i].
                movingNormalsString = argv[i];
                useNormal = 1; 
                normalFileProvided++;
            } else { // If there was no argument to the destination option.
                std::cerr << "-nm option requires filepath for moving normals." 
                << std::endl;
                return 1;
            }
        }
        // Parse normal fixed from .txt
        else if (arg == "-nf") {
            if (i + 1 < argc) { // Make sure we aren't at the end of argv!
                i++; // Increment 'i' so we don't get the argument as the next argv[i].
                fixedNormalsString = argv[i];
                useNormal = 1;
                normalFileProvided++;
            } else { // If there was no argument to the destination option.
                std::cerr << "-nm option requires filepath for moving normals." 
                << std::endl;
                return 1;
            }
        }
        // If an invalid argument is provided
        else {
            std::cerr << "Invalid argument. Usage: -pm .. -pv .. (optional) (-nm .. -nv ..)" 
             << std::endl;
            return 1;
        }
    }

    // Check if enough arguments were entered
    if (pointFileProvided != 2 || (useNormal && normalFileProvided != 2)) {
        std::cerr << "Not enough argument. Usage: -pm .. -pv .. (optional) (-nm .. -nv ..)" 
        << std::endl;
        return 1;
    }
	int inlierRatio = 1; int maxIterations = 100; int windowSize = 20;
	double toleranceT = 0.001; double toleranceR = 0.009; double uncertainty = 300;
	long double *result = qf_register(movingPointsString.c_str(), fixedPointsString.c_str(), inlierRatio, maxIterations, windowSize, toleranceT, toleranceR, uncertainty);
    free(result);
    cout << "\nTEST\n\n";
}

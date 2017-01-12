# dual_quaternion_registration

Branched out of ros-service. Changes include:<br>
1. Converted everything to long double for better precision <br>
2. Cleared all the existing memory errors

01/09/2017 Updates by Olivia:

Changes:

/data:<br />
	ptcld_moving.txt and ptcld_fixed.txt are re-named to ptcld_moving_1.txt and ptcld_fixed_1.txt to keep consistency <br />
  ptcld_moving_5.txt and ptcld_fixed_5.txt are added into the folder
  
/src:<br />
  KDTree.cpp: Several functions are modified in terms of argument/return type to improve memory utilization.<br />
  Algorithm was untouched. A function was added for freeing the tree. 
  
  bingham_kf.cpp: additional steps were added to normalize the eigenvector/eigenvalue calculations.
  
  registration_est_bingham_kf_rgbd.cpp: variables related to kd_search function call were modified.
  
  registration_main.cpp： the registration function is now run 10 times to get the average registration time.

CMakeList.txt:<br />
  Compiled flags: -std=c+11 -03 -ffloat-store ${PCL_DEFINITIONS}
  
Initialization values in registration_est_bingham_kf_rgbd.cpp:<br />
  Inlier Ratio: 1<br />
  Max Iterations: 100<br />
  Min Iterations: 20<br />
  Window Size: 20<br />
  Tolerance: [.0001 .009]<br />
  Xk = [1 0 0 0]<br />
  Mk = 4x4 identity<br />
  Zk = diagonal [0 1 1 1] * -10^-100<br />
  dataset: ptcld_*_3.txt<br />
  
I also have the data (runtime, Xregsaves) of running different dataset, if you need those numbers let me know. (data collected by calling catkin_make to make files)
  

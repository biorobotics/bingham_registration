CC = g++
DEBUG = -g
CFLAGS = -I -Wall -c $(DEBUG)
LFLAGS = -Wall $(DEBUG)
OBJS = registration_est_kf_rgbd.o qr_kf.o KDTree.o get_changes_in_transformation_estimate.o compute_transformed_points.o

registration_make : $(OBJS)
	$(CC) $(LFLAGS) $(OBJS) -o registration_make -O2 -larmadillo

registration_est_kf_rgbd.o : registration_est_kf_rgbd.cpp KDTree.h get_changes_in_transformation_estimate.h qr_kf.h compute_transformed_points.h
	$(CC) $(CFLAGS) registration_est_kf_rgbd.cpp -O2 -larmadillo

qr_kf.o : qr_kf.h qr_kf.cpp KDTree.h
	$(CC) $(CFLAGS) qr_kf.cpp -O2 -larmadillo

KDTree.o : KDTree.h KDTree.cpp compute_transformed_points.h
	$(CC) $(CFLAGS) KDTree.cpp -O2 -larmadillo

get_changes_in_transformation_estimate.o : get_changes_in_transformation_estimate.h get_changes_in_transformation_estimate.cpp  
	$(CC) $(CFLAGS) get_changes_in_transformation_estimate.cpp -O2 -larmadillo

compute_transformed_points.o : compute_transformed_points.h compute_transformed_points.cpp
	$(CC) $(CFLAGS) compute_transformed_points.cpp -O2 -larmadillo

clean:
	\rm *.o *~ registration_make
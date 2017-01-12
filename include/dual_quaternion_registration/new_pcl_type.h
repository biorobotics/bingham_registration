#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

struct XregType
{
  double x; 
  double y;
  double z;
  double thetaX;
  double thetaY;
  double thetaZ;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (XregType,           // here we assume a XYZ + "test" (as fields)
                                   (double, x, x)
                                   (double, y, y)
                                   (double, z, z)
                                   (double, thetaX, thetaX)
                                   (double, thetaY, thetaY)
                                   (double, thetaZ, thetaZ)
)
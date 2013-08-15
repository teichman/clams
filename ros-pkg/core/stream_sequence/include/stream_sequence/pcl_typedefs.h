#ifndef PCL_TYPEDEFS_H
#define PCL_TYPEDEFS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace clams
{
  typedef pcl::PointXYZRGB Point;
  typedef pcl::PointCloud<Point> Cloud;
}

#endif // PCL_TYPEDEFS_H

#ifndef TYPEDEFS_H
#define TYPEDEFS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <eigen_extensions/eigen_extensions.h>

namespace clams
{
  typedef pcl::PointXYZRGB Point;
  typedef pcl::PointCloud<Point> Cloud;
  // PCL and OpenCV are row-major, but lots of work is required to make this change.
  // eigen_extensions serialization and re-collection of data, probably.
  //typedef Eigen::Matrix<unsigned short, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> DepthMat;
  typedef Eigen::Matrix<unsigned short, Eigen::Dynamic, Eigen::Dynamic> DepthMat;  
  typedef boost::shared_ptr<DepthMat> DepthMatPtr;
  typedef boost::shared_ptr<const DepthMat> DepthMatConstPtr;
}

#endif // TYPEDEFS_H

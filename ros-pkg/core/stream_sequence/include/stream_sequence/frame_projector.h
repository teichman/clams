#ifndef FRAME_PROJECTOR_H
#define FRAME_PROJECTOR_H

#include <ros/assert.h>
#include <ros/console.h>
#include <pcl/common/transforms.h>
#include <pcl/common/distances.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <serializable/serializable.h>
#include <eigen_extensions/eigen_extensions.h>
#include <stream_sequence/pcl_typedefs.h>
#include <stream_sequence/typedefs.h>

namespace clams
{

  //! "Projective" point comes from the OpenNI terminology, and refers to (u, v, z), i.e.
  //! pixel id and depth value.  Here I've added color, too, so that this represents everything
  //! that is known about a pixel in an RBGD camera.
  class ProjectivePoint : public Serializable
  {
  public:
    int u_;
    int v_;
    //! In millimeters, same as the raw depth image from the primesense device.
    unsigned short z_;
    unsigned char r_;
    unsigned char g_;
    unsigned char b_;

    void serialize(std::ostream& out) const;
    void deserialize(std::istream& in);
  };

  class Frame : public Serializable
  {
  public:
    DepthMatPtr depth_;
    cv::Mat3b img_;
    double timestamp_;

    cv::Mat3b depthImage() const;
    void serialize(std::ostream& out) const;
    void deserialize(std::istream& in);
    
  protected:
    cv::Vec3b colorize(double depth, double min_range, double max_range) const;
  };

  //! This is essentially a pinhole camera model for an RGBD sensor, with
  //! some extra functions added on for use during calibration.
  class FrameProjector : public Serializable
  {
  public:
    // For storing z values in meters.  This is not Euclidean distance.
    typedef std::vector< std::vector< std::vector<double> > > RangeIndex;
    typedef Eigen::Matrix<size_t, Eigen::Dynamic, Eigen::Dynamic> IndexMap;
    
    int width_;
    int height_;
    double cx_;
    double cy_;
    double fx_;
    double fy_;

    FrameProjector();

    //! max_range in meters
    void frameToCloud(const Frame& frame, Cloud* pcd,
                      double max_range = std::numeric_limits<double>::max()) const;
    void cloudToFrame(const Cloud& pcd, Frame* frame, IndexMap* indexmap = NULL) const;
    void cloudToRangeIndex(const Cloud& pcd, RangeIndex* rindex) const;
    //! transform is applied to the map, then projected into a depth index.
    //! The best depth estimate from the map corresponding to the measurement depth frame
    //! will be returned.
    void estimateMapDepth(const Cloud& map, const Eigen::Affine3f& transform,
                          const Frame& measurement,
                          DepthMat* estimate) const;
                          
    void project(const ProjectivePoint& ppt, Point* pt) const;
    void project(const Point& pt, ProjectivePoint* ppt) const;

    bool initialized() const;
    std::string status(const std::string& prefix = "") const;
    void serialize(std::ostream& out) const;
    void deserialize(std::istream& in);

  protected:
    bool coneFit(const DepthMat& naive_mapdepth, const RangeIndex& rindex,
                 int uc, int vc, double radius, double measurement_depth,
                 double* mean, double* stdev) const;
  };

}  // namespace clams

#endif // FRAME_PROJECTOR_H

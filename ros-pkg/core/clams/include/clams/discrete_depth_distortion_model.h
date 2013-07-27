#ifndef DISCRETE_DEPTH_DISTORTION_MODEL_H
#define DISCRETE_DEPTH_DISTORTION_MODEL_H

#include <ros/assert.h>
#include <ros/console.h>
#include <boost/shared_ptr.hpp>
#include <bag_of_tricks/lockable.h>
#include <stream_sequence/frame_projector.h>

namespace clams
{

  class DiscreteFrustum : public Serializable, public SharedLockable
  {
  public:
    DiscreteFrustum(int smoothing = 1, double bin_depth = 1.0);
    //! z value, not distance to origin.
    void addExample(double ground_truth, double measurement);
    int index(double z) const;
    void undistort(double* z) const;
    void interpolatedUndistort(double* z) const;
    void serialize(std::ostream& out) const;
    void deserialize(std::istream& in);
  
  protected:
    double max_dist_;
    int num_bins_;
    double bin_depth_;
    Eigen::VectorXf counts_;
    Eigen::VectorXf total_numerators_;
    Eigen::VectorXf total_denominators_;
    Eigen::VectorXf multipliers_;

    friend class DiscreteDepthDistortionModel;
  };

  class DiscreteDepthDistortionModel : public Serializable
  {
  public:
    typedef boost::shared_ptr<DiscreteDepthDistortionModel> Ptr;
    typedef boost::shared_ptr<const DiscreteDepthDistortionModel> ConstPtr;
    DiscreteDepthDistortionModel() {}
    ~DiscreteDepthDistortionModel();
    DiscreteDepthDistortionModel(const FrameProjector& proj, int bin_width = 8, int bin_height = 6, double bin_depth = 2.0, int smoothing = 1);
    DiscreteDepthDistortionModel(const DiscreteDepthDistortionModel& other);
    DiscreteDepthDistortionModel& operator=(const DiscreteDepthDistortionModel& other);
    void undistort(Frame* frame) const;
    //! Returns the number of training examples it used from this pair.
    //! Thread-safe.
    size_t accumulate(const Frame& ground_truth, const Frame& measurement);
    void addExample(const ProjectivePoint& ppt, double ground_truth, double measurement);
    void serialize(std::ostream& out) const;
    void deserialize(std::istream& in);
    //! Saves images to the directory found at path.
    //! If path doesn't exist, it will be created.
    void visualize(const std::string& path) const;
  
  protected:
    FrameProjector proj_;
    int width_;
    int height_;
    int bin_width_;
    int bin_height_;
    double bin_depth_;
    int num_bins_x_;
    int num_bins_y_;
    //! frustums_[y][x]
    std::vector< std::vector<DiscreteFrustum*> > frustums_;

    void deleteFrustums();
    DiscreteFrustum& frustum(int y, int x);
    const DiscreteFrustum& frustum(int y, int x) const;
  };

}  // namespace clams

#endif // DISCRETE_DEPTH_DISTORTION_MODEL_H

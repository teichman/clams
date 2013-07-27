#ifndef CLAMS_STANDALONE_H
#define CLAMS_STANDALONE_H

/************************************************************
 * This is a giant header file which contains everything
 * you need to read a DiscreteDepthDistortionModel from disk
 * and apply it to new data.
 *
 * This is definitely not an example of good programming
 * practice... but you might find it useful in applying a
 * learned model in your own projects rather than linking
 * in our code.
 ************************************************************/

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


namespace clams
{

#define scopeLockRead boost::shared_lock<boost::shared_mutex> lockable_shared_lock(shared_mutex_)
#define scopeLockWrite boost::unique_lock<boost::shared_mutex> lockable_unique_lock(shared_mutex_)

#define MAX_MULT 1.2
#define MIN_MULT 0.8

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

  //! This is essentially a pinhole camera model for an RGBD sensor.
  class FrameProjector : public Serializable
  {
  public:
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
    void project(const ProjectivePoint& ppt, Point* pt) const;
    void project(const Point& pt, ProjectivePoint* ppt) const;
    bool initialized() const;
    std::string status(const std::string& prefix = "") const;
  };

  
  /* SharedLockable is based on the uncopyable boost::shared_mutex.
     This presents a dilemma when assigning or copy constructing.
     Right now, the state of the mutex in the other SharedLockable
     does not get copied to the target SharedLockable.
     I'm not sure yet if this is the desired behavior.
  */
  class SharedLockable
  {
  public:
    SharedLockable();
    //! Copy constructor will make a new shared_mutex that is unlocked.
    SharedLockable(const SharedLockable& other) {}
    //! Assignment operator will *not* copy the shared_mutex_ or the state of shared_mutex_ from other.
    SharedLockable& operator=(const SharedLockable& other) { return *this; }

    void lockWrite();
    void unlockWrite();
    bool trylockWrite();
  
    void lockRead();
    void unlockRead();
    bool trylockRead();

  protected:
    //! For the first time ever, I'm tempted to make this mutable.
    //! It'd make user methods still be able to be const even if they are locking.
    boost::shared_mutex shared_mutex_;
  };
  
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

SharedLockable::SharedLockable()
{
}

void SharedLockable::lockWrite()
{
  shared_mutex_.lock();
}

void SharedLockable::unlockWrite()
{
  shared_mutex_.unlock();
}

bool SharedLockable::trylockWrite()
{
  return shared_mutex_.try_lock();
}

void SharedLockable::lockRead()
{
  shared_mutex_.lock_shared();
}

void SharedLockable::unlockRead()
{
  shared_mutex_.unlock_shared();
}

bool SharedLockable::trylockRead()
{
  return shared_mutex_.try_lock_shared();
}


  
}
#endif // CLAMS_STANDALONE_H

#include <stream_sequence/stream_sequence_pcl_wrapper.h>

using namespace std;
namespace bfs = boost::filesystem;

namespace clams
{

  StreamSequencePCLWrapper::StreamSequencePCLWrapper():
    max_depth_(numeric_limits<double>::max())
  {
  }
  
  void StreamSequencePCLWrapper::loadImpl(const std::string& dir)
  {
    grabber_.reset (new pcl::ImageGrabber<pcl::PointXYZRGB> (dir, 0, false, true) ); // PCLZF mode
    grabber_->start ();
    proj_.width_ = 640; // TODO ?
    proj_.height_ = 480; // TODO ?
    grabber_->getCameraIntrinsics (proj_.fx_, proj_.fy_, proj_.cx_, proj_.cy_);
    // Update timestamps
    timestamps_.resize (grabber_->size ());
    for (size_t i = 0; i < grabber_->size (); i++)
    {
      uint64_t timestamp_microsec;
      grabber_->getTimestampAtIndex (i, timestamp_microsec);
      timestamps_[i] = static_cast<double> (timestamp_microsec) * 1E-6;
    }
  }
  

  void StreamSequencePCLWrapper::readFrameImpl(size_t idx, Frame* frame) const
  {
    ROS_ASSERT (idx < grabber_->size ());
    clams::Cloud::ConstPtr cloud = grabber_->at (idx);
    // Make img_ and depth_;
    frame->timestamp_ = timestamps_[idx];
    frame->depth_ = DepthMatPtr (new DepthMat (cloud->height, cloud->width));
    frame->depth_->setZero ();
    frame->img_ = cv::Mat3b (cloud->height, cloud->width);
    for (size_t u = 0; u < cloud->width; u++)
    {
      for (size_t v = 0; v < cloud->height; v++)
      {
        const pcl::PointXYZRGB &pt = cloud->at (u, v);
        frame->img_(v, u)[0] = pt.b;
        frame->img_(v, u)[1] = pt.g;
        frame->img_(v, u)[2] = pt.r;
        if (!pcl_isnan (pt.z))
          frame->depth_->coeffRef (v, u) = pt.z*1000;

      }
    }
  }


  size_t StreamSequencePCLWrapper::size() const
  {
    return grabber_->size ();
  }
  
}  // namespace clams




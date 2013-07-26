#include <rgbd_sequence/stream_sequence_base.h>
#include <rgbd_sequence/stream_sequence.h>
#include <rgbd_sequence/stream_sequence_pcl_wrapper.h>
#include <limits>

rgbd::StreamSequenceBase::Ptr rgbd::StreamSequenceBase::initializeFromDirectory (const std::string &dir)
{
  // Check if it's the old or new format
  StreamSequenceBase::Ptr out;
  if (boost::filesystem::exists (dir + "/primesense_model"))
    out.reset (new StreamSequence);
  else
    out.reset (new StreamSequencePCLWrapper);
  out->load (dir);
  return out;
}

void rgbd::StreamSequenceBase::load (const std::string &root_path)
{
  
   loadImpl (root_path);
   // Populate distortion model
   if (boost::filesystem::exists (root_path + "/distortion_model"))
   {
     dddm_.reset (new DiscreteDepthDistortionModel);
     dddm_->load (root_path + "/distortion_model");
   }
   root_path_ = root_path;
}

size_t rgbd::StreamSequenceBase::seek(double timestamp, double* dt) const
{
  ROS_ASSERT(!timestamps_.empty());
  
  // TODO: This could be much faster than linear search.
  size_t nearest = 0;
  *dt = std::numeric_limits<double>::max();
  for(size_t i = 0; i < timestamps_.size(); ++i) {
    double d = timestamp - timestamps_[i];
    if(fabs(d) < *dt) {
      *dt = d;
      nearest = i;
    }
  }

  return nearest;
}
  
void rgbd::StreamSequenceBase::applyTimeOffset(double dt)
{
  for(size_t i = 0; i < timestamps_.size(); ++i)
    timestamps_[i] += dt;
}
  
rgbd::Cloud::Ptr rgbd::StreamSequenceBase::getCloud(size_t idx) const
{
  Cloud::Ptr pcd(new Cloud);
  Frame frame;
  readFrame(idx, &frame);
  model_.frameToCloud(frame, pcd.get());
  return pcd;
}

rgbd::Cloud::Ptr rgbd::StreamSequenceBase::getCloud(double timestamp, double* dt) const
{
  Cloud::Ptr pcd(new Cloud);
  Frame frame;
  readFrame(timestamp, dt, &frame);
  model_.frameToCloud(frame, pcd.get());
  return pcd;
}

cv::Mat3b rgbd::StreamSequenceBase::getImage(size_t idx) const
{
  Frame frame;
  readFrame(idx, &frame);
  return frame.img_;
}

cv::Mat3b rgbd::StreamSequenceBase::getImage(double timestamp, double* dt) const
{
  Frame frame;
  readFrame(timestamp, dt, &frame);
  return frame.img_;
}


rgbd::Cloud::ConstPtr rgbd::StreamSequenceBase::operator[] (size_t idx) const
{
  // With no cache, we just return the cloud
  if (cache_size_ == 0)
    return getCloud (idx);
  // Otherwise we update cache -- not threadsafe
  if (!pcds_cache_[idx])
  {
#pragma omp critical
    {
    if (!pcds_cache_[idx])
    {
      rgbd::Cloud::ConstPtr cloud = getCloud (idx);
      addCloudToCache (idx, cloud);
    }
    }
  }
  return pcds_cache_[idx];
}

rgbd::Cloud::ConstPtr rgbd::StreamSequenceBase::at (size_t idx) const
{
  ROS_ASSERT (idx < size ());
  return operator[] (idx);
}

void rgbd::StreamSequenceBase::addCloudToCache (
    size_t idx, 
    rgbd::Cloud::ConstPtr cloud) const
{
  if (frames_cache_.size () >= cache_size_)
  {
    size_t frame_to_remove = frames_cache_.back ();
    rgbd::Cloud::ConstPtr &cloud_to_remove = pcds_cache_[frame_to_remove];
    cloud_to_remove.reset ();
    frames_cache_.pop_back ();
  }
  // Add to the cache
  frames_cache_.push_front (idx);
  pcds_cache_[idx] = cloud;
}
  
size_t rgbd::StreamSequenceBase::readFrame(double timestamp, double* dt, Frame* frame) const
{
  size_t idx = seek(timestamp, dt);
  readFrame(idx, frame);    
  return idx;
}

void rgbd::StreamSequenceBase::readFrame(size_t idx, Frame* frame) const
{
  readFrameImpl (idx, frame);
  if (undistort_)  
  {
    if (!dddm_)
      PCL_WARN ("Attempted to undistort a frame, but no distortion model was found!\n");
    else
    {
      dddm_->undistort (frame); 
    }
  }
}

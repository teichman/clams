#ifndef SLAM_CALIBRATION_VISUALIZER_H
#define SLAM_CALIBRATION_VISUALIZER_H

#include <boost/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <bag_of_tricks/lockable.h>
#include <stream_sequence/stream_sequence_base.h>
#include <clams/slam_calibrator.h>

namespace clams
{

  class SlamCalibrationVisualizer : public SharedLockable
  {
  public:
    DiscreteDepthDistortionModel* dddm_;
  
    SlamCalibrationVisualizer(SlamCalibrator::Ptr calibrator);
    ~SlamCalibrationVisualizer() { if(dddm_) delete dddm_; }
      
    void run();
    void setCamera(const std::string& camera_path);
  
  protected:
    pcl::visualization::PCLVisualizer vis_;
    SlamCalibrator::Ptr calibrator_;
    Cloud::Ptr map_;
    bool quitting_;
    bool needs_update_;
    size_t seq_idx_;
    size_t frame_idx_;
    bool show_frame_;
    bool use_distortion_model_;
    bool color_frame_;

    void visualizationThreadFunction();
    void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie);
    void pointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* cookie);
    void setSequenceIdx(size_t idx);
    void incrementSequenceIdx(int num);
    void incrementFrameIdx(int num);
  };

}

#endif // SLAM_CALIBRATION_VISUALIZER_H

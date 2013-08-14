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

  class TrajectoryVisualizer : public SharedLockable
  {
  public:
    DiscreteDepthDistortionModel* dddm_;
    double max_range_;
    double vgsize_;
  
    TrajectoryVisualizer(StreamSequenceBase::ConstPtr sseq,
                         Trajectory traj, std::string title = "");
    ~TrajectoryVisualizer() { if(dddm_) delete dddm_; }
    void run();
  
  protected:
    StreamSequenceBase::ConstPtr sseq_;
    Trajectory traj_;
    pcl::visualization::PCLVisualizer vis_;
    Cloud::Ptr map_;
    bool quitting_;
    bool needs_update_;
    size_t frame_idx_;
    bool show_frame_;
    bool use_distortion_model_;
    bool color_frame_;
    std::string title_;

    void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie);
    void pointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* cookie);
    void incrementFrameIdx(int num);
  };

}

#endif // SLAM_CALIBRATION_VISUALIZER_H

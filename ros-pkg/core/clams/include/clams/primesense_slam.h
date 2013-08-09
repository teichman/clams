#ifndef PRIMESENSE_SLAM_H
#define PRIMESENSE_SLAM_H

#include <pcl/filters/voxel_grid.h>
#include <bag_of_tricks/agent.h>
#include <pose_graph_slam/pose_graph_slam.h>
#include <rgbd_sequence/stream_sequence_base.h>
#include <clams/trajectory.h>
#include <clams/frame_aligner.h>
#include <clams/discrete_depth_distortion_model.h>
#include <vector>

#define MAX_RANGE_MAP 2.0

namespace clams
{

  class PrimeSenseSlam : public Agent
  {
  public:
    typedef boost::shared_ptr<cv::Mat1f> FeaturesPtr;
    typedef boost::shared_ptr<const cv::Mat1f> FeaturesConstPtr;

    // -- Inputs
    StreamSequenceBase::ConstPtr sseq_;
    //! If not NULL, it will use these intrinsics when computing frame alignments.
    //! PrimeSenseSlam will not delete this.
    const DiscreteDepthDistortionModel* intrinsics_;
  
    // -- Params
    //! When choosing the next frame, advance by at least this much.
    double min_dt_;
    //! Max number per frame to use.  Selected in random order.
    size_t max_loopclosures_;
    //! per frame
    size_t max_loopclosure_tests_;
    int keypoints_per_frame_;
  
    // -- Outputs
    std::vector<Trajectory> trajs_;
    //! Just for visualization.
    std::vector<Cloud::Ptr> maps_;
    //! For saving the graph.
    PoseGraphSlam::Ptr pgs_;
  
    // -- Methods
    PrimeSenseSlam();
    void _run();
    //! Called at the end of _run(), public so we can call it externally w/o running everything
    void populateTrajAndMaps();
    FeaturesPtr getFeatures(const Frame &frame, std::vector<cv::KeyPoint> &keypoints,
                            Cloud::ConstPtr &keycloud) const;

    pl::Params params_;
    static inline pl::Params defaultParams()
    {
      pl::Params params;
      params.load(ros::package::getPath("xpl_calibration") + "/data/default_slam_params.txt");
      return params;
    }

  protected:
    std::map< size_t, std::vector<cv::KeyPoint> > keypoint_cache_;
    std::map< size_t, Cloud::ConstPtr > keycloud_cache_;
    std::map< size_t, FeaturesPtr > feature_cache_;
    std::vector<size_t> cached_frames_;

    Cloud::Ptr buildMap(const Trajectory& traj) const;
    FeaturesPtr cacheFeatures(const Frame &frame, size_t t, 
                              std::vector<cv::KeyPoint> &keypoints, Cloud::ConstPtr &keycloud);
  };

}

#endif // PRIMESENSE_SLAM_H

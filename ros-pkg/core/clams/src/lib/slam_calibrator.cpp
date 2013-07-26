#include <clams/slam_calibrator.h>

using namespace std;
using namespace Eigen;

namespace clams
{

  SlamCalibrator::SlamCalibrator(const FrameProjector& proj, double max_range, double vgsize) :
    proj_(proj),
    max_range_(max_range),
    vgsize_(vgsize),
    increment_(1)
  {
  }

  void filterFringe(Frame* frame)
  {
    DepthMat& depth = *frame->depth_;

    cv::Mat1b mask(depth.rows(), depth.cols());  // points to be removed.
    mask = 0;
    ushort threshold = 5000;  // millimeters
    for(int y = 1; y < depth.rows(); ++y) {
      for(int x = 1; x < depth.cols(); ++x) {
        if(depth(y, x) == 0 || depth(y-1, x) == 0 || depth(y, x-1) == 0 ||
           fabs(depth(y, x) - depth(y-1, x)) > threshold ||
           fabs(depth(y, x) - depth(y, x-1)) > threshold)
        {
          mask(y, x) = 255;
        }
      }
    }

    // cv::imshow("mask", mask);
    // cv::imshow("depth", frame->depthImage());
    // cv::waitKey();
  
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 8);
    // cv::imshow("mask", mask);
    // cv::waitKey();
    
    for(int y = 1; y < depth.rows(); ++y)
      for(int x = 1; x < depth.cols(); ++x)
        if(mask(y, x))
          depth(y, x) = 0;  

    // cv::imshow("depth", frame->depthImage());
    // cv::waitKey();
  }

  Cloud::Ptr SlamCalibrator::buildMap(StreamSequenceBase::ConstPtr sseq, const Trajectory& traj, double max_range, double vgsize)
  {
    ROS_DEBUG_STREAM("Building slam calibration map using max range of " << max_range);
  
    Cloud::Ptr map(new Cloud);
    int num_used_frames = 0;
    for(size_t i = 0; i < traj.size(); ++i) {
      if(!traj.exists(i))
        continue;

      cout << "Using frame " << i << " / " << traj.size() << endl;
      Frame frame;

      sseq->readFrame(i, &frame);
      filterFringe(&frame);
    
      Cloud::Ptr tmp(new Cloud);
      sseq->proj_.frameToCloud(frame, tmp.get(), max_range);
      Cloud::Ptr nonans(new Cloud);
      nonans->reserve(tmp->size());
      for(size_t j = 0; j < tmp->size(); ++j)
        if(isFinite(tmp->at(j)))
          nonans->push_back(tmp->at(j));
        
      pcl::transformPointCloud(*nonans, *nonans, traj.get(i).cast<float>());

      *map += *nonans;
      ++num_used_frames;
      // Added intermediate filtering to handle memory overload on huge maps
      if(num_used_frames % 50 == 0)
      {
        cout << "Filtering..." << endl;
        HighResTimer hrt("filtering");
        hrt.start();
        pcl::VoxelGrid<Point> vg;
        vg.setLeafSize(vgsize, vgsize, vgsize);
        Cloud::Ptr tmp(new Cloud);
        vg.setInputCloud(map);
        vg.filter(*tmp);
        *map = *tmp;
        hrt.stop();
      }
    }

    cout << "Filtering..." << endl;
    HighResTimer hrt("filtering");
    hrt.start();
    pcl::VoxelGrid<Point> vg;
    vg.setLeafSize(vgsize, vgsize, vgsize);
    Cloud::Ptr tmp(new Cloud);
    vg.setInputCloud(map);
    vg.filter(*tmp);
    *map = *tmp;
    hrt.stop();
    cout << hrt.reportMilliseconds() << endl;
    cout << "Filtered map has " << map->size() << " points." << endl;

    return map;
  }

  Cloud::Ptr SlamCalibrator::buildMap(size_t idx) const
  {
    ROS_ASSERT(idx < trajectories_.size());
    ROS_ASSERT(trajectories_.size() == sseqs_.size());
    return buildMap(sseqs_[idx], trajectories_[idx], max_range_, vgsize_);
  }

  size_t SlamCalibrator::size() const
  {
    ROS_ASSERT(trajectories_.size() == sseqs_.size());
    return trajectories_.size();
  }

  DiscreteDepthDistortionModel SlamCalibrator::calibrate() const
  {
    ROS_ASSERT(!sseqs_.empty());
    DiscreteDepthDistortionModel model(sseqs_[0]->proj_);

    size_t total_num_training = 0;
    for(size_t i = 0; i < size(); ++i) {
      // -- Construct the map from the data and the trajectory.
      StreamSequenceBase::ConstPtr sseq = sseqs_[i];
      const Trajectory& traj = trajectories_[i];
      Cloud::Ptr map = buildMap(i);
      total_num_training += processMap(*sseq, traj, *map, &model);
    }

    cout << "Training new DiscreteDepthDistortionModel using "
         << total_num_training << " training examples." << endl;
  
    return model;
  }

  size_t SlamCalibrator::processMap(const StreamSequenceBase& sseq,
                                    const Trajectory& traj, const Cloud& map,
                                    DiscreteDepthDistortionModel* model) const
  {
    // -- Select which frame indices from the sequence to use.
    //    Consider only those with a pose in the Trajectory,
    //    and apply downsampling based on increment_.
    vector<size_t> indices;
    indices.reserve(traj.numValid());
    int num = 0;
    for(size_t i = 0; i < traj.size(); ++i) {
      if(traj.exists(i) && num % increment_ == 0) {
        indices.push_back(i);
        ++num;
      }
    }

    // -- For all selected frames, accumulate training examples
    //    in the distortion model.
    VectorXi counts = VectorXi::Zero(indices.size());
#pragma omp parallel for
    for(size_t i = 0; i < indices.size(); ++i) {
      size_t idx = indices[i];
      ROS_ASSERT(traj.exists(idx));
      cout << "." << flush;

      Frame measurement;
      sseq.readFrame(idx, &measurement);
    
      Frame mapframe;
      mapframe.depth_ = DepthMatPtr(new DepthMat);
      sseq.proj_.estimateMapDepth(map, traj.get(idx).inverse().cast<float>(),
                                  measurement, mapframe.depth_.get());
      counts[i] = model->accumulate(mapframe, measurement);
    }

    return counts.sum();
  }

}  // namespace clams

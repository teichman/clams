#include <clams/slam_calibrator.h>

using namespace std;
using namespace Eigen;

namespace clams
{

  SlamCalibrator::SlamCalibrator(const FrameProjector& proj) :
    proj_(proj),
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

      // if(i % traj.size() / 10 == 0)
      //   cout << "." << flush;
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
        //cout << "Filtering..." << endl;
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
    //cout << endl;
    
    //cout << "Filtering..." << endl;
    HighResTimer hrt("filtering");
    hrt.start();
    pcl::VoxelGrid<Point> vg;
    vg.setLeafSize(vgsize, vgsize, vgsize);
    Cloud::Ptr tmp(new Cloud);
    vg.setInputCloud(map);
    vg.filter(*tmp);
    *map = *tmp;
    hrt.stop();
    //cout << hrt.reportMilliseconds() << endl;
    cout << "Filtered map has " << map->size() << " points." << endl;

    return map;
  }

  size_t SlamCalibrator::size() const
  {
    ROS_ASSERT(trajectories_.size() == sseqs_.size());
    return trajectories_.size();
  }

  DiscreteDepthDistortionModel SlamCalibrator::calibrate() const
  {
    ROS_ASSERT(!sseqs_.empty());
    DiscreteDepthDistortionModel model(sseqs_[0]->proj_.width_, sseqs_[0]->proj_.height_);

    size_t total_num_training = 0;
    for(size_t i = 0; i < size(); ++i) {
      cout << "Accumulating training data for sequence " << i << flush;
      total_num_training += processMap(*sseqs_[i], trajectories_[i], *maps_[i], &model);
    }

    cout << "Trained new DiscreteDepthDistortionModel using "
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
      if(traj.exists(i)) { 
        ++num;
        if(num % increment_ == 0)
          indices.push_back(i);
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
      counts[i] = model->accumulate(*mapframe.depth_, *measurement.depth_);

      // -- Quick and dirty option for data inspection.
      if(getenv("U") && getenv("V")) {
        int u_center = atoi(getenv("U"));
        int v_center = atoi(getenv("V"));
        int radius = 1;
        for(int u = max(0, u_center - radius); u < min(640, u_center + radius + 1); ++u) {
          for(int v = max(0, v_center - radius); v < min(480, v_center + radius + 1); ++v) {
            if(mapframe.depth_->coeffRef(v, u) == 0)
              continue;
            if(measurement.depth_->coeffRef(v, u) == 0)
              continue;
            cerr << mapframe.depth_->coeffRef(v, u) * 0.001
                 << " "
                 << measurement.depth_->coeffRef(v, u) * 0.001
                 << endl;
          }
        }
      }
    }
    cout << endl;
    

    return counts.sum();
  }

}  // namespace clams

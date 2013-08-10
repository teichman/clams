#include <clams/slam_calibration_visualizer.h>

using namespace std;
using namespace Eigen;

namespace clams
{

  SlamCalibrationVisualizer::SlamCalibrationVisualizer(SlamCalibrator::Ptr calibrator) :
    dddm_(NULL),
    calibrator_(calibrator),
    map_(new Cloud),
    quitting_(false),
    needs_update_(false),
    seq_idx_(0),
    frame_idx_(0),
    show_frame_(false),
    use_distortion_model_(false),
    color_frame_(false)
  {
    vis_.registerKeyboardCallback(&SlamCalibrationVisualizer::keyboardCallback, *this);
    vis_.registerPointPickingCallback(&SlamCalibrationVisualizer::pointPickingCallback, *this);
    vis_.setBackgroundColor(1, 1, 1);
  
    // -- Set the viewpoint to be sensible for PrimeSense devices.
    // SDM TODO vis_.camera_.clip[0] = 0.00387244;
    // SDM TODO vis_.camera_.clip[1] = 3.87244;
    // SDM TODO vis_.camera_.focal[0] = -0.160878;
    // SDM TODO vis_.camera_.focal[1] = -0.0444743;
    // SDM TODO vis_.camera_.focal[2] = 1.281;
    // SDM TODO vis_.camera_.pos[0] = 0.0402195;
    // SDM TODO vis_.camera_.pos[1] = 0.0111186;
    // SDM TODO vis_.camera_.pos[2] = -1.7;
    // SDM TODO vis_.camera_.view[0] = 0;
    // SDM TODO vis_.camera_.view[1] = -1;
    // SDM TODO vis_.camera_.view[2] = 0;
    // SDM TODO vis_.camera_.window_size[0] = 1678;
    // SDM TODO vis_.camera_.window_size[1] = 525;
    // SDM TODO vis_.camera_.window_pos[0] = 2;
    // SDM TODO vis_.camera_.window_pos[1] = 82;
    // SDM TODO vis_.updateCamera();    
  }

  void SlamCalibrationVisualizer::run()
  {
    //boost::thread thread_slam(boost::bind(&SlamVisualizer::slamThreadFunction, this));
    // Apparently PCLVisualizer needs to run in the main thread.

    setSequenceIdx(0);
    incrementFrameIdx(1);
    visualizationThreadFunction();

    //thread_slam.join();
  }

  void SlamCalibrationVisualizer::setCamera(const std::string& camera_path)
  {
    int argc = 3;
    char* argv[argc];
    argv[0] = (char*)string("aoeu").c_str();
    argv[1] = (char*)string("-cam").c_str();
    argv[2] = (char*)camera_path.c_str();
    bool success = vis_.getCameraParameters(argc, argv);
    ROS_ASSERT(success);
    vis_.updateCamera();    
  }

  void SlamCalibrationVisualizer::visualizationThreadFunction()
  {
    while(true) {
      { scopeLockRead; if(quitting_) break; }
      usleep(5e3);
    
      lockWrite();
      if(needs_update_) {
        Cloud::Ptr pcd(new Cloud);
        *pcd = *map_;

        // -- Add the raw sensor data from the current frame.
        if(show_frame_) { 
          const Trajectory& traj = calibrator_->trajectories_[seq_idx_];
          StreamSequenceBase::ConstPtr sseq = calibrator_->sseqs_[seq_idx_];
          if(traj.exists(frame_idx_)) {
            Frame pose_frame;
            sseq->readFrame(frame_idx_, &pose_frame);
            if(dddm_ && use_distortion_model_) {
              ScopedTimer st("Undistorting");
              dddm_->undistort(pose_frame.depth_.get());
            }
            Cloud pose_pcd;
            sseq->proj_.frameToCloud(pose_frame, &pose_pcd);
            if(!color_frame_) {
              for(size_t i = 0; i < pose_pcd.size(); ++i) {
                pose_pcd[i].r = 255;
                pose_pcd[i].g = 0;
                pose_pcd[i].b = 0;
              }
            }
          
            Affine3f transform = traj.get(frame_idx_).cast<float>();
            pcl::transformPointCloud(pose_pcd, pose_pcd, transform);
            *pcd += pose_pcd;
          }
        }
      
        if(!vis_.updatePointCloud(pcd, "default"))
          vis_.addPointCloud(pcd, "default");
        needs_update_ = false;
      }

      vis_.removeCoordinateSystem();
      vis_.addCoordinateSystem(0.3, calibrator_->trajectories_[seq_idx_].get(frame_idx_).cast<float>());
    
      unlockWrite();
      vis_.spinOnce(3);
    }
  }

  void SlamCalibrationVisualizer::pointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* cookie)
  {
    scopeLockRead;

    if(event.getPointIndex() == -1)
      return;
    const Trajectory& traj = calibrator_->trajectories_[seq_idx_];
    if(!traj.exists(frame_idx_))
      return;
    
    Point pt;
    event.getPoint(pt.x, pt.y, pt.z);
    cout << "Selected point: " << pt.x << ", " << pt.y << ", " << pt.z << endl;
    vis_.removeAllShapes();
  
    Point origin;
    Affine3f transform = traj.get(frame_idx_).cast<float>();
    origin.getVector3fMap() = transform.translation();
    vis_.addArrow<Point, Point>(origin, pt, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, "line");
  }

  void SlamCalibrationVisualizer::keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie)
  {
    if(event.keyDown()) {
      char key = event.getKeyCode();
      if(key == 27) {
        scopeLockWrite;
        quitting_ = true;
      }
      else if(key == '>')
        incrementFrameIdx(30);
      else if(key == '<')
        incrementFrameIdx(-30);
      else if(key == '.')
        incrementFrameIdx(1);
      else if(key == ',')
        incrementFrameIdx(-1);
      else if(key == 's') {
        scopeLockWrite;
        show_frame_ = !show_frame_;
        needs_update_ = true;
        cout << "show_frame_: " << show_frame_ << endl;
      }
      else if(key == 'm') {
        scopeLockWrite;
        use_distortion_model_ = !use_distortion_model_;
        cout << "use_distortion_model_: " << use_distortion_model_ << endl;
        if(use_distortion_model_ && !dddm_)
          cout << "No distortion model provided." << endl;
        needs_update_ = true;
      }
      else if(key == 'c') {
        scopeLockWrite;
        color_frame_ = !color_frame_;
      }
    }
  }


  void SlamCalibrationVisualizer::setSequenceIdx(size_t idx)
  {
    cout << "Building map..." << endl;
    Cloud::Ptr map = calibrator_->buildMap(idx);
    cout << "Done building map." << endl;
    cout << map->size() << " points." << endl;

    scopeLockWrite;
    seq_idx_ = idx;
    map_ = map;
    needs_update_ = true;
  }

  void SlamCalibrationVisualizer::incrementSequenceIdx(int num)
  {
    int idx = (int)seq_idx_ + num;
    if(idx < 0)
      idx = 0;
    if((size_t)idx > calibrator_->size())
      idx = calibrator_->size();

    setSequenceIdx((size_t)idx);
  }

  void SlamCalibrationVisualizer::incrementFrameIdx(int num)
  {
    int idx = (int)frame_idx_;
    int incr = 1;
    if(num < 0)
      incr = -1;

    const Trajectory& traj = calibrator_->trajectories_[seq_idx_];
    while(num != 0) {
      // Find the next valid transform.
      while(true) {
        idx += incr;
        if(idx < 0)
          idx = traj.size() - 1;
        if(idx >= (int)traj.size())
          idx = 0;

        if(traj.exists(idx)) {
          break;
        }
      }
      num -= incr;
    }
  
    scopeLockWrite;
    vis_.removeAllShapes();
    frame_idx_ = (size_t)idx;
    needs_update_ = true;
  }

}  // namespace clams

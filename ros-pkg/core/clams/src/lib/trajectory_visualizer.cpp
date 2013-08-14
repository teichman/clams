#include <clams/trajectory_visualizer.h>

using namespace std;
using namespace Eigen;

namespace clams
{

  TrajectoryVisualizer::TrajectoryVisualizer(StreamSequenceBase::ConstPtr sseq,
                                             Trajectory traj, std::string title) :
                                             
    dddm_(NULL),
    max_range_(MAX_RANGE_MAP),
    vgsize_(DEFAULT_VGSIZE),
    sseq_(sseq),
    traj_(traj),
    map_(Cloud::Ptr(new Cloud)),
    quitting_(false),
    needs_update_(false),
    frame_idx_(0),
    show_frame_(false),
    use_distortion_model_(false),
    color_frame_(false),
    title_(title)
  {
    vis_.registerKeyboardCallback(&TrajectoryVisualizer::keyboardCallback, *this);
    vis_.registerPointPickingCallback(&TrajectoryVisualizer::pointPickingCallback, *this);
    vis_.setBackgroundColor(1, 1, 1);
    if(title != "")
      vis_.addText(title_, 10, 10, 16, 0, 0, 0, "title");

    Cloud::Ptr pcd(new Cloud);
    Point tmp(0, 0, 0);
    tmp.r = 0;
    tmp.g = 0;
    tmp.b = 0;
    pcd->push_back(tmp);  // PCLVis / VTK can spit out horrible garbage if you give it an empty pcd.
    vis_.addPointCloud(pcd, "map");
    vis_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "map");
  }

  void TrajectoryVisualizer::run()
  {
    // -- Build the map.
    cout << "Building map..." << endl;
    map_ = SlamCalibrator::buildMap(sseq_, traj_, max_range_, vgsize_);
    cout << "Done building map." << endl;
    cout << map_->size() << " points." << endl;
    needs_update_ = true;

    // -- Run the main visualization loop.
    incrementFrameIdx(1);
    while(true) {
      { scopeLockRead; if(quitting_) break; }
      usleep(5e3);
    
      lockWrite();
      if(needs_update_) {
        Cloud::Ptr pcd(new Cloud);
        *pcd = *map_;
        
        // -- Add the raw sensor data from the current frame.
        if(show_frame_) { 
          if(traj_.exists(frame_idx_)) {
            Frame pose_frame;
            sseq_->readFrame(frame_idx_, &pose_frame);
            if(dddm_ && use_distortion_model_) {
              ScopedTimer st("Undistorting");
              dddm_->undistort(pose_frame.depth_.get());
            }
            Cloud pose_pcd;
            sseq_->proj_.frameToCloud(pose_frame, &pose_pcd);
            if(!color_frame_) {
              for(size_t i = 0; i < pose_pcd.size(); ++i) {
                pose_pcd[i].r = 255;
                pose_pcd[i].g = 0;
                pose_pcd[i].b = 0;
              }
            }
          
            Affine3f transform = traj_.get(frame_idx_).cast<float>();
            pcl::transformPointCloud(pose_pcd, pose_pcd, transform);
            *pcd += pose_pcd;
          }
        }

        vis_.removeShape("title");
        vis_.addText(title_, 10, 10, 16, 0, 0, 0, "title");
      
        vis_.updatePointCloud(pcd, "map");
        needs_update_ = false;
      }

      vis_.removeCoordinateSystem();
      vis_.addCoordinateSystem(0.3, traj_.get(frame_idx_).cast<float>());
    
      unlockWrite();
      vis_.spinOnce(3);
    }

    // This PCL call doesn't appear to do anything.
    // The result is that the visualizer persists when it shouldn't.
    // This is a known bug. See
    // https://github.com/PointCloudLibrary/pcl/pull/85
    vis_.close();  
  }

  void TrajectoryVisualizer::pointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* cookie)
  {
    scopeLockRead;

    if(event.getPointIndex() == -1)
      return;
    if(!traj_.exists(frame_idx_))
      return;
    
    Point pt;
    event.getPoint(pt.x, pt.y, pt.z);
    cout << "Selected point: " << pt.x << ", " << pt.y << ", " << pt.z << endl;
    vis_.removeAllShapes();
    vis_.addText(title_, 10, 10, 16, 0, 0, 0, "title");
  
    Point origin;
    Affine3f transform = traj_.get(frame_idx_).cast<float>();
    origin.getVector3fMap() = transform.translation();
    vis_.addArrow<Point, Point>(origin, pt, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, "line");
  }

  void TrajectoryVisualizer::keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie)
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

  void TrajectoryVisualizer::incrementFrameIdx(int num)
  {
    int idx = (int)frame_idx_;
    int incr = 1;
    if(num < 0)
      incr = -1;

    while(num != 0) {
      // Find the next valid transform.
      while(true) {
        idx += incr;
        if(idx < 0)
          idx = traj_.size() - 1;
        if(idx >= (int)traj_.size())
          idx = 0;

        if(traj_.exists(idx)) {
          break;
        }
      }
      num -= incr;
    }
  
    scopeLockWrite;
    vis_.removeAllShapes();
    vis_.addText(title_, 10, 10, 16, 0, 0, 0, "title");
    frame_idx_ = (size_t)idx;
    needs_update_ = true;
  }

}  // namespace clams

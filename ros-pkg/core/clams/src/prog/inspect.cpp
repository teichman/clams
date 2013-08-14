#include <boost/program_options.hpp>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <clams/discrete_depth_distortion_model.h>
#include <bag_of_tricks/lockable.h>
#include <stream_sequence/frame_projector.h>

using namespace std;
using namespace Eigen;
using namespace clams;

class Inspector : public Lockable
{
public:
  DiscreteDepthDistortionModel* dddm_;
  
  void run();
  Inspector();
  
protected:
  Frame frame_;
  cv::Mat3b img_vis_;
  bool use_intrinsics_;
  pcl::visualization::PCLVisualizer pcd_vis_;
  std::vector< boost::shared_ptr<openni_wrapper::Image> > image_buffer_;
  std::vector< boost::shared_ptr<openni_wrapper::DepthImage> > depth_buffer_;
  bool quitting_;

  cv::Mat3b oniToCV(const openni_wrapper::Image& oni) const;
  void idiCallback(const boost::shared_ptr<openni_wrapper::Image>& img,
                   const boost::shared_ptr<openni_wrapper::DepthImage>& depth,
                   float callback);
  void updateDepth(const openni_wrapper::Image& img,
                   const openni_wrapper::DepthImage& depth);
  void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie);
  void pointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* cookie);
  void toggleModel();
  void enableModel();
  void disableModel();
};

Inspector::Inspector() :
  dddm_(NULL),
  use_intrinsics_(false),
  pcd_vis_("Cloud"),
  quitting_(false)
{
  frame_.depth_ = DepthMatPtr(new DepthMat(480, 640));
  frame_.depth_->setZero();

  pcd_vis_.addCoordinateSystem(0.3);
  pcd_vis_.setBackgroundColor(0, 0, 0);
  Cloud::Ptr cloud(new Cloud);
  pcd_vis_.addPointCloud(cloud, "cloud");
  pcd_vis_.registerKeyboardCallback(&Inspector::keyboardCallback, *this);
  pcd_vis_.registerPointPickingCallback(&Inspector::pointPickingCallback, *this);
  int x = 10;
  int y = 10;
  int fontsize = 16;
  pcd_vis_.addText("raw", x, y, fontsize, 1, 0, 0, "text");
  pcd_vis_.setShowFPS(false);
}

void Inspector::toggleModel()
{
  if(use_intrinsics_)
    disableModel();
  else
    enableModel();
}

void Inspector::disableModel()
{
  use_intrinsics_ = false;
  
  pcd_vis_.removeShape("text");
  int x = 10;
  int y = 10;
  int fontsize = 16;
  pcd_vis_.addText("raw", x, y, fontsize, 1, 0, 0, "text");
}

void Inspector::enableModel()
{
  use_intrinsics_ = true;
  if(use_intrinsics_ && !dddm_) {
    cout << "Cannot apply non-existent distortion model.  Supply one at the command line." << endl;
    use_intrinsics_ = false;
    return;
  }
  
  pcd_vis_.removeShape("text");
  int x = 10;
  int y = 10;
  int fontsize = 16;
  pcd_vis_.addText("undistorted", x, y, fontsize, 0, 1, 0, "text");
}

void Inspector::keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie)
{
  if(event.keyDown()) {
    if(event.getKeyCode() == 'm') {
      toggleModel();
    }
    else if(event.getKeyCode() == 'c') {
      pcd_vis_.removeAllShapes();
    }
    else if(event.getKeyCode() == 27) {
      quitting_ = true;
    }
  }
}

void Inspector::pointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* cookie)
{
  if(event.getPointIndex() == -1)
    return;
    
  Point pt;
  event.getPoint(pt.x, pt.y, pt.z);
  cout << "Selected point: " << pt.x << ", " << pt.y << ", " << pt.z << endl;
  pcd_vis_.removeAllShapes();
  
  Point origin(0, 0, 0);
  pcd_vis_.addArrow<Point, Point>(origin, pt, 0.0, 0.0, 0.8, 0.9, 0.9, 0.9, "line");
}

void Inspector::run()
{
  pcl::OpenNIGrabber::Mode mode = pcl::OpenNIGrabber::OpenNI_VGA_30Hz;
  cv::Size sz(640, 480);
  img_vis_ = cv::Mat3b(sz, cv::Vec3b(0, 0, 0));
  
  pcl::OpenNIGrabber grabber("", mode, mode);
  boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&,
                        const boost::shared_ptr<openni_wrapper::DepthImage>&,
                        float)> cb;
  cb = boost::bind(&Inspector::idiCallback, this, _1, _2, _3);
  grabber.registerCallback(cb);
  grabber.start();

  while(!quitting_) {
    lock();
    if(!image_buffer_.empty()) {
      updateDepth(*image_buffer_.back(), *depth_buffer_.back());
    }
    image_buffer_.clear();
    depth_buffer_.clear();
    unlock();
  }
}

cv::Mat3b Inspector::oniToCV(const openni_wrapper::Image& oni) const
{
  cv::Mat3b img(oni.getHeight(), oni.getWidth());
  uchar data[img.rows * img.cols * 3];
  oni.fillRGB(img.cols, img.rows, data);
  int i = 0;
  for(int y = 0; y < img.rows; ++y) {
    for(int x = 0; x < img.cols; ++x, i+=3) {
      img(y, x)[0] = data[i+2];
      img(y, x)[1] = data[i+1];
      img(y, x)[2] = data[i];
    }
  }
    
  return img;
}

void Inspector::updateDepth(const openni_wrapper::Image& image,
                            const openni_wrapper::DepthImage& depth)
{
  frame_.depth_->setZero();
  ushort data[depth.getHeight() * depth.getWidth()];
  depth.fillDepthImageRaw(depth.getWidth(), depth.getHeight(), data);
  int i = 0;
  for(size_t y = 0; y < depth.getHeight(); ++y) {
    for(size_t x = 0; x < depth.getWidth(); ++x, ++i) {
      if(data[i] == depth.getNoSampleValue() || data[i] == depth.getShadowValue())
        continue;
      frame_.depth_->coeffRef(y, x) = data[i];
    }
  }

  if(dddm_ && use_intrinsics_)
    dddm_->undistort(frame_.depth_.get());

  frame_.img_ = oniToCV(image);
    
  Cloud::Ptr cloud(new Cloud);
  FrameProjector proj;
  proj.width_ = 640;
  proj.height_ = 480;
  proj.cx_ = proj.width_ / 2;
  proj.cy_ = proj.height_ / 2;
  proj.fx_ = 525;
  proj.fy_ = 525;
  proj.frameToCloud(frame_, cloud.get());
  pcd_vis_.updatePointCloud(cloud, "cloud");
  pcd_vis_.spinOnce(5);
}

void Inspector::idiCallback(const boost::shared_ptr<openni_wrapper::Image>& img,
                            const boost::shared_ptr<openni_wrapper::DepthImage>& depth,
                            float constant)
{
  lock();
  image_buffer_.push_back(img);
  depth_buffer_.push_back(depth);
  unlock();
}

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  namespace bfs = boost::filesystem;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  opts_desc.add_options()
    ("help,h", "produce help message")
    ("intrinsics", bpo::value<string>(), "Optional distortion model.")
    ;

  p.add("intrinsics", 1);
  
  bpo::variables_map opts;
  bool badargs = false;
  try {
    bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
    bpo::notify(opts);
  }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << bfs::basename(argv[0]) << " [INTRINSICS]" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  int retval = system("killall XnSensorServer 2> /dev/null"); --retval;
  
  Inspector inspector;
  if(opts.count("intrinsics")) {
    inspector.dddm_ = new DiscreteDepthDistortionModel;
    inspector.dddm_->load(opts["intrinsics"].as<string>());
    cout << "Loading DiscreteDepthDistortionModel at " << opts["intrinsics"].as<string>() << endl;
  }
    
  inspector.run();

  if(inspector.dddm_)
    delete inspector.dddm_;
  
  return 0;
}

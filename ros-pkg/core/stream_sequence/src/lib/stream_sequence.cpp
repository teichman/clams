#include <stream_sequence/stream_sequence.h>

using namespace std;
namespace bfs = boost::filesystem;

namespace clams
{

  StreamSequence::StreamSequence() :
    max_depth_(numeric_limits<double>::max())
  {
  }
  
  void StreamSequence::save() const
  {
    ROS_ASSERT(0);
    // ROS_ASSERT(proj_.initialized());
    // proj_.save(root_path_ + "/primesense_model");

    // ROS_ASSERT(timestamps_.size() == clk_names_.size());
    // for(size_t i = 0; i < timestamps_.size(); ++i) { 
    //   ofstream fs((root_path_ + "/" + clk_names_[i]).c_str());
    //   fs.precision(10);
    //   fs.setf(ios::fixed, ios::floatfield);
    //   fs << timestamps_[i] << endl;
    //   fs.close();
    // }
  }

  void StreamSequence::init(const std::string& root_path)
  {
    ROS_ASSERT(!bfs::exists(root_path));
    root_path_ = root_path;
    bfs::create_directory(root_path_);
  }

  void StreamSequence::loadImpl(const std::string& dir)
  {
    root_path_ = dir;
    ROS_WARN("Using old, deprecated StreamSequence.  FrameProjector is being constructed manually.");
    proj_.width_ = 640; 
    proj_.height_ = 480;
    proj_.fx_ = 525;
    proj_.fy_ = 525;
    proj_.cx_ = proj_.width_ / 2;
    proj_.cy_ = proj_.height_ / 2;
    cout << "FrameProjector: " << endl;
    cout << proj_.status("  ") << endl;
    
    // -- Build filename index.
    img_names_.clear();
    dpt_names_.clear();
    clk_names_.clear();
    bfs::recursive_directory_iterator it(root_path_), eod;
    BOOST_FOREACH(const bfs::path& p, make_pair(it, eod)) {
      ROS_ASSERT(is_regular_file(p));
      if(p.leaf().string().substr(0, 3).compare("img") == 0 &&
         (bfs::extension(p).compare(".ppm") == 0 ||
          bfs::extension(p).compare(".png") == 0))
      {
        img_names_.push_back(p.leaf().string());
      }
      else if(bfs::extension(p).compare(".eig") == 0)
        dpt_names_.push_back(p.leaf().string());
      else if(bfs::extension(p).compare(".clk") == 0)
        clk_names_.push_back(p.leaf().string());
    }
    ROS_ASSERT(img_names_.size() == dpt_names_.size());
    ROS_ASSERT(img_names_.size() == clk_names_.size());

    // -- Sort all filenames.
    sort(img_names_.begin(), img_names_.end());
    sort(dpt_names_.begin(), dpt_names_.end());
    sort(clk_names_.begin(), clk_names_.end());

    // -- Load timestamps.
    timestamps_.resize(clk_names_.size());
    for(size_t i = 0; i < clk_names_.size(); ++i) {
      ifstream fs((root_path_ + "/" + clk_names_[i]).c_str());
      ROS_ASSERT(fs.is_open());
      fs >> timestamps_[i];
      fs.close();
    }
  }
  
  void StreamSequence::writeFrame(const Frame& frame)
  {
    ostringstream oss;    
    size_t idx = size();
    
    // -- Write image
    vector<int> params;
    if(getenv("SSEQ_PPM"))
      oss << "img" << setw(5) << setfill('0') << idx << ".ppm";
    else
      oss << "img" << setw(5) << setfill('0') << idx << ".png";
    cv::imwrite(root_path_ + "/" + oss.str(), frame.img_, params);
    img_names_.push_back(oss.str());

    // -- Write depth
    oss.str("");
    oss << "dpt" << setw(5) << setfill('0') << idx << ".eig";
    eigen_extensions::save(*frame.depth_, root_path_ + "/" + oss.str());
    dpt_names_.push_back(oss.str());

    // -- Write timestamp and add to index
    oss.str("");
    oss << "clk" << setw(5) << setfill('0') << idx << ".clk";
    ofstream fs((root_path_ + "/" + oss.str()).c_str());
    fs.precision(10);
    fs.setf(ios::fixed, ios::floatfield);
    fs << frame.timestamp_ << endl;
    fs.close();
    clk_names_.push_back(oss.str());
    
    timestamps_.push_back(frame.timestamp_);
  }

  void StreamSequence::readFrameImpl(size_t idx, Frame* frame) const
  {
    ROS_ASSERT(idx < img_names_.size());
    frame->img_ = cv::imread(root_path_ + "/" + img_names_[idx], 1);
    frame->depth_ = DepthMatPtr(new DepthMat);
    eigen_extensions::load(root_path_ + "/" + dpt_names_[idx], frame->depth_.get());
    frame->timestamp_ = timestamps_[idx];
    
    if(max_depth_ != numeric_limits<double>::max())
      for(int y = 0; y < frame->depth_->rows(); ++y)
        for(int x = 0; x < frame->depth_->cols(); ++x)
          if(frame->depth_->coeffRef(y, x) > max_depth_ * 1000)
            frame->depth_->coeffRef(y, x) = 0;

  }



  size_t StreamSequence::size() const
  {
    ROS_ASSERT(img_names_.size() == dpt_names_.size());
    ROS_ASSERT(img_names_.size() == clk_names_.size());
    ROS_ASSERT(img_names_.size() == timestamps_.size());
    return img_names_.size();
  }


  
  
} // namespace rgbd



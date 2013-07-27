#include <clams/discrete_depth_distortion_model.h>

using namespace std;
using namespace Eigen;
namespace bfs = boost::filesystem;

namespace clams
{

  DiscreteFrustum::DiscreteFrustum(int smoothing, double bin_depth) :
    max_dist_(10),
    bin_depth_(bin_depth)
  {
    num_bins_ = ceil(max_dist_ / bin_depth_);
    counts_ = VectorXf::Ones(num_bins_) * smoothing;
    total_numerators_ = VectorXf::Ones(num_bins_) * smoothing;
    total_denominators_ = VectorXf::Ones(num_bins_) * smoothing;
    multipliers_ = VectorXf::Ones(num_bins_);
  }

  void DiscreteFrustum::addExample(double ground_truth, double measurement)
  {
    scopeLockWrite;
  
    double mult = ground_truth / measurement;
    if(mult > MAX_MULT || mult < MIN_MULT)
      return;
  
    int idx = min(num_bins_ - 1, (int)floor(measurement / bin_depth_));
    ROS_ASSERT(idx >= 0);
    total_numerators_(idx) += ground_truth * ground_truth;
    total_denominators_(idx) += ground_truth * measurement;
    ++counts_(idx);
    multipliers_(idx) = total_numerators_(idx) / total_denominators_(idx);
  }

  inline int DiscreteFrustum::index(double z) const
  {
    return min(num_bins_ - 1, (int)floor(z / bin_depth_));
  }
  
  inline void DiscreteFrustum::undistort(double* z) const
  {
    *z *= multipliers_.coeffRef(index(*z));
  }

  void DiscreteFrustum::interpolatedUndistort(double* z) const
  {
    int idx = index(*z);
    double start = bin_depth_ * idx;
    int idx1;
    if(*z - start < bin_depth_ / 2)
      idx1 = idx;
    else
      idx1 = idx + 1;
    int idx0 = idx1 - 1;
    if(idx0 < 0 || idx1 >= num_bins_ || counts_(idx0) < 50 || counts_(idx1) < 50) {
      undistort(z);
      return;
    }

    double z0 = (idx0 + 1) * bin_depth_ - bin_depth_ * 0.5;
    double coeff1 = (*z - z0) / bin_depth_;
    double coeff0 = 1.0 - coeff1;
    double mult = coeff0 * multipliers_.coeffRef(idx0) + coeff1 * multipliers_.coeffRef(idx1);
    *z *= mult;
  }

  void DiscreteFrustum::serialize(std::ostream& out) const
  {
    eigen_extensions::serializeScalar(max_dist_, out);
    eigen_extensions::serializeScalar(num_bins_, out);
    eigen_extensions::serializeScalar(bin_depth_, out);
    eigen_extensions::serialize(counts_, out);
    eigen_extensions::serialize(total_numerators_, out);
    eigen_extensions::serialize(total_denominators_, out);
    eigen_extensions::serialize(multipliers_, out);
  }

  void DiscreteFrustum::deserialize(std::istream& in)
  {
    eigen_extensions::deserializeScalar(in, &max_dist_);
    eigen_extensions::deserializeScalar(in, &num_bins_);
    eigen_extensions::deserializeScalar(in, &bin_depth_);
    eigen_extensions::deserialize(in, &counts_);
    eigen_extensions::deserialize(in, &total_numerators_);
    eigen_extensions::deserialize(in, &total_denominators_);
    eigen_extensions::deserialize(in, &multipliers_);
  }

  DiscreteDepthDistortionModel::DiscreteDepthDistortionModel(const DiscreteDepthDistortionModel& other)
  {
    *this = other;
  }

  DiscreteDepthDistortionModel& DiscreteDepthDistortionModel::operator=(const DiscreteDepthDistortionModel& other)
  {
    proj_ = other.proj_;
    width_ = other.width_;
    height_ = other.height_;
    bin_width_ = other.bin_width_;
    bin_height_ = other.bin_height_;
    bin_depth_ = other.bin_depth_;
    num_bins_x_ = other.num_bins_x_;
    num_bins_y_ = other.num_bins_y_;
  
    frustums_ = other.frustums_;
    for(size_t i = 0; i < frustums_.size(); ++i)
      for(size_t j = 0; j < frustums_[i].size(); ++j)
        frustums_[i][j] = new DiscreteFrustum(*other.frustums_[i][j]);

    return *this;
  }

  DiscreteDepthDistortionModel::DiscreteDepthDistortionModel(const FrameProjector& proj,
                                                             int bin_width, int bin_height, double bin_depth,
                                                             int smoothing) :
    proj_(proj),
    bin_width_(bin_width),
    bin_height_(bin_height),
    bin_depth_(bin_depth)
  {
    ROS_ASSERT(proj_.width_ % bin_width_ == 0);
    ROS_ASSERT(proj_.height_ % bin_height_ == 0);

    num_bins_x_ = proj_.width_ / bin_width_;
    num_bins_y_ = proj_.height_ / bin_height_;
  
    frustums_.resize(num_bins_y_);
    for(size_t i = 0; i < frustums_.size(); ++i) {
      frustums_[i].resize(num_bins_x_, NULL);
      for(size_t j = 0; j < frustums_[i].size(); ++j)
        frustums_[i][j] = new DiscreteFrustum(smoothing, bin_depth);
    }
  }

  void DiscreteDepthDistortionModel::deleteFrustums()
  {
    for(size_t y = 0; y < frustums_.size(); ++y)
      for(size_t x = 0; x < frustums_[y].size(); ++x)
        if(frustums_[y][x])
          delete frustums_[y][x];
  }

  DiscreteDepthDistortionModel::~DiscreteDepthDistortionModel()
  {
    deleteFrustums();
  }

  void DiscreteDepthDistortionModel::undistort(Frame* frame) const
  {
    ROS_ASSERT(proj_.width_ == frame->depth_->cols());
    ROS_ASSERT(proj_.height_ == frame->depth_->rows());

    ProjectivePoint ppt;
    Point pt;
#pragma omp parallel for
    for(int v = 0; v < proj_.height_; ++v) {
      for(int u = 0; u < proj_.width_; ++u) {
        if(frame->depth_->coeffRef(v, u) == 0)
          continue;

        double z = frame->depth_->coeffRef(v, u) * 0.001;
        frustum(v, u).interpolatedUndistort(&z);
        frame->depth_->coeffRef(v, u) = z * 1000;
      }
    }
  }

  void DiscreteDepthDistortionModel::addExample(const ProjectivePoint& ppt, double ground_truth, double measurement)
  {
    frustum(ppt.v_, ppt.u_).addExample(ground_truth, measurement);
  }

  size_t DiscreteDepthDistortionModel::accumulate(const Frame& ground_truth, const Frame& measurement)
  {
    ROS_ASSERT(proj_.width_ == ground_truth.depth_->cols());
    ROS_ASSERT(proj_.height_ == ground_truth.depth_->rows());
    ROS_ASSERT(proj_.width_ == measurement.depth_->cols());
    ROS_ASSERT(proj_.height_ == measurement.depth_->rows());

    size_t num_training_examples = 0;
    ProjectivePoint ppt;
    Point pt;
    for(ppt.v_ = 0; ppt.v_ < proj_.height_; ++ppt.v_) {
      for(ppt.u_ = 0; ppt.u_ < proj_.width_; ++ppt.u_) {
        if(ground_truth.depth_->coeffRef(ppt.v_, ppt.u_) == 0)
          continue;
        if(measurement.depth_->coeffRef(ppt.v_, ppt.u_) == 0)
          continue;

        double gt = ground_truth.depth_->coeffRef(ppt.v_, ppt.u_) * 0.001;
        double meas = measurement.depth_->coeffRef(ppt.v_, ppt.u_) * 0.001;
        frustum(ppt.v_, ppt.u_).addExample(gt, meas);
        ++num_training_examples;
      }
    }

    return num_training_examples;
  }

  void DiscreteDepthDistortionModel::serialize(std::ostream& out) const
  {
    out << "DiscreteDepthDistortionModel v01" << endl;
    proj_.serialize(out);
    eigen_extensions::serializeScalar(bin_width_, out);
    eigen_extensions::serializeScalar(bin_height_, out);
    eigen_extensions::serializeScalar(bin_depth_, out);
    eigen_extensions::serializeScalar(num_bins_x_, out);
    eigen_extensions::serializeScalar(num_bins_y_, out);

    for(int y = 0; y < num_bins_y_; ++y)
      for(int x = 0; x < num_bins_x_; ++x) 
        frustums_[y][x]->serialize(out);
  }

  void DiscreteDepthDistortionModel::deserialize(std::istream& in)
  {
    string buf;
    getline(in, buf);
    ROS_ASSERT(buf == "DiscreteDepthDistortionModel v01");
    proj_.deserialize(in);
    eigen_extensions::deserializeScalar(in, &bin_width_);
    eigen_extensions::deserializeScalar(in, &bin_height_);
    eigen_extensions::deserializeScalar(in, &bin_depth_);
    eigen_extensions::deserializeScalar(in, &num_bins_x_);
    eigen_extensions::deserializeScalar(in, &num_bins_y_);

    deleteFrustums();
    frustums_.resize(num_bins_y_);
    for(size_t y = 0; y < frustums_.size(); ++y) {
      frustums_[y].resize(num_bins_x_, NULL);
      for(size_t x = 0; x < frustums_[y].size(); ++x) {
        frustums_[y][x] = new DiscreteFrustum;
        frustums_[y][x]->deserialize(in);
      }
    }
  }

  void DiscreteDepthDistortionModel::visualize(const std::string& dir) const
  {
    if(!bfs::exists(dir))
      bfs::create_directory(dir);
    else
      ROS_ASSERT(bfs::is_directory(dir));
  
    const DiscreteFrustum& reference_frustum = *frustums_[0][0];
    int num_layers = reference_frustum.num_bins_;

    // -- Set up for combined imagery.
    int horiz_divider = 10;
    int vert_divider = 20;
    cv::Mat3b overview(cv::Size(proj_.width_ * 2 + horiz_divider, proj_.height_ * num_layers + vert_divider * (num_layers + 2)), cv::Vec3b(0, 0, 0));
    vector<int> pub_layers;
    for(int i = 0; i < num_layers; ++i)
      pub_layers.push_back(i);
    // pub_layers.push_back(1);
    // pub_layers.push_back(2);
    // pub_layers.push_back(3);
    cv::Mat3b pub(cv::Size(proj_.width_, proj_.height_ * pub_layers.size() + vert_divider * (pub_layers.size() + 2)), cv::Vec3b(255, 255, 255));
  
    for(int i = 0; i < num_layers; ++i) {
      // -- Determine the path to save the image for this layer.
      char buffer[50];
      float mindepth = reference_frustum.bin_depth_ * i;
      float maxdepth = reference_frustum.bin_depth_ * (i + 1);
      sprintf(buffer, "%05.2f-%05.2f", mindepth, maxdepth);
      ostringstream oss;
      oss << dir << "/multipliers_" << buffer << ".png";

      // -- Compute the multipliers visualization for this layer.
      //    Multiplier of 1 is black, >1 is red, <1 is blue.  Think redshift.
      cv::Mat3b mult(cv::Size(proj_.width_, proj_.height_), cv::Vec3b(0, 0, 0));
      for(int y = 0; y < mult.rows; ++y) {
        for(int x = 0; x < mult.cols; ++x) {
          const DiscreteFrustum& frustum = *frustums_[y / bin_height_][x / bin_width_];
          float val = frustum.multipliers_(i);
          if(val > 1)
            mult(y, x)[2] = min(255., 255 * (val - 1.0) / 0.25);
          if(val < 1)
            mult(y, x)[0] = min(255., 255 * (1.0 - val) / 0.25);
        }
      }
      cv::imwrite(oss.str(), mult);

      // -- Compute the counts visualization for this layer.
      //    0 is black, 100 is white.
      cv::Mat3b count(cv::Size(proj_.width_, proj_.height_), cv::Vec3b(0, 0, 0));
      for(int y = 0; y < count.rows; ++y) {
        for(int x = 0; x < count.cols; ++x) {
          const DiscreteFrustum& frustum = *frustums_[y / bin_height_][x / bin_width_];
          uchar val = min(255., (double)(255 * frustum.counts_(i) / 100));
          count(y, x)[0] = val;
          count(y, x)[1] = val;
          count(y, x)[2] = val;
        }
      }
      oss.str("");
      oss << dir << "/counts_" << buffer << ".png";
      cv::imwrite(oss.str(), count);

      // -- Make images showing the two, side-by-side.
      cv::Mat3b combined(cv::Size(proj_.width_ * 2 + horiz_divider, proj_.height_), cv::Vec3b(0, 0, 0));
      for(int y = 0; y < combined.rows; ++y) {
        for(int x = 0; x < combined.cols; ++x) {
          if(x < count.cols)
            combined(y, x) = count(y, x);
          else if(x > count.cols + horiz_divider)
            combined(y, x) = mult(y, x - count.cols - horiz_divider);
        }
      }
      oss.str("");
      oss << dir << "/combined_" << buffer << ".png";
      cv::imwrite(oss.str(), combined);

      // -- Append to the overview image.
      for(int y = 0; y < combined.rows; ++y)
        for(int x = 0; x < combined.cols; ++x)
          overview(y + i * (combined.rows + vert_divider) + vert_divider, x) = combined(y, x);

      // -- Compute the publication multipliers visualization for this layer.
      //    Multiplier of 1 is white, >1 is red, <1 is blue.  Think redshift.
      cv::Mat3b pubmult(cv::Size(proj_.width_, proj_.height_), cv::Vec3b(255, 255, 255));
      for(int y = 0; y < pubmult.rows; ++y) {
        for(int x = 0; x < pubmult.cols; ++x) {
          const DiscreteFrustum& frustum = *frustums_[y / bin_height_][x / bin_width_];
          float val = frustum.multipliers_(i);
          if(val > 1) {
            pubmult(y, x)[0] = 255 - min(255., 255 * (val - 1.0) / 0.1);
            pubmult(y, x)[1] = 255 - min(255., 255 * (val - 1.0) / 0.1);
          }
          if(val < 1) {
            pubmult(y, x)[1] = 255 - min(255., 255 * (1.0 - val) / 0.1);
            pubmult(y, x)[2] = 255 - min(255., 255 * (1.0 - val) / 0.1);
          }
        }
      }
  
      // -- Append to publication image.
      for(size_t j = 0; j < pub_layers.size(); ++j)
        if(pub_layers[j] == i)
          for(int y = 0; y < pubmult.rows; ++y)
            for(int x = 0; x < pubmult.cols; ++x)
              pub(y + j * (pubmult.rows + vert_divider) + vert_divider, x) = pubmult(y, x);
    }
  
    // -- Add a white bar at the top and bottom for reference.
    for(int y = 0; y < overview.rows; ++y)
      if(y < vert_divider || y > overview.rows - vert_divider)
        for(int x = 0; x < overview.cols; ++x)
          overview(y, x) = cv::Vec3b(255, 255, 255);
  
    // -- Save overview image.
    ostringstream oss;
    oss << dir << "/overview.png";
    cv::imwrite(oss.str(), overview);

    // -- Save a small version for easy loading.
    cv::Mat3b overview_scaled;
    cv::resize(overview, overview_scaled, cv::Size(), 0.2, 0.2, cv::INTER_CUBIC);
    oss.str("");
    oss << dir << "/overview_scaled.png";
    cv::imwrite(oss.str(), overview_scaled);

    // -- Save publication image.
    oss.str("");
    oss << dir << "/pub";
    // for(size_t i = 0; i < pub_layers.size(); ++i)
    //   oss << "-" << setw(2) << setfill('0') << pub_layers[i];
    oss << ".png";
    cv::imwrite(oss.str(), pub);
  }
  
  DiscreteFrustum& DiscreteDepthDistortionModel::frustum(int y, int x)
  {
    ROS_ASSERT(x >= 0 && x < proj_.width_);
    ROS_ASSERT(y >= 0 && y < proj_.height_);
    int xidx = x / bin_width_;
    int yidx = y / bin_height_;
    return (*frustums_[yidx][xidx]);
  }

  const DiscreteFrustum& DiscreteDepthDistortionModel::frustum(int y, int x) const
  {
    ROS_ASSERT(x >= 0 && x < proj_.width_);
    ROS_ASSERT(y >= 0 && y < proj_.height_);
    int xidx = x / bin_width_;
    int yidx = y / bin_height_;
    return (*frustums_[yidx][xidx]);
  }

}  // namespace clams

#include <stream_sequence/frame_projector.h>

using namespace std;
using namespace Eigen;

namespace clams
{

  void ProjectivePoint::serialize(std::ostream& out) const
  {
    out << setprecision(16) << u_ << " " << v_ << " " << z_
        << " " << (int)r_ << " " << (int)g_ << " " << (int)b_ << endl;
  }

  void ProjectivePoint::deserialize(std::istream& in)
  {
    in >> u_;
    in >> v_;
    in >> z_;
    in >> r_;
    in >> g_;
    in >> b_;

    string buf;
    getline(in, buf);
  }

  cv::Vec3b Frame::colorize(double depth, double min_range, double max_range) const
  {
    if(depth == 0)
      return cv::Vec3b(0, 0, 0);
    
    double increment = (max_range - min_range) / 3;
    double thresh0 = min_range;
    double thresh1 = thresh0 + increment;
    double thresh2 = thresh1 + increment;
    double thresh3 = thresh2 + increment;
    
    if(depth < thresh0) {
      return cv::Vec3b(0, 0, 255);
    }
    if(depth >= thresh0 && depth < thresh1) {
      int val = (depth - thresh0) / (thresh1 - thresh0) * 255.;
      return cv::Vec3b(val, val, 255 - val);
    }
    else if(depth >= thresh1 && depth < thresh2) {
      int val = (depth - thresh1) / (thresh2 - thresh1) * 255.;
      return cv::Vec3b(255, 255 - val, 0);
    }
    else if(depth >= thresh2 && depth < thresh3) {
      int val = (depth - thresh2) / (thresh3 - thresh2) * 255.;
      return cv::Vec3b(255 - val, val, 0);
    }
    
    return cv::Vec3b(0, 255, 0);
  }
  
  cv::Mat3b Frame::depthImage() const
  {
    cv::Mat3b depth(depth_->rows(), depth_->cols());
    depth = cv::Vec3b(0, 0, 0);
    for(int y = 0; y < depth.rows; ++y)
      for(int x = 0; x < depth.cols; ++x)
        depth(y, x) = colorize(depth_->coeffRef(y, x) * 0.001, 0, 10);
    return depth;
  }

  void Frame::serialize(std::ostream& out) const
  {
    eigen_extensions::serializeScalar(timestamp_, out);
    eigen_extensions::serialize(*depth_, out);

    vector<uchar> buf;
    cv::imencode(".png", img_, buf);
    size_t num_bytes = buf.size();
    out.write((const char*)&num_bytes, sizeof(num_bytes));
    out.write((const char*)&buf[0], num_bytes * sizeof(uchar));
  }

  void Frame::deserialize(std::istream& in)
  {
    eigen_extensions::deserializeScalar(in, &timestamp_);
    depth_ = DepthMatPtr(new DepthMat);
    eigen_extensions::deserialize(in, depth_.get());

    size_t num_bytes;
    in.read((char*)&num_bytes, sizeof(num_bytes));
    vector<uchar> buf(num_bytes);
    in.read((char*)&buf[0], num_bytes * sizeof(uchar));
    img_ = cv::imdecode(buf, 1);
  }
  
  FrameProjector::FrameProjector() :
    width_(-1),
    height_(-1),
    cx_(-1),
    cy_(-1),
    fx_(-1),
    fy_(-1)
  {
  }

  void FrameProjector::cloudToRangeIndex(const Cloud& pcd, RangeIndex* rindex) const
  {
    RangeIndex& ind = *rindex;
    if((int)ind.size() != height_)
      ind.resize(height_);
    for(size_t y = 0; y < ind.size(); ++y)
      if((int)ind[y].size() != width_)
        ind[y].resize(width_);
    for(size_t y = 0; y < ind.size(); ++y) {
      for(size_t x = 0; x < ind[y].size(); ++x) { 
        ind[y][x].clear();
        ind[y][x].reserve(10);
      }
    }

    ProjectivePoint ppt;
    for(size_t i = 0; i < pcd.size(); ++i) {
      if(!isFinite(pcd[i]))
        continue;
      project(pcd[i], &ppt);
      if(ppt.z_ == 0 || !(ppt.u_ >= 0 && ppt.v_ >= 0 && ppt.u_ < width_ && ppt.v_ < height_))
        continue;
      ind[ppt.v_][ppt.u_].push_back(pcd[i].z);
    }
  }
  
  void FrameProjector::cloudToFrame(const Cloud& pcd, Frame* frame, IndexMap* indexmap) const
  {
    ROS_ASSERT(frame);
    ROS_ASSERT(width_ != -1 && height_ != -1 && cx_ != -1 && cy_ != -1 && fx_ != -1 && fy_ != -1);

    frame->timestamp_ = pcd.header.stamp * 1e-9;
    frame->depth_ = DepthMatPtr(new DepthMat(height_, width_));
    frame->depth_->setZero();  // 0 indicates a bad point.
    frame->img_ = cv::Mat3b(height_, width_);

    if(indexmap)
    {
      *indexmap = IndexMap(height_, width_);
    }

    ProjectivePoint ppt;
    for(size_t i = 0; i < pcd.size(); ++i) {
      if(!isFinite(pcd[i]))
        continue;
      
      // Ignore points outside the depth image or behind the sensor.
      project(pcd[i], &ppt);
      if(ppt.z_ <= 0 || !(ppt.u_ >= 0 && ppt.v_ >= 0 && ppt.u_ < width_ && ppt.v_ < height_))
        continue;

      // Eigen is column-major by default: http://eigen.tuxfamily.org/dox/TopicStorageOrders.html
      // opencv is row-major
      // pcl is row-major:
      //cout << "u, v: " << ppt.u_ << " " << ppt.v_ << endl;
      
      // Take the closest point in pcd.
      unsigned short curr_depth = frame->depth_->coeffRef(ppt.v_, ppt.u_);
      if(curr_depth == 0 || ppt.z_ < curr_depth) { 
        frame->depth_->coeffRef(ppt.v_, ppt.u_) = ppt.z_;
        frame->img_(ppt.v_, ppt.u_)[0] = ppt.b_;
        frame->img_(ppt.v_, ppt.u_)[1] = ppt.g_;
        frame->img_(ppt.v_, ppt.u_)[2] = ppt.r_;
        if(indexmap) {
          (*indexmap)(ppt.v_, ppt.u_) = i;
        }
      }
    }
  }
  
  void FrameProjector::frameToCloud(const Frame& frame, Cloud* pcd, double max_range) const
  {    
    const DepthMat& dm = *frame.depth_;
    cv::Mat3b img = frame.img_;

    ROS_ASSERT(fx_ > 0 && fy_ > 0 && cx_ > 0 && cy_ > 0);
    ROS_ASSERT(dm.rows() == img.rows);
    ROS_ASSERT(dm.cols() == img.cols);
    ROS_ASSERT(img.rows == height_);
    ROS_ASSERT(img.cols == width_);
    
    pcd->clear();
    pcd->height = dm.rows();
    pcd->width = dm.cols();
    pcd->is_dense = false;
    pcd->resize(dm.rows() * dm.cols());
    pcd->header.stamp = (frame.timestamp_) * 1e9;

    int idx = 0;
    ProjectivePoint ppt;
    for(ppt.v_ = 0; ppt.v_ < dm.rows(); ++ppt.v_) {
      for(ppt.u_ = 0; ppt.u_ < dm.cols(); ++ppt.u_, ++idx) {
        ppt.z_ = dm(ppt.v_, ppt.u_);
        if(ppt.z_ > max_range * 1000)
          ppt.z_ = 0;  // bad point.

        ppt.r_ = img(ppt.v_, ppt.u_)[2];
        ppt.g_ = img(ppt.v_, ppt.u_)[1];
        ppt.b_ = img(ppt.v_, ppt.u_)[0];
        project(ppt, &pcd->at(idx));
      }
    }
  }
  
  void FrameProjector::project(const ProjectivePoint& ppt, Point* pt) const
  {
    ROS_ASSERT(ppt.u_ >= 0 && ppt.v_ >= 0 && ppt.u_ < width_ && ppt.v_ < height_);

    pt->r = ppt.r_;
    pt->g = ppt.g_;
    pt->b = ppt.b_;

    if(ppt.z_ == 0) {
      pt->x = std::numeric_limits<float>::quiet_NaN();
      pt->y = std::numeric_limits<float>::quiet_NaN();
      pt->z = std::numeric_limits<float>::quiet_NaN();
    }
    else {
      pt->z = ppt.z_ * 0.001;
      pt->x = pt->z * (ppt.u_ - cx_) / fx_;
      pt->y = pt->z * (ppt.v_ - cy_) / fy_;
    }
  }
  
  void FrameProjector::project(const Point& pt, ProjectivePoint* ppt) const
  {
    ROS_ASSERT(isFinite(pt));
    
    ppt->u_ = pt.x * fx_ / pt.z + cx_;
    ppt->v_ = pt.y * fy_ / pt.z + cy_;
    ppt->z_ = pt.z * 1000;
    ppt->r_ = pt.r;
    ppt->g_ = pt.g;
    ppt->b_ = pt.b;

    if(pt.z < 0)
      ppt->z_ = 0;
  }
  
  bool FrameProjector::initialized() const
  {
    if(cx_ == -1 || cy_ == -1)
      return false;
    if(fx_ == -1 || fy_ == -1)
      return false;
    if(width_ == -1 || height_ == -1)
      return false;

    return true;
  }

  void FrameProjector::serialize(std::ostream& out) const
  {
    ROS_ASSERT(width_ != -1);
    ROS_ASSERT(height_ != -1);
    ROS_ASSERT(fx_ != -1);
    ROS_ASSERT(fy_ != -1);
    ROS_ASSERT(cx_ != -1);
    ROS_ASSERT(cy_ != -1);

    out << "FrameProjector v01" << endl;
    eigen_extensions::serializeScalar(width_, out);
    eigen_extensions::serializeScalar(height_, out);
    eigen_extensions::serializeScalar(fx_, out);
    eigen_extensions::serializeScalar(fy_, out);
    eigen_extensions::serializeScalar(cx_, out);
    eigen_extensions::serializeScalar(cy_, out);
  }

  void FrameProjector::deserialize(std::istream& in)
  {
    string buf;
    getline(in, buf);
    ROS_ASSERT(buf == "FrameProjector v01");
    eigen_extensions::deserializeScalar(in, &width_);
    eigen_extensions::deserializeScalar(in, &height_);
    eigen_extensions::deserializeScalar(in, &fx_);
    eigen_extensions::deserializeScalar(in, &fy_);
    eigen_extensions::deserializeScalar(in, &cx_);
    eigen_extensions::deserializeScalar(in, &cy_);
  }

  
  std::string FrameProjector::status(const std::string& prefix) const
  {
    ostringstream oss;
    oss << prefix << "size: " << width_ << " x " << height_ << endl;
    oss << prefix << "fx: " << fx_ << endl;
    oss << prefix << "fy: " << fy_ << endl;
    oss << prefix << "cx: " << cx_ << endl;
    oss << prefix << "cy: " << cy_ << endl;
    
    return oss.str();
  }

  void FrameProjector::estimateMapDepth(const Cloud& map, const Eigen::Affine3f& transform,
                                         const Frame& measurement,
                                         DepthMat* estimate) const
  {
    // -- Reallocate estimate if necessary.
    if(estimate->rows() != measurement.depth_->rows() ||
       estimate->cols() != measurement.depth_->cols())
    {
      *estimate = DepthMat(measurement.depth_->rows(), measurement.depth_->cols());
    }
    estimate->setZero();

    // -- Get the depth index.
    Cloud transformed;
    transformPointCloud(map, transformed, transform);
    RangeIndex rindex;
    cloudToRangeIndex(transformed, &rindex);

    // -- Compute the edge-of-map mask.
    Frame naive_mapframe;
    cloudToFrame(transformed, &naive_mapframe);
    const DepthMat& measurement_depth = *measurement.depth_;
    const DepthMat& naive_mapdepth = *naive_mapframe.depth_;
    cv::Mat1b mask(measurement_depth.rows(), measurement_depth.cols());
    mask = 0;
    for(int y = 0; y < mask.rows; ++y)
      for(int x = 0; x < mask.cols; ++x)
        if(naive_mapdepth(y, x) != 0)
          mask(y, x) = 255;
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 4);
    cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 15);

    // -- Main loop: for all points in the image...
    ProjectivePoint ppt;
    Point pt;
    for(ppt.v_ = 0; ppt.v_ < measurement_depth.rows(); ++ppt.v_) {
      for(ppt.u_ = 0; ppt.u_ < measurement_depth.cols(); ++ppt.u_) {
        // -- Reject points with no data.
        if(measurement_depth(ppt.v_, ppt.u_) == 0)
          continue;
        if(naive_mapdepth(ppt.v_, ppt.u_) == 0)
          continue;
        
        // -- Reject points on the edge of the map.
        if(mask(ppt.v_, ppt.u_) == 0)
          continue;

        // -- Find nearby points in the cone to get a good estimate of the map depth.
        double radius = 0.02;
        double mean = 0;
        double stdev = 0;
        //double stdev_thresh = numeric_limits<double>::max();
        double stdev_thresh = 0.03;
        bool valid = coneFit(naive_mapdepth, rindex, ppt.u_, ppt.v_, radius, measurement_depth(ppt.v_, ppt.u_) * 0.001, &mean, &stdev);
        if(!valid)
          continue;
        if(stdev > stdev_thresh)
          continue;

        (*estimate)(ppt.v_, ppt.u_) = mean * 1000;
      }
    }
  }

  bool FrameProjector::coneFit(const DepthMat& naive_mapdepth, const RangeIndex& rindex,
                                int uc, int vc, double radius, double measurement_depth,
                                double* mean, double* stdev) const
  {
    Point pt_center, pt_ul, pt_lr;
    ProjectivePoint ppt, ppt_ul, ppt_lr;
    ppt.u_ = uc;
    ppt.v_ = vc;
    ppt.z_ = (ushort)(measurement_depth * 1000);
    project(ppt, &pt_center);

    pt_ul = pt_center;
    pt_lr = pt_center;
    pt_ul.x -= radius;
    pt_ul.y -= radius;
    pt_lr.x += radius;
    pt_lr.y += radius;

    project(pt_ul, &ppt_ul);
    project(pt_lr, &ppt_lr);
    if(ppt_ul.z_ == 0 || !(ppt_ul.u_ >= 0 && ppt_ul.v_ >= 0 && ppt_ul.u_ < naive_mapdepth.cols() && ppt_ul.v_ < naive_mapdepth.rows()))
      return false;
    if(ppt_lr.z_ == 0 || !(ppt_lr.u_ >= 0 && ppt_lr.v_ >= 0 && ppt_lr.u_ < naive_mapdepth.cols() && ppt_lr.v_ < naive_mapdepth.rows()))
      return false;

    int min_u = ppt_ul.u_;
    int max_u = ppt_lr.u_;
    int min_v = ppt_ul.v_;
    int max_v = ppt_lr.v_;

    *mean = 0;
    double num = 0;
    for(ppt.u_ = min_u; ppt.u_ <= max_u; ++ppt.u_) {
      for(ppt.v_ = min_v; ppt.v_ <= max_v; ++ppt.v_) {
        const vector<double>& vals = rindex[ppt.v_][ppt.u_];
        for(size_t i = 0; i < vals.size(); ++i) {
          double mult = vals[i] / measurement_depth;
          if(mult > MIN_MULT && mult < MAX_MULT) {
            *mean += vals[i];
            ++num;
          }
        }
      }
    }
    if(num == 0)
      return false;
    *mean /= num;

    double var = 0;
    for(ppt.u_ = min_u; ppt.u_ <= max_u; ++ppt.u_) {
      for(ppt.v_ = min_v; ppt.v_ <= max_v; ++ppt.v_) {
        const vector<double>& vals = rindex[ppt.v_][ppt.u_];
        for(size_t i = 0; i < vals.size(); ++i) {
          double mult = vals[i] / measurement_depth;
          if(mult > MIN_MULT && mult < MAX_MULT)
            var += (vals[i] - *mean) * (vals[i] - *mean);
        }
      }
    }
    var /= num;
    
    *stdev = sqrt(var);
    return true;
  }
  
}  // namespace clams


#include <primesense_slam/mean_depth_error.h>
#include <ros/package.h>

using namespace std;
using namespace Eigen;

//#define TIMING
#define VISUALIZE

namespace clams
{

  Eigen::Affine3f generateTransform(double rx, double ry, double rz,
                                    double tx, double ty, double tz)
  {
    Affine3f Rx, Ry, Rz, T;
    Rx = Eigen::AngleAxisf(rx, Vector3f(1, 0, 0));
    Ry = Eigen::AngleAxisf(ry, Vector3f(0, 1, 0));
    Rz = Eigen::AngleAxisf(rz, Vector3f(0, 0, 1));
    T = Eigen::Translation3f(tx, ty, tz);
    return T * Rz * Ry * Rx;
  }

  void generateXYZYPR(const Eigen::Affine3f &trans, 
                      double &rx, double &ry, double &rz, double &tx, double &ty, double &tz)
  {
    Eigen::Vector3f xyz = trans.translation();
    tx = xyz(0); ty = xyz(1); tz = xyz(2);
    Eigen::Matrix3f R = trans.rotation();
    rx = atan2(R(2,1),R(2,2));
    ry = atan2( -R(2,0), sqrt(pow(R(2,1),2)+pow(R(2,2),2)) );
    rz = atan2( R(1,0), R(0,0) );
  }
  
// Takes points from frame0, turns them in to lines in the coordinate system of frame1, then finds how far keypoints in frame1 are
// from the lines they should lie on.
  void keypointError(const FrameProjector& model0, Frame frame0, const std::vector<cv::Point2d> correspondences0,
                     const Eigen::Affine3f& f0_to_f1,
                     const FrameProjector& model1, Frame frame1, const std::vector<cv::Point2d>& correspondences1,
                     double keypoint_hinge,
                     double* keypoint_error, double* keypoint_error_count)
  {
    ROS_ASSERT(correspondences0.size() == correspondences1.size());

    cv::Mat3b vis = frame1.img_.clone();
  
    // -- Find the location of the origin of frame0 in the image of frame1.
    Point originpt;
    originpt.getVector3fMap() = f0_to_f1.translation();
    ProjectivePoint originppt;
    model1.project(originpt, &originppt);
    // cout << "@@@@@@@@@@" << endl;
    // cout << f0_to_f1.matrix() << endl;
    // cout << "originpt: " << originpt.getVector3fMap().transpose() << ", originppt: " << originppt.u_ << " " << originppt.v_ << endl;
  
    ProjectivePoint ppt;
    Point pt;
    for(size_t i = 0; i < correspondences0.size(); ++i) {
    
      // -- Get a 3D test point along the ray.
      ppt.u_ = correspondences0[i].x;
      ppt.v_ = correspondences0[i].y;
      ppt.z_ = 5000;

      Point testpt;
      model0.project(ppt, &testpt);
      testpt.getVector3fMap() = testpt.getVector3fMap() * 100000;
      testpt.getVector3fMap() = f0_to_f1 * testpt.getVector3fMap();
    
      // -- Project the test point and origin point into frame1.
      ProjectivePoint testppt;
      model1.project(testpt, &testppt);
      //cout << "testpt: " << testpt.getVector3fMap().transpose() << ", testppt: " << testppt.u_ << " " << testppt.v_ << endl;

      // Ignore edge case.  Could do distance to point here, but that probably doesn't really matter.
      if(originppt.u_ == testppt.u_ && originppt.v_ == testppt.v_)
        continue;
    
      // -- Get the error for this keypoint.
      Vector2d origin;
      origin(0) = originppt.u_;
      origin(1) = originppt.v_;
      Vector2d test;
      test(0) = testppt.u_;
      test(1) = testppt.v_;
      Vector2d v = test - origin;
      Vector2d normal;
      normal(0) = -v(1);
      normal(1) = v(0);
      normal.normalize();
      double b = normal.dot(origin);
      if(!(fabs(b - normal.dot(test)) < 1e-6)) { 
        ROS_FATAL_STREAM("b: " << b << ", origin: " << origin.transpose() << ", v: " << v.transpose() << ", normal: " << normal.transpose() << ", test: " << test.transpose() << ", normal.dot(test): " << normal.dot(test));
        abort();
      }
      Vector2d p;
      p(0) = correspondences1[i].x;
      p(1) = correspondences1[i].y;

      Vector2d pp = p - (normal.dot(p) - b) / (normal.dot(normal)) * normal;
      //    cout << "fabs(normal.dot(pp) - b): " << fabs(normal.dot(pp) - b) << endl;
      ROS_ASSERT(fabs(normal.dot(pp) - b) < 1e-6);
      double error = min(keypoint_hinge, fabs(normal.dot(p - pp)));
      *keypoint_error += error;
      //*keypoint_error += fabs(normal.dot(p - pp));
      ++(*keypoint_error_count);


      // -- Add to visualization.
      // cv::Scalar color(rand() % 255, rand() % 255, rand() % 255);
      // cv::circle(vis, correspondences1[i], 2, color, -1);
      // cv::Point2d cvorigin, cvtestpt;
      // cvorigin.x = originppt.u_;
      // cvorigin.y = originppt.v_;
      // cvtestpt.x = testppt.u_;
      // cvtestpt.y = testppt.v_;
      // cv::line(vis, cvorigin, cvtestpt, color, 1);

      // cout << "cvtestpt: " << cvtestpt << endl;
      // cout << "error: " << error << endl;
    }
    // cv::imshow("epipolar", vis);
    // cv::waitKey(3);
  }

  FrameAlignmentMDE::FrameAlignmentMDE(const Params& params,
                                       const FrameProjector& model0, const FrameProjector& model1,
                                       Frame frame0, Frame frame1,
                                       const std::vector<cv::Point2d>& correspondences0, const std::vector<cv::Point2d>& correspondences1) :
    count_(NULL),
    depth_error_(NULL),
    params_(params),
    model0_(model0),
    model1_(model1),
    frame0_(frame0),
    frame1_(frame1),
    correspondences0_(correspondences0),
    correspondences1_(correspondences1)
  {
#ifdef TIMING
    ScopedTimer st("FrameAlignmentMDE::FrameAlignmentMDE.  frameToClouds.");
#endif

    ROS_ASSERT(correspondences0_.size() == correspondences1_.size());

    model0_.frameToCloud(frame0_, &pcd0_, params_.get<double>("max_range"));
    model1_.frameToCloud(frame1_, &pcd1_, params_.get<double>("max_range"));
    ROS_ASSERT(pcd0_.size() == pcd1_.size());

  
    // Set up which random pixels to look at.
    // (Calling rand from multiple execution threads is a disaster)
    indices_.reserve(pcd0_.size());
    for(size_t i = 0; i < pcd0_.size(); ++i)
      if((double)rand() / RAND_MAX <= params_.get<double>("fraction"))
        indices_.push_back(i);

// #ifdef VISUALIZE
//   cv::Mat3b vis0 = frame0.img_.clone();
//   cv::Mat3b vis1 = frame1.img_.clone();
//   for(size_t i = 0; i < correspondences0.size(); ++i) {
//     cv::Scalar color(rand() % 255, rand() % 255, rand() % 255);
//     cv::circle(vis0, correspondences0[i], 2, color, -1);
//     cv::circle(vis1, correspondences1[i], 2, color, -1);
//   }
//   cv::imshow("vis0", vis0);
//   cv::imshow("vis1", vis1);
//   cv::waitKey(5);
// #endif
//
    // Precompute hue, Canny image, etc here
    // HUE
    cv::Mat3f img0_f; frame0_.img_.convertTo(img0_f, CV_32F, 1/255.);
    cv::Mat3f img1_f; frame1_.img_.convertTo(img1_f, CV_32F, 1/255.);
    cv::cvtColor(img0_f, img0_hsv_, CV_BGR2HSV);
    cv::cvtColor(img1_f, img1_hsv_, CV_BGR2HSV);
    // CANNY (TODO)
    cv::Mat1b gray0; cv::cvtColor(frame0_.img_, gray0, CV_BGR2GRAY);
    cv::Mat1b gray1; cv::cvtColor(frame1_.img_, gray1, CV_BGR2GRAY);
    int canny_k = params_.get<int>("canny_kernel_radius")*2+1;
    cv::Mat1b rawedges0; cv::Canny(gray0, rawedges0, 
                                   params_.get<int>("canny_lower_thresh"), params_.get<int>("canny_upper_thresh"), canny_k);
    cv::Mat1b rawedges1; cv::Canny(gray1, rawedges1,
                                   params_.get<int>("canny_lower_thresh"), params_.get<int>("canny_upper_thresh"), canny_k);
    //cv::Mat1b dilated0; cv::dilate(rawedges0, dilated0, cv::Mat1b::ones(1,1)); 
    //cv::Mat1b dilated1; cv::dilate(rawedges1, dilated1, cv::Mat1b::ones(1,1)); 
    //cv::GaussianBlur(dilated0, edges0_, cv::Size(35,35), 15);
    //cv::GaussianBlur(dilated1, edges1_, cv::Size(35,35), 15);
    int ndilate=params_.get<int>("edge_fanout");
    edges0_ = cv::Mat1b::zeros(rawedges0.size());
    edges1_ = cv::Mat1b::zeros(rawedges1.size());
    cv::Mat1b dilated0; 
    cv::Mat1b dilated1; 
    for(int i = 0; i < ndilate; i++)
    {
      float scale = 1./(ndilate*2); //LINEAR
      //float scale = (i+1)/(float)( (ndilate)*(ndilate+1)/2.);
      cv::dilate(rawedges0, dilated0, cv::Mat1b::ones(i*2+1,i*2+1)); 
      cv::dilate(rawedges1, dilated1, cv::Mat1b::ones(i*2+1,i*2+1)); 
      edges0_ += scale * dilated0;
      edges1_ += scale * dilated1;
    }
    //cv::imshow("raw_edges", rawedges0);
    //cv::imshow("edges", edges0_); 
    //cv::waitKey(30);
    // Load the color_names lookup table
    eigen_extensions::load(ros::package::getPath("primesense_slam") + "/data/w2c.eig", 
                           &color_names_);

  }

  double FrameAlignmentMDE::eval(const Eigen::VectorXd& x) const
  {
#ifdef TIMING
    ScopedTimer st("FrameAlignmentMDE::eval");
#endif 
    Eigen::Affine3f f0_to_f1 = generateTransform(x(0), x(1), x(2), x(3), x(4), x(5));

    double count = 0;  // Total number of points with both ground truth and measurements.
    double val = 0;  // Total objective.
    double depth_error = 0;
    double garbage;
    double color_error = 0;
    // Specific weightings for different metrics
    double rgb_error = 0;
    double hue_error = 0;
    double edge_error = 0;
    double cn_error = 0;
    double rgb_count = 0;
    double hue_count = 0;
    double edge_count = 0;
    double cn_count = 0;

    Cloud transformed;
    transformAndDecimate(pcd1_, f0_to_f1.inverse(), indices_, &transformed);
    Frame transformed_frame;
    IndexMap indexmap;
    model0_.cloudToFrame(transformed, &transformed_frame, &indexmap);
    if(params_.get<double>("rgb_weight") > 0 || params_.get<double>("depth_weight") > 0)
      meanDepthMultiplierAndColorError(model0_, frame0_, transformed, transformed_frame, &depth_error, &rgb_error, &rgb_count, params_.get<double>("max_range"));
    if(params_.get<double>("hue_weight") > 0)
      meanDepthMultiplierAndHueError(model0_, frame0_, transformed, transformed_frame, indexmap, img0_hsv_, img1_hsv_, indices_, &garbage, &hue_error, &hue_count, params_.get<double>("max_range"));
    if(params_.get<double>("edge_weight") > 0)
      meanDepthMultiplierAndEdgeError(model0_, frame0_, transformed, transformed_frame, indexmap, edges0_, edges1_, indices_, &garbage, &edge_error, &edge_count, params_.get<double>("max_range"));
    if(params_.get<double>("cn_weight") > 0)
      meanDepthMultiplierAndCNError(model0_, frame0_, transformed, transformed_frame, color_names_, &garbage, &cn_error, &cn_count, params_.get<double>("max_range"));
    // keypointError(model0_, frame0_, correspondences0_, f0_to_f1, model1_, frame1_, correspondences1_,
    //                 params_.get<double>("keypoint_hinge"), &keypoint_error, &keypoint_error_count);
  
    transformAndDecimate(pcd0_, f0_to_f1, indices_, &transformed); 
    model1_.cloudToFrame(transformed, &transformed_frame, &indexmap);
    if(params_.get<double>("rgb_weight") > 0 || params_.get<double>("depth_weight") > 0)
      meanDepthMultiplierAndColorError(model1_, frame1_, transformed, transformed_frame, &depth_error, &rgb_error, &rgb_count, params_.get<double>("max_range"));
    if(params_.get<double>("hue_weight") > 0)
      meanDepthMultiplierAndHueError(model1_, frame1_, transformed, transformed_frame, indexmap, img1_hsv_, img0_hsv_, indices_, &garbage, &hue_error, &hue_count, params_.get<double>("max_range"));
    if(params_.get<double>("edge_weight") > 0)
      meanDepthMultiplierAndEdgeError(model1_, frame1_, transformed, transformed_frame, indexmap, edges1_, edges0_, indices_, &garbage, &edge_error, &edge_count, params_.get<double>("max_range"));
    if(params_.get<double>("cn_weight") > 0)
      meanDepthMultiplierAndCNError(model1_, frame1_, transformed, transformed_frame, color_names_, &garbage, &cn_error, &cn_count, params_.get<double>("max_range"));

    // keypointError(model1_, frame1_, correspondences1_, f0_to_f1.inverse(), model0_, frame0_, correspondences0_,
    //                 params_.get<double>("keypoint_hinge"), &keypoint_error, &keypoint_error_count);

    // int min_correspondences = 20;
    // if(keypoint_error_count < min_correspondences) {
    //   ROS_WARN("Ignoring candidate alignment because of min_correspondences.");
    //   return numeric_limits<double>::max();
    // }
    // else
    //   keypoint_error /= keypoint_error_count;
    //cout << "Avg error: " << color_error / count << endl;
    double min_points = 100;
    count = rgb_count; //TODO make it actually count just depth regardless of method
    if(count < min_points) {
      ROS_WARN("FrameAlignmentMDE had < min_points overlapping 3d points.");
      return numeric_limits<double>::max();
    }
    else {
      // Aggregate them all together
      depth_error /= count;
      if(rgb_count > 0) rgb_error /= rgb_count;
      if(hue_count > 0) hue_error /= hue_count;
      if(edge_count > 0) edge_error /= edge_count;
      if(cn_count > 0) cn_error /= cn_count;
      color_error = 
        params_.get<double>("rgb_weight")*rgb_error +
        params_.get<double>("hue_weight")*hue_error +
        params_.get<double>("edge_weight")*edge_error +
        params_.get<double>("cn_weight")*cn_error;
    }

    //cout << "Num correspondences used for keypoint error: " << keypoint_error_count << ", Keypoint error: " << keypoint_error << endl;

    // Color error term has a per-pixel max of 441.
    // Keypoint error is hinged at 50.  It's probably a bit weaker than the 3D data, though, so we want it
    // to just nudge things when the 3D doesn't really have a preference.
    double depth_term = params_.get<double>("depth_weight") * depth_error;
    double color_term = params_.get<double>("color_weight") * color_error;
    //double keypoint_term = params_.get<double>("keypoint_weight") * keypoint_error;
    double keypoint_term = 0;
    val = depth_term + color_term + keypoint_term;
    //cout << "Depth fraction: " << depth_term / val << ", color fraction: " << color_term / val << ", keypoint fraction: " << keypoint_term / val << endl;

    // Make data available to other users in single-threaded mode.
    if(count_)
      *count_ = count;
    if(depth_error_)
      *depth_error_ = depth_error;
  
    return val;
  }

  void transformAndDecimate(const Cloud& in,
                            const Eigen::Affine3f& transform,
                            const std::vector<size_t>& indices,
                            Cloud* out)
  {
#ifdef TIMING
    ScopedTimer st("transformAndDecimate");
#endif
    out->clear();
    out->reserve(indices.size());  
    for(size_t i = 0; i < indices.size(); ++i) {
      size_t idx = indices[i];
      if(idx >= in.size())
        break;
      out->push_back(Point());
      out->back().getVector4fMap() = transform * in[idx].getVector4fMap();
      out->back().r = in[idx].r;
      out->back().g = in[idx].g;
      out->back().b = in[idx].b;
    }
  }

  SequenceAlignmentMDE::SequenceAlignmentMDE(const FrameProjector& model,
                                             const std::vector<Frame>& frames,
                                             const std::vector<Cloud::ConstPtr>& pcds) :
    model_(model),
    frames_(frames),
    pcds_(pcds),
    dt_thresh_(0.005)
  {
  }

  int seek(const std::vector<Frame>& frames, double ts1, double dt_thresh)
  {
    int idx = -1;
    double min = numeric_limits<double>::max();
    // TODO: This could be faster than linear search.
    for(size_t i = 0; i < frames.size(); ++i) {
      double ts0 = frames[i].timestamp_;
      double dt = fabs(ts0 - ts1);
      if(dt < min) {
        min = dt;
        idx = i;
      }
    }

    if(min < dt_thresh)
      return idx;
    else
      return -1;
  }

  double SequenceAlignmentMDE::eval(const Eigen::VectorXd& x) const
  {
    double offset = x(0);
    Eigen::Affine3f transform = generateTransform(x(1), x(2), x(3), x(4), x(5), x(6));

    double count = 0;  // Total number of points with both ground truth and measurements.
    double val = 0;  // Total objective.
    Cloud transformed;
    Frame transformed_frame;
    int num_pcds = 0;
    for(size_t i = 0; i < pcds_.size(); ++i) {
      int idx = seek(frames_, offset + pcds_[i]->header.stamp * 1e-9, dt_thresh_);
      if(idx == -1)
        continue;

      pcl::transformPointCloud(*pcds_[i], transformed, transform);
      model_.cloudToFrame(transformed, &transformed_frame);
      meanDepthError(model_, frames_[idx], transformed, transformed_frame, &val, &count);
      ++num_pcds;
    }

    double min_per_pcd = 1000;
    if(count / num_pcds < min_per_pcd) {
      //ROS_WARN_STREAM("Mean number of corresponding points per pcd is " << count / num_pcds << ", which is less than threshold of " << min_per_pcd);
      return std::numeric_limits<double>::max();
    }
    else
      return val / count;
  }

  FocalLengthMDE::FocalLengthMDE(const FrameProjector& model,
                                 const std::vector<Frame>& frames,
                                 const std::vector<Cloud::ConstPtr>& pcds,
                                 const std::vector<Eigen::Affine3d>& transforms,
                                 double fraction) :
    model_(model),
    frames_(frames),
    pcds_(pcds),
    transforms_(transforms)
  {
    // Set up which random pixels to look at.
    // (Calling rand from multiple execution threads is a disaster)
    size_t max_num_pts = 0;
    for(size_t i = 0; i < pcds.size(); ++i)
      max_num_pts = max(max_num_pts, pcds[i]->size());
  
    indices_.reserve(max_num_pts);
    for(size_t i = 0; i < max_num_pts; ++i)
      if((double)rand() / RAND_MAX <= fraction)
        indices_.push_back(i);
  }

  double FocalLengthMDE::eval(const Eigen::VectorXd& x) const
  {
    FrameProjector model = model_;
    model.fx_ = x(0);
    model.fy_ = x(0);

    double count = 0;  // Total number of points with both ground truth and measurements.
    double val = 0;  // Total objective.
    Cloud transformed;
    Frame transformed_frame;
    for(size_t i = 0; i < pcds_.size(); ++i) {
      transformAndDecimate(*pcds_[i], transforms_[i].cast<float>(), indices_, &transformed);
      model.cloudToFrame(transformed, &transformed_frame);

      //pcl::transformPointCloud(*pcds_[i], transformed, transforms_[i].cast<float>());
      meanDepthError(model, frames_[i], transformed, transformed_frame, &val, &count);
    }

    double min_count = 100 * pcds_.size();
    if(count < min_count) {
      ROS_WARN_STREAM("Number of corresponding pcds is less than the threshold of " << min_count);
      return std::numeric_limits<double>::max();
    }
    else
      return val / count;
  }

  void meanDepthError(const FrameProjector& model,
                      const Frame &frame, const Cloud& pcd, const Frame &gt,
                      double* val, double* count, double max_range)
  {
    //ScopedTimer st("meanDepthError total");
    ROS_ASSERT(frame.depth_->rows() == model.height_);
    ROS_ASSERT(frame.depth_->cols() == model.width_);
    HighResTimer hrt;
  
    // -- Make the ground truth depth image.
    hrt.reset("meanDepthError: cloudToFrame"); hrt.start();
    //Frame gt;
    //model.cloudToFrame(pcd, &gt);
#ifdef TIMING
    hrt.stop(); cout << hrt.reportMilliseconds() << endl;
#endif 

    // -- Count up mean depth error.
    hrt.reset("meanDepthError: counting"); hrt.start();
    ProjectivePoint ppt;
    Point pt;
    Point gtpt;
    double max_range_mm = max_range * 1000;
    double val_mm = 0;
    for(ppt.u_ = 0; ppt.u_ < gt.depth_->cols(); ++ppt.u_) {
      for(ppt.v_ = 0; ppt.v_ < gt.depth_->rows(); ++ppt.v_) {
        // -- Both ground truth and measurement must have data.
        double gtz = gt.depth_->coeffRef(ppt.v_, ppt.u_);
        if(gtz == 0)
          continue;
        double z = frame.depth_->coeffRef(ppt.v_, ppt.u_);
        if(z == 0)
          continue;
      
        // -- Ignore measured points beyond max_range.
        if(z > max_range_mm)
          continue;

        // -- Ignore points for which both are far away.
        // if(frame.depth_->coeffRef(ppt.v_, ppt.u_) > max_range * 1000 &&
        //          gt.depth_->coeffRef(ppt.v_, ppt.u_) > max_range * 1000)
        // {
        //         continue;
        // }


        // -- Count up range error.
        // ppt.z_ = gt.depth_->coeffRef(ppt.v_, ppt.u_);
        // model.project(ppt, &gtpt);
        // ppt.z_ = frame.depth_->coeffRef(ppt.v_, ppt.u_);
        // model.project(ppt, &pt);
        // *val += (pt.getVector3fMap() - gtpt.getVector3fMap()).norm();

        // -- Count up z error.
        val_mm += fabs(z - gtz);
      
        ++(*count);
      }
    }
    *val += val_mm * 0.001;
  
#ifdef TIMING
    hrt.stop(); cout << hrt.reportMilliseconds() << endl;
#endif 
  }

  void meanDepthAndColorError(const FrameProjector& model,
                              const Frame &frame, const Cloud& pcd,
                              const Frame &gt,
                              double* depth_error, double* color_error,
                              double* count, double max_range)
  {
    ROS_ASSERT(frame.depth_->rows() == model.height_);
    ROS_ASSERT(frame.depth_->cols() == model.width_);
    HighResTimer hrt;
  
    // -- Make the ground truth depth image.
    hrt.reset("meanDepthError: cloudToFrame"); hrt.start();
    //Frame gt;
    //model.cloudToFrame(pcd, &gt);
#ifdef TIMING
    hrt.stop(); cout << hrt.reportMilliseconds() << endl;
#endif 

    // -- Count up mean depth error.
    hrt.reset("meanDepthError: counting"); hrt.start();
    ProjectivePoint ppt;
    Point pt;
    Point gtpt;
    double max_range_mm = max_range * 1000;
    double depth_error_mm = 0;
    double local_color_error = 0;

    for(ppt.u_ = 0; ppt.u_ < gt.depth_->cols(); ++ppt.u_) {
      for(ppt.v_ = 0; ppt.v_ < gt.depth_->rows(); ++ppt.v_) {
        // -- Both ground truth and measurement must have data.
        double gtz = gt.depth_->coeffRef(ppt.v_, ppt.u_);
        if(gtz == 0)
          continue;
        double z = frame.depth_->coeffRef(ppt.v_, ppt.u_);
        if(z == 0)
          continue;
      
        // -- Ignore measured points beyond max_range.
        if(z > max_range_mm)
          continue;

        // -- Count up z error.
        depth_error_mm += fabs(z - gtz);
      
        // -- Count up color error.
        cv::Vec3b gtc = gt.img_(ppt.v_, ppt.u_);
        cv::Vec3b c = frame.img_(ppt.v_, ppt.u_);
        local_color_error += sqrt((gtc[0] - c[0]) * (gtc[0] - c[0]) +
                                  (gtc[1] - c[1]) * (gtc[1] - c[1]) +
                                  (gtc[2] - c[2]) * (gtc[2] - c[2]));
        //local_color_error += fabs((double)gtc[0] - c[0]) + fabs((double)gtc[1] - c[1]) + fabs((double)gtc[2] - c[2]);
              
        ++(*count);
      }
    }
    *depth_error += depth_error_mm * 0.001;
    *color_error += local_color_error;
  
#ifdef TIMING
    hrt.stop(); cout << hrt.reportMilliseconds() << endl;
#endif 
  }

  void meanDepthMultiplierAndColorError(const FrameProjector& model,
                                        const Frame &frame, const Cloud& pcd,
                                        const Frame &gt,
                                        double* depth_error, double* color_error,
                                        double* count, double max_range)
  {
    ROS_ASSERT(frame.depth_->rows() == model.height_);
    ROS_ASSERT(frame.depth_->cols() == model.width_);
    HighResTimer hrt;
  
    // -- Make the ground truth depth image.
    hrt.reset("meanDepthError: cloudToFrame"); hrt.start();
    //Frame gt;
    //model.cloudToFrame(pcd, &gt);
#ifdef TIMING
    hrt.stop(); cout << hrt.reportMilliseconds() << endl;
#endif 

    // -- Count up mean depth error.
    hrt.reset("meanDepthError: counting"); hrt.start();
    ProjectivePoint ppt;
    Point pt;
    Point gtpt;
    double max_range_mm = max_range * 1000;
    double local_depth_error = 0;
    double local_color_error = 0;

    for(ppt.u_ = 0; ppt.u_ < gt.depth_->cols(); ++ppt.u_) {
      for(ppt.v_ = 0; ppt.v_ < gt.depth_->rows(); ++ppt.v_) {
        // -- Both ground truth and measurement must have data.
        double gtz = gt.depth_->coeffRef(ppt.v_, ppt.u_);
        if(gtz == 0)
          continue;
        double z = frame.depth_->coeffRef(ppt.v_, ppt.u_);
        if(z == 0)
          continue;
      
        // -- Ignore measured points beyond max_range.
        if(z > max_range_mm)
          continue;

        // -- Count up z error.
        local_depth_error += fabs(1.0 - z / gtz);

        // -- Count up color error.
        cv::Vec3b gtc = gt.img_(ppt.v_, ppt.u_);
        cv::Vec3b c = frame.img_(ppt.v_, ppt.u_);
        local_color_error += sqrt((gtc[0] - c[0]) * (gtc[0] - c[0]) +
                                  (gtc[1] - c[1]) * (gtc[1] - c[1]) +
                                  (gtc[2] - c[2]) * (gtc[2] - c[2]));
        //local_color_error += fabs((double)gtc[0] - c[0]) + fabs((double)gtc[1] - c[1]) + fabs((double)gtc[2] - c[2]);
              
        ++(*count);
      }
    }
    *depth_error += local_depth_error;
    *color_error += local_color_error;
  
#ifdef TIMING
    hrt.stop(); cout << hrt.reportMilliseconds() << endl;
#endif 
  }

  void meanDepthMultiplierAndHueError(const FrameProjector& model,
                                      const Frame &frame, const Cloud& pcd,
                                      const Frame &gt, const IndexMap &indexmap,
                                      const cv::Mat3f &hsv_frame, const cv::Mat3f &hsv_pcd,
                                      const std::vector<size_t> &cloud_indices, 
                                      double* depth_error, double* color_error,
                                      double* count, double max_range)
  {
    ROS_ASSERT(frame.depth_->rows() == model.height_);
    ROS_ASSERT(frame.depth_->cols() == model.width_);
    HighResTimer hrt;
  
    // -- Make the ground truth depth image.
    hrt.reset("meanDepthError: cloudToFrame"); hrt.start();
    //Frame gt;
    //IndexMap indexmap;
    //model.cloudToFrame(pcd, &gt, &indexmap);
#ifdef TIMING
    hrt.stop(); cout << hrt.reportMilliseconds() << endl;
#endif 

    // -- Count up mean depth error.
    hrt.reset("meanDepthError: counting"); hrt.start();
    ProjectivePoint ppt;
    Point pt;
    Point gtpt;
    double max_range_mm = max_range * 1000;
    double local_depth_error = 0;
    double local_color_error = 0;

    for(ppt.u_ = 0; ppt.u_ < gt.depth_->cols(); ++ppt.u_) {
      for(ppt.v_ = 0; ppt.v_ < gt.depth_->rows(); ++ppt.v_) {
        // -- Both ground truth and measurement must have data.
        double gtz = gt.depth_->coeffRef(ppt.v_, ppt.u_);
        if(gtz == 0)
          continue;
        double z = frame.depth_->coeffRef(ppt.v_, ppt.u_);
        if(z == 0)
          continue;
      
        // -- Ignore measured points beyond max_range.
        if(z > max_range_mm)
          continue;

        // -- Count up z error.
        local_depth_error += fabs(1.0 - z / gtz);

        // -- Count up color error.
        size_t idx = indexmap(ppt.v_, ppt.u_);
        size_t orig_idx = cloud_indices[idx];
        int gtu = orig_idx % model.width_;
        int gtv = orig_idx / model.width_;
        cv::Vec3f gth = hsv_pcd(gtv, gtu);
        cv::Vec3f h = hsv_frame(ppt.v_, ppt.u_);
        float hdiff = fabs(gth[0] - h[0]);
        if(hdiff >= 180) 
          hdiff -= 180;
        float avg_sat = (gth[1]+h[1])/2.;
        local_color_error += avg_sat * hdiff;
        //local_color_error += fabs((double)gtc[0] - c[0]) + fabs((double)gtc[1] - c[1]) + fabs((double)gtc[2] - c[2]);
              
        ++(*count);
      }
    }
    *depth_error += local_depth_error;
    *color_error += local_color_error;
  
#ifdef TIMING
    hrt.stop(); cout << hrt.reportMilliseconds() << endl;
#endif 
  }

  void meanDepthMultiplierAndEdgeError(const FrameProjector& model,
                                       const Frame &frame, const Cloud& pcd,
                                       const Frame &gt, const IndexMap &indexmap,
                                       const cv::Mat1b &edge_frame, const cv::Mat1b &edge_pcd,
                                       const std::vector<size_t> &cloud_indices, 
                                       double* depth_error, double* color_error,
                                       double* count, double max_range)
  {
    ROS_ASSERT(frame.depth_->rows() == model.height_);
    ROS_ASSERT(frame.depth_->cols() == model.width_);
    HighResTimer hrt;
  
    // -- Make the ground truth depth image.
    hrt.reset("meanDepthError: cloudToFrame"); hrt.start();
    //Frame gt;
    //IndexMap indexmap;
    //model.cloudToFrame(pcd, &gt, &indexmap);
#ifdef TIMING
    hrt.stop(); cout << hrt.reportMilliseconds() << endl;
#endif 

    // -- Count up mean depth error.
    hrt.reset("meanDepthError: counting"); hrt.start();
    ProjectivePoint ppt;
    Point pt;
    Point gtpt;
    double max_range_mm = max_range * 1000;
    double local_depth_error = 0;
    double local_color_error = 0;

    for(ppt.u_ = 0; ppt.u_ < gt.depth_->cols(); ++ppt.u_) {
      for(ppt.v_ = 0; ppt.v_ < gt.depth_->rows(); ++ppt.v_) {
        // -- Both ground truth and measurement must have data.
        double gtz = gt.depth_->coeffRef(ppt.v_, ppt.u_);
        if(gtz == 0)
          continue;
        double z = frame.depth_->coeffRef(ppt.v_, ppt.u_);
        if(z == 0)
          continue;
      
        // -- Ignore measured points beyond max_range.
        if(z > max_range_mm)
          continue;

        // -- Count up z error.
        local_depth_error += fabs(1.0 - z / gtz);

        // -- Count up color error.
        size_t idx = indexmap(ppt.v_, ppt.u_);
        size_t orig_idx = cloud_indices[idx];
        int gtu = orig_idx % model.width_;
        int gtv = orig_idx / model.width_;
        uint8_t gte = edge_pcd(gtv, gtu);
        uint8_t e = edge_frame(ppt.v_, ppt.u_);
        local_color_error += fabs((double)gte - e);
        //local_color_error += fabs((double)gtc[0] - c[0]) + fabs((double)gtc[1] - c[1]) + fabs((double)gtc[2] - c[2]);
              
        ++(*count);
      }
    }
    *depth_error += local_depth_error;
    *color_error += local_color_error;
  
#ifdef TIMING
    hrt.stop(); cout << hrt.reportMilliseconds() << endl;
#endif 
  }

  void meanDepthMultiplierAndCNError(const FrameProjector& model,
                                     const Frame &frame, const Cloud& pcd,
                                     const Frame &gt,
                                     const Eigen::MatrixXf &color_names_lookup,
                                     double* depth_error, double* color_error,
                                     double* count, double max_range)
  {
    ROS_ASSERT(frame.depth_->rows() == model.height_);
    ROS_ASSERT(frame.depth_->cols() == model.width_);
    HighResTimer hrt;
  
    // -- Make the ground truth depth image.
    hrt.reset("meanDepthError: cloudToFrame"); hrt.start();
    //Frame gt;
    //model.cloudToFrame(pcd, &gt);
#ifdef TIMING
    hrt.stop(); cout << hrt.reportMilliseconds() << endl;
#endif 

    // -- Count up mean depth error.
    hrt.reset("meanDepthError: counting"); hrt.start();
    ProjectivePoint ppt;
    Point pt;
    Point gtpt;
    double max_range_mm = max_range * 1000;
    double local_depth_error = 0;
    double local_color_error = 0;

    for(ppt.u_ = 0; ppt.u_ < gt.depth_->cols(); ++ppt.u_) {
      for(ppt.v_ = 0; ppt.v_ < gt.depth_->rows(); ++ppt.v_) {
        // -- Both ground truth and measurement must have data.
        double gtz = gt.depth_->coeffRef(ppt.v_, ppt.u_);
        if(gtz == 0)
          continue;
        double z = frame.depth_->coeffRef(ppt.v_, ppt.u_);
        if(z == 0)
          continue;
      
        // -- Ignore measured points beyond max_range.
        if(z > max_range_mm)
          continue;

        // -- Count up z error.
        local_depth_error += fabs(1.0 - z / gtz);

        // -- Count up color error.
        cv::Vec3b gtc = gt.img_(ppt.v_, ppt.u_);
        cv::Vec3b c = frame.img_(ppt.v_, ppt.u_);
        size_t gtidx = gtc[2]/8 + 32*(gtc[1]/8) + 32*32*(gtc[0]/8);
        size_t idx = c[2]/8 + 32*(c[1]/8) + 32*32*(c[0]/8);
        local_color_error += (color_names_lookup.row(gtidx) - color_names_lookup.row(idx)).norm();
        //local_color_error += sqrt((gtc[0] - c[0]) * (gtc[0] - c[0]) +
        //                                (gtc[1] - c[1]) * (gtc[1] - c[1]) +
        //                                (gtc[2] - c[2]) * (gtc[2] - c[2]));
        //local_color_error += fabs((double)gtc[0] - c[0]) + fabs((double)gtc[1] - c[1]) + fabs((double)gtc[2] - c[2]);
              
        ++(*count);
      }
    }
    *depth_error += local_depth_error;
    *color_error += 255*sqrt(3/13.)*local_color_error; //So we're closer to pixels
  
#ifdef TIMING
    hrt.stop(); cout << hrt.reportMilliseconds() << endl;
#endif 
  }

}  // namespace clams

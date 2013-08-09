#include <primesense_slam/frame_aligner.h>
#include <pcl/common/distances.h>
#include <pcl/common/transformation_from_correspondences.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;
using namespace Eigen;

#define VISUALIZE

namespace clams
{

  FrameAligner::FrameAligner(const FrameProjector& model0,
                             const FrameProjector& model1) :
    view_handler_(NULL),
    params_(defaultParams()),
    model0_(model0),
    model1_(model1)
  {
  }

  bool FrameAligner::align(Frame frame0, Frame frame1, Eigen::Affine3d* f0_to_f1) const
  {
    vector<cv::Point2d> correspondences0, correspondences1;
    return wideGridSearch(frame0, frame1, correspondences0, correspondences1, f0_to_f1);
  }

  bool FrameAligner::align(Frame frame0, Frame frame1,
                           const std::vector<cv::KeyPoint>& keypoints0, const std::vector<cv::KeyPoint>& keypoints1,
                           Cloud::ConstPtr keycloud0, Cloud::ConstPtr keycloud1,
                           FeaturesConstPtr features0, FeaturesConstPtr features1,
                           bool consider_wide_search, Eigen::Affine3d* f0_to_f1) const
  {
    // -- Try to get rough transform between the two based on the corresponding keypoints.
    Affine3d guess;
    vector<cv::Point2d> correspondences0, correspondences1;
    bool found_rough_transform = computeRoughTransform(//frame0, frame1,
      keypoints0, keypoints1, 
      keycloud0, keycloud1, features0, features1, &correspondences0, &correspondences1, &guess);

    // -- Visualize the initial rough transform.
#ifdef VISUALIZE
    if(found_rough_transform && view_handler_) {
      double rx, ry, rz, tx, ty, tz;
      generateXYZYPR(guess.cast<float>(), rx, ry, rz, tx, ty, tz);
      Eigen::ArrayXd x(6); x << rx, ry, rz, tx, ty, tz;
      FrameAlignmentMDE::Ptr mde(new FrameAlignmentMDE(params_, model0_, model1_, frame0, frame1, correspondences0, correspondences1));
      view_handler_->handleGridSearchUpdate(x, mde->eval(x));
      cout << "^^^^ Objective with initial transform from feature matching." << endl;
      if(getenv("PAUSE_ON_ROUGH_TRANSFORM"))
        cv::waitKey();
    }
#endif
  
    // -- Run grid search as desired.
    if(found_rough_transform)
      return narrowGridSearch(frame0, frame1, correspondences0, correspondences1, guess, f0_to_f1);
    else if(consider_wide_search)
      return wideGridSearch(frame0, frame1, correspondences0, correspondences1, f0_to_f1);
    else
      return false;
  }

  GridSearch FrameAligner::setupGridSearch() const
  {
    GridSearch gs(6);
    string type = params_.get<string>("gridsearch_type");  
    if(type == "default") {
      gs.verbose_ = false;
      gs.view_handler_ = view_handler_;
      gs.num_scalings_ = 5;
      double max_res_rot = 1.5 * M_PI / 180.0;
      double max_res_trans = 0.1;
      gs.max_resolutions_ << max_res_rot, max_res_rot, max_res_rot, max_res_trans, max_res_trans, max_res_trans;
      int gr = 2;
      gs.grid_radii_ << gr, gr, gr, gr, gr, gr;
      double sf = 0.5;
      gs.scale_factors_ << sf, sf, sf, sf, sf, sf;
      gs.couplings_ << 0, 1, 2, 1, 0, 3;  // Search over (pitch, y) and (yaw, x) jointly.
    }
    else if(type == "coupled") {
      gs.verbose_ = false;
      gs.view_handler_ = view_handler_;
      gs.num_scalings_ = 5;
      double max_res_rot = 1.5 * M_PI / 180.0;
      double max_res_trans = 0.1;
      gs.max_resolutions_ << max_res_rot, max_res_rot, max_res_rot, max_res_trans, max_res_trans, max_res_trans;
      int gr = 1;
      gs.grid_radii_ << gr, gr, gr, gr, gr, gr;
      double sf = 0.5;
      gs.scale_factors_ << sf, sf, sf, sf, sf, sf;
      gs.couplings_ << 0, 0, 0, 0, 0, 0;
    }
  
    return gs;
  }

  bool FrameAligner::wideGridSearch(Frame frame0, Frame frame1,
                                    const std::vector<cv::Point2d>& correspondences0, const std::vector<cv::Point2d>& correspondences1,
                                    Eigen::Affine3d* f0_to_f1) const
  {
    ROS_DEBUG("FrameAligner::wideGridSearch");
    // -- Run grid search.
    ScopedTimer st("FrameAligner::align");
    FrameAlignmentMDE::Ptr mde(new FrameAlignmentMDE(params_, model0_, model1_, frame0, frame1, correspondences0, correspondences1));
    GridSearch gs = setupGridSearch();
    gs.objective_ = mde;
    ArrayXd x = gs.search(ArrayXd::Zero(6));

    // -- Print out statistics.
    cout << "============================== Frame alignment complete" << endl;
    cout << "GridSearch solution: " << x.transpose() << endl;
    cout << "Computed " << gs.num_evals_ << " evals in " << gs.time_ << " seconds." << endl;
    cout << gs.num_evals_ / gs.time_ << " evals / second." << endl;
    cout << gs.time_ / gs.num_evals_ << " seconds / eval." << endl;

    double count, depth_error;
    mde->count_ = &count;
    mde->depth_error_ = &depth_error;
    double final_objective = mde->eval(x);
  
    cout << " -- Single-number statistics" << endl;
    cout << "Final objective: " << final_objective << endl;
    cout << "Depth error: " << depth_error << endl;
    cout << "Count: " << count << endl;
    cout << "==============================" << endl;

    *f0_to_f1 = generateTransform(x(0), x(1), x(2), x(3), x(4), x(5)).cast<double>();
    return validate(count, depth_error);
  }

  bool FrameAligner::narrowGridSearch(Frame frame0, Frame frame1,
                                      const std::vector<cv::Point2d>& correspondences0, const std::vector<cv::Point2d>& correspondences1, 
                                      const Eigen::Affine3d& guess,
                                      Eigen::Affine3d* f0_to_f1) const
  {
    ROS_DEBUG("FrameAligner::narrowGridSearch");
    // -- Run grid search.
    ScopedTimer st("FrameAligner::align");
    FrameAlignmentMDE::Ptr mde(new FrameAlignmentMDE(params_, model0_, model1_, frame0, frame1, correspondences0, correspondences1));
    GridSearch gs = setupGridSearch();
    gs.objective_ = mde;
    //Convert guess to XYZRPY
    double rx, ry, rz, tx, ty, tz;
    generateXYZYPR(guess.cast<float>(), rx, ry, rz, tx, ty, tz);
    //Now search with that initialization
    Eigen::ArrayXd init(6); init << rx, ry, rz, tx, ty, tz;
    Eigen::ArrayXd x = gs.search(init);

    // -- Print out statistics.
    cout << "============================== Frame alignment complete" << endl;
    cout << "GridSearch solution: " << x.transpose() << endl;
    cout << "Computed " << gs.num_evals_ << " evals in " << gs.time_ << " seconds." << endl;
    cout << gs.num_evals_ / gs.time_ << " evals / second." << endl;
    cout << gs.time_ / gs.num_evals_ << " seconds / eval." << endl;

    double count, depth_error;
    mde->count_ = &count;
    mde->depth_error_ = &depth_error;
    double final_objective = mde->eval(x);
  
    cout << " -- Single-number statistics" << endl;
    cout << "Final objective: " << final_objective << endl;
    cout << "Depth error: " << depth_error << endl;
    cout << "Count: " << count << endl;
    cout << "==============================" << endl;

    *f0_to_f1 = generateTransform(x(0), x(1), x(2), x(3), x(4), x(5)).cast<double>();
    return validate(count, depth_error);
  }

  bool FrameAligner::validate(double count, double depth_error) const
  {
    if(count < params_.get<int>("min_depth_count") || depth_error > params_.get<double>("max_depth_error")) {
      ROS_WARN("Alignment finished but was not considered successful..");
      return false;
    }
  
    // if(final_objective > 0.04) {
    //   ROS_WARN("Alignment finished but was not considered successful..");
    //   return false;
    // }
  
    return true;
  }

  bool FrameAligner::computeRoughTransform(//Frame frame0, Frame frame1,
    const std::vector<cv::KeyPoint>& keypoints0, const std::vector<cv::KeyPoint>& keypoints1,
    Cloud::ConstPtr keycloud0, Cloud::ConstPtr keycloud1,
    FeaturesConstPtr features0, FeaturesConstPtr features1,
    std::vector<cv::Point2d>* correspondences0, std::vector<cv::Point2d>* correspondences1,
    Eigen::Affine3d* f0_to_f1) const
  {
    correspondences0->clear();
    correspondences1->clear();
  
    if(features0->rows < params_.get<int>("k") || features1->rows < params_.get<int>("k")) {
      ROS_DEBUG("Not enough features, skipping.");
      return false;
    }

    cv::FlannBasedMatcher matcher;
    vector< vector<cv::DMatch> > matches_each;
    // TODO: Why was this inside a try / catch?
    matcher.knnMatch(*features0, *features1, matches_each, params_.get<int>("k"));

    // -- Get best correspondences for use in grid search and build up flat vector of all matches
    //    that meet our criteria.
    vector<cv::DMatch> matches;
    correspondences0->reserve(matches_each.size());
    correspondences1->reserve(matches_each.size());
    double max_feature_dist = params_.get<double>("max_feature_dist");
    for(size_t j = 0; j < matches_each.size(); j++) {
      const vector<cv::DMatch>& mat = matches_each[j];
      if(mat.empty())
        continue;

      bool added_corr = false;
      for(size_t k = 0; k < mat.size(); k++) {
        //cout << mat[k].distance << " ";
        if(mat[k].queryIdx < 0 || mat[k].trainIdx < 0)
          continue;
        if(mat[k].distance > max_feature_dist)
          continue;

        // Make sure opencv is doing what we think it is.
        if(k > 0)
          ROS_ASSERT(mat[k].distance >= mat[k-1].distance);
      
        // Correspondences get filled with the best matches that are at least somewhat good.
        // They don't need depth.
        if(!added_corr) {
          correspondences0->push_back(keypoints0[mat[k].queryIdx].pt);
          correspondences1->push_back(keypoints1[mat[k].trainIdx].pt);
          added_corr = true;
        }

        // Saved matches must have depth for 3d ransac.
        /*  
            if(frame0.depth_->coeffRef(keypoints0[mat[k].queryIdx].pt.y, keypoints0[mat[k].queryIdx].pt.x) > 0 &&
            frame1.depth_->coeffRef(keypoints1[mat[k].trainIdx].pt.y, keypoints1[mat[k].trainIdx].pt.x) > 0)
            {
            matches.push_back(mat[k]);
            }
        */
        if( isFinite(keycloud0->at(mat[k].queryIdx)) && 
            isFinite(keycloud1->at(mat[k].trainIdx)) )
        {
          matches.push_back(mat[k]);
        }
      }
    }

    //Check that enough are matched
    if(matches.size() < (size_t)params_.get<int>("min_ransac_inliers")) {
      ROS_DEBUG_STREAM(matches.size() << " is not enough matches, skipping.");
      return false;
    }
// SDM Removed for now, trying to avoid loading frames
//#ifdef VISUALIZE
//  cv::Mat3b img_match;
//  cv::drawMatches(frame0.img_, keypoints0, frame1.img_, keypoints1,
//                  matches, img_match);
//  cv::imshow("match", img_match);
//  // cv::imshow("frame0", frame0.img_);
//  // cv::imshow("frame1", frame1.img_);
//  cv::waitKey(500);
//#endif

    // TODO: Don't need to project the whole frame here, just keypoints.
    //Cloud::Ptr cloud0(new Cloud);
    //Cloud::Ptr cloud1(new Cloud);
    //model0_.frameToCloud(frame0, cloud0.get());
    //model1_.frameToCloud(frame1, cloud1.get());

    //From here the logic is more or less copied from orb_matcher
    vector<Eigen::Affine3f> candidates;
    int num_ransac_samples = params_.get<int>("num_ransac_samples");
    double min_pairwise_keypoint_dist = params_.get<double>("min_pairwise_keypoint_dist");
    double ransac_max_inlier_dist = params_.get<double>("ransac_max_inlier_dist");
    for(int j = 0; j < num_ransac_samples; ++j)
    {
      //Sample 3 points in ref img and precompute their distances to each other
      const cv::DMatch &match0 = matches[rand() % matches.size()];
      const cv::DMatch &match1 = matches[rand() % matches.size()];
      const cv::DMatch &match2 = matches[rand() % matches.size()];
      int idx0 = match0.queryIdx;
      int idx1 = match1.queryIdx;
      int idx2 = match2.queryIdx;
      //pcl::PointXYZRGB r0 = cloud0->at(keypoints0[idx0].pt.x, keypoints0[idx0].pt.y);
      //pcl::PointXYZRGB r1 = cloud0->at(keypoints0[idx1].pt.x, keypoints0[idx1].pt.y);
      //pcl::PointXYZRGB r2 = cloud0->at(keypoints0[idx2].pt.x, keypoints0[idx2].pt.y);
      pcl::PointXYZRGB r0 = keycloud0->at(idx0);
      pcl::PointXYZRGB r1 = keycloud0->at(idx1);
      pcl::PointXYZRGB r2 = keycloud0->at(idx2);
      ROS_ASSERT(isFinite(r0) && isFinite(r1) && isFinite(r2));
      double d01 = pcl::euclideanDistance(r0, r1);
      double d02 = pcl::euclideanDistance(r0, r2);
      double d12 = pcl::euclideanDistance(r1, r2);
      if(d01 < min_pairwise_keypoint_dist ||
         d02 < min_pairwise_keypoint_dist ||
         d12 < min_pairwise_keypoint_dist)
      {
        //ROS_DEBUG_STREAM("Skipping sampled correspondences because inter-point distance is too small.");
        continue;
      }
      //Get their corresponding matches
      int oidx0 = match0.trainIdx;
      int oidx1 = match1.trainIdx;
      int oidx2 = match2.trainIdx;
      //pcl::PointXYZRGB t0 = cloud1->at(keypoints1[oidx0].pt.x, keypoints1[oidx0].pt.y);
      //pcl::PointXYZRGB t1 = cloud1->at(keypoints1[oidx1].pt.x, keypoints1[oidx1].pt.y);
      //pcl::PointXYZRGB t2 = cloud1->at(keypoints1[oidx2].pt.x, keypoints1[oidx2].pt.y);
      pcl::PointXYZRGB t0 = keycloud1->at(oidx0);
      pcl::PointXYZRGB t1 = keycloud1->at(oidx1);
      pcl::PointXYZRGB t2 = keycloud1->at(oidx2);
      ROS_ASSERT(isFinite(t0) && isFinite(t1) && isFinite(t2));
      //If the matches are too distant from themselves w.r.t the distance in the current frame, continue
      if(fabs(pcl::euclideanDistance(t0, t1) - d01) > ransac_max_inlier_dist ||
         fabs(pcl::euclideanDistance(t0, t2) - d02) > ransac_max_inlier_dist ||
         fabs(pcl::euclideanDistance(t1, t2) - d12) > ransac_max_inlier_dist)
      {
        //ROS_DEBUG_STREAM("Skipping sampled correspondences because inter-point distances are too different between the two frames.");
        continue;
      }
      //Otherwise generate transformation from correspondence
      pcl::TransformationFromCorrespondences tfc;
      tfc.add(t0.getVector3fMap(), r0.getVector3fMap());
      tfc.add(t1.getVector3fMap(), r1.getVector3fMap());
      tfc.add(t2.getVector3fMap(), r2.getVector3fMap());
    
      Eigen::Affine3f trans = tfc.getTransformation();
      if((trans * t0.getVector3fMap() - r0.getVector3fMap()).norm() > 0.01 ||
         (trans * t1.getVector3fMap() - r1.getVector3fMap()).norm() > 0.01 ||
         (trans * t2.getVector3fMap() - r2.getVector3fMap()).norm() > 0.01)
        continue;
      candidates.push_back(trans);
    }
    cout << "Have " << candidates.size() << " candidates after RANSAC, before inlier check" << endl;
    //Do inlier check
    // SDM for now leaving this in to filter out infinite points
    Cloud::Ptr keypoint_cloud0(new Cloud);
    for(size_t j = 0; j < keypoints0.size(); ++j) {
      Point pt = keycloud0->at(j);
      if(isFinite(pt))
        keypoint_cloud0->push_back(pt);
    }
    Cloud::Ptr keypoint_cloud1(new Cloud);
    for(size_t j = 0; j < keypoints1.size(); ++j) {
      Point pt = keycloud1->at(j);
      if(isFinite(pt))
        keypoint_cloud1->push_back(pt);
    }
  
    pcl::KdTreeFLANN<Point> kdtree0;
    kdtree0.setInputCloud(keypoint_cloud0);
  
    Cloud transformed_keypoint_cloud1;
    vector<int> indices;
    vector<float> distances;
    Eigen::Affine3f best_transform;
    float best_distance = std::numeric_limits<float>::infinity();
    int best_num_inliers = 0;
    bool has_best = false;
    vector< std::pair<size_t, size_t> > best_inliers;
    for(size_t j = 0; j < candidates.size(); ++j)
    {
      float inlier_distance = 0;
      //Ensure that, in at least 2 directions, there's variance here
      float minx=std::numeric_limits<float>::infinity();
      float maxx=-std::numeric_limits<float>::infinity();
      float miny=std::numeric_limits<float>::infinity();
      float maxy=-std::numeric_limits<float>::infinity();
      float minz=std::numeric_limits<float>::infinity();
      float maxz=-std::numeric_limits<float>::infinity();
      pcl::transformPointCloud(*keypoint_cloud1, transformed_keypoint_cloud1, candidates[j]);
      int num_inliers = 0;
      vector< std::pair<size_t, size_t> > inliers;
      for(size_t k = 0; k < transformed_keypoint_cloud1.size(); ++k)
      {
        indices.clear();
        distances.clear();
        kdtree0.nearestKSearch(transformed_keypoint_cloud1.points[k], 1, indices, distances);
        if(distances.size() != 0 && distances[0] < ransac_max_inlier_dist)
        {
          ++num_inliers;
          inlier_distance += distances[0];
          inliers.push_back(std::pair<size_t, size_t>(k, indices[0]));
          const pcl::PointXYZRGB& kpt = transformed_keypoint_cloud1.points[k];
          //Update bounding volume
          if(kpt.x < minx) minx = kpt.x;
          if(kpt.x > maxx) maxx = kpt.x;
          if(kpt.y < miny) miny = kpt.y;
          if(kpt.y > maxy) maxy = kpt.y;
          if(kpt.z < minz) minz = kpt.z;
          if(kpt.z > maxz) maxz = kpt.z;
        }
      }
      //Check number of inliers
      if(num_inliers < params_.get<double>("min_ransac_inlier_percent") * keypoint_cloud0->size()) {
        //ROS_DEBUG_STREAM("Inlier percent " << (double)num_inliers / keypoint_cloud0->size() << " is too low.");
        continue;
      }
      if(num_inliers < params_.get<int>("min_ransac_inliers")) {
        //ROS_DEBUG_STREAM(num_inliers << " is too few inliers.");
        continue;
      }

      // -- Do PCA check.
      Vector3d mean = Vector3d::Zero();
      for(size_t i = 0; i < transformed_keypoint_cloud1.size(); ++i)
        mean += transformed_keypoint_cloud1[i].getVector3fMap().cast<double>();
      mean /= (double)transformed_keypoint_cloud1.size();
      Matrix3d xxt = Matrix3d::Zero();
      for(size_t i = 0; i < transformed_keypoint_cloud1.size(); ++i) {
        Vector3d pt = transformed_keypoint_cloud1[i].getVector3fMap().cast<double>() - mean;
        xxt += pt * pt.transpose();
      }
      xxt /= (double)transformed_keypoint_cloud1.size();
      Eigen::JacobiSVD<Matrix3d> svd(xxt, Eigen::ComputeFullU | Eigen::ComputeFullV);
      Matrix3d U = svd.matrixU();
      bool passed_pca = true;
      // Only check for variation in the first two principal components.
      // It's ok to have lots of points defining a plane; the problem is when they're all on
      // a line.
      for(int i = 0; i < U.cols() - 1; ++i) {
        Vector3d vec = U.col(i);
        double maxval = -numeric_limits<double>::max();
        double minval = numeric_limits<double>::max();
        for(size_t k = 0; k < transformed_keypoint_cloud1.size(); ++k) {
          Vector3d pt = transformed_keypoint_cloud1[k].getVector3fMap().cast<double>() - mean;
          double val = vec.dot(pt);
          maxval = max(val, maxval);
          minval = min(val, minval);
        }
        if(maxval - minval < params_.get<double>("min_bounding_length")) {
          // ROS_WARN_STREAM("Rejecting this hypothesized transform due to PCA check.");
          // ROS_WARN_STREAM("Extremal values of " << minval << " to " << maxval);
          // ROS_WARN_STREAM("Distance of " << maxval - minval << " along principal component " << i << ", " << vec.transpose());
          // ROS_WARN_STREAM("Num pts: " << transformed_keypoint_cloud1.size());
          passed_pca = false;
        }
        //ROS_DEBUG_STREAM("Distance of " << maxval - minval << " along principal component " << i << ", " << vec.transpose());
      }
      if(!passed_pca) {
        //ROS_DEBUG_STREAM("PCA test failed.");
        continue;
      }
         
      //Check bounding volume
      // int has_x = (maxx - minx) > min_bounding_length_;
      // int has_y = (maxy - miny) > min_bounding_length_;
      // int has_z = (maxz - minz) > min_bounding_length_;
      // if(has_x + has_y + has_z < 2)
      // {
      //   continue;
      // }
    
      // Consider this transform
      // inlier_distance /= num_inliers; //Average out
      // if(inlier_distance < best_distance)
      // {
      if(num_inliers > best_num_inliers) {
        best_transform = candidates[j];
        best_distance = inlier_distance;
        best_num_inliers = num_inliers;
        best_inliers = inliers;
        has_best = true;

        cout << "Best so far: " << endl;
        cout << "  Num inliers: " << num_inliers << endl;
        cout << "  Has bounding size of x: " << maxx - minx << ", y: " << maxy - miny << ", z: " << maxz - minz << endl;
        cout << "  f1_to_f0: " << endl << best_transform.matrix() << endl;
      }
    }
    if(has_best)
    {
      // Re-compute the transform using only the inliers.
      pcl::TransformationFromCorrespondences tfc;
      for(size_t j = 0; j < best_inliers.size(); j++)
        tfc.add(keypoint_cloud1->points[best_inliers[j].first].getVector3fMap(), 
                keypoint_cloud0->points[best_inliers[j].second].getVector3fMap());
      Affine3d f1_to_f0 = tfc.getTransformation().cast<double>();
      *f0_to_f1 = f1_to_f0.inverse();
      cout << "Final f1_to_f0: " << endl;
      cout << f1_to_f0.matrix() << endl;
      return true;
    }
    else
      return false;
  }


  FrameAlignmentVisualizer::FrameAlignmentVisualizer(FrameProjector model0, FrameProjector model1) : 
    Agent(),
    model0_(model0),
    model1_(model1),
    cloud0_(new Cloud),
    cloud1_(new Cloud),
    vis_("FrameAlignmentVisualizer"),
    needs_update_(false),
    foo_(false)
  {
    vis_.registerKeyboardCallback(&FrameAlignmentVisualizer::keyboardCallback, *this);
  
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

  void FrameAlignmentVisualizer::handleGridSearchUpdate(const Eigen::ArrayXd& x, double objective)
  {
    scopeLockWrite;
    foo_ = false;
    f0_to_f1_ = generateTransform(x(0), x(1), x(2), x(3), x(4), x(5));
    needs_update_ = true;
    cout << "Grid search improvement: " << objective << " at x = " << x.transpose() << endl;
  }

  void FrameAlignmentVisualizer::_run()
  {
    while(true) {
      { scopeLockRead; if(quitting_) break; }
        
      usleep(1e3);
      scopeLockWrite;
      foo_ = true;
      if(needs_update_) {
        vis_.removeAllPointClouds();
        Cloud::Ptr pcd(new Cloud);
        pcl::transformPointCloud(*cloud0_, *pcd, f0_to_f1_);
        *pcd += *cloud1_;
        if(!vis_.updatePointCloud(pcd, "default"))
          vis_.addPointCloud(pcd, "default");
        needs_update_ = false;
      }
      vis_.spinOnce(2);
      ROS_ASSERT(foo_);
    }
  }

  void FrameAlignmentVisualizer::setFrames(Frame frame0, Frame frame1)
  {
    scopeLockWrite;
    model0_.frameToCloud(frame0, cloud0_.get());
    model1_.frameToCloud(frame1, cloud1_.get());
  }

  void FrameAlignmentVisualizer::keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie)
  {
    if(event.keyDown()) {
      cout << "Pressed " << (int)event.getKeyCode() << endl;

      if(event.getKeyCode() == 'd' || event.getKeyCode() == 27) {
        cout << "Quitting FrameAlignmentVisualizer" << endl;
        quitting_ = true;
      }
    }
  }

}  // namespace clams

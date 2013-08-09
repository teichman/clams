#ifndef FRAME_ALIGNER_H
#define FRAME_ALIGNER_H

#include <ros/package.h>
#include <bag_of_tricks/agent.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/features2d/features2d.hpp>
#include <primesense_slam/params.h>
#include <primesense_slam/mean_depth_error.h>

namespace clams
{

  class FrameAlignmentVisualizer : public GridSearchViewHandler, public Agent
  {
  public:
    FrameAlignmentVisualizer(FrameProjector model0, FrameProjector model1);
    void setFrames(Frame frame0, Frame frame1);
    void handleGridSearchUpdate(const Eigen::ArrayXd& x, double objective);
    //! You must run() in the main thread for the visualizer to work.
    void _run();
  
  protected:
    FrameProjector model0_;
    FrameProjector model1_;
    Cloud::Ptr cloud0_;
    Cloud::Ptr cloud1_;
    Eigen::Affine3f f0_to_f1_;
    pcl::visualization::PCLVisualizer vis_;
    bool needs_update_;
    bool foo_;

    void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie);
  };

  class FrameAligner
  {
  public:
    typedef boost::shared_ptr<cv::Mat1f> FeaturesPtr;
    typedef boost::shared_ptr<const cv::Mat1f> FeaturesConstPtr;

    GridSearchViewHandler* view_handler_;

    Params params_;
    static inline Params defaultParams()
    {
      Params params;
      params.load(ros::package::getPath("primesense_slam") + "/data/default_frame_alignment_params.txt");
      return params;
    }
  
    FrameAligner(const FrameProjector& model0, const FrameProjector& model1);
  
    //! Computes transform that takes points in 0 to points in 1. Tries using feature matching to get close, and, failing that, does
    //! a wide area grid search if you tell it to.
    //! keypoints don't necessarily correspond.  Correspondences will be computed.
    //! Returns false if no transform found.
    bool align(Frame frame0, Frame frame1,
               const std::vector<cv::KeyPoint>& keypoints0, const std::vector<cv::KeyPoint>& keypoints1,
               Cloud::ConstPtr keycloud0, Cloud::ConstPtr keycloud1,
               FeaturesConstPtr features0, FeaturesConstPtr features1,
               bool consider_wide_search, Eigen::Affine3d* f0_to_f1) const;
  
    //! Computes transform that takes points in 0 to points in 1.  Starts with identity transform, does a wide search.
    //! Returns false if no transform found.
    bool align(Frame frame0, Frame frame1, Eigen::Affine3d* f0_to_f1) const;

    //! Using keypoint matching method only to compute transform that takes points in 0 to points in 1.
    //! correspondences will be filled with whatever can be found.
    //! Returns false if no transform found.
    bool computeRoughTransform( // Frame frame0, Frame frame1, TODO REMOVE
      const std::vector<cv::KeyPoint>& keypoints0, const std::vector<cv::KeyPoint>& keypoints1,
      Cloud::ConstPtr keycloud0, Cloud::ConstPtr keycloud1,
      FeaturesConstPtr features0, FeaturesConstPtr features1,
      std::vector<cv::Point2d>* correspondences0, std::vector<cv::Point2d>* correspondences1,
      Eigen::Affine3d* f0_to_f1) const;
    //! Computes transform that takes points in 0 to points in 1.  Starts with guess, has a narrow search.
    //! Returns false if no transform found.
    bool narrowGridSearch(Frame frame0, Frame frame1,
                          const std::vector<cv::Point2d>& correspondences0, const std::vector<cv::Point2d>& correspondences1, 
                          const Eigen::Affine3d& guess,
                          Eigen::Affine3d* f0_to_f1) const;
                 
  protected:
    FrameProjector model0_;
    FrameProjector model1_;

    //! Computes transform that takes points in 0 to points in 1. Starts with identity transform, has a wide search.
    //! Returns false if no transform found.
    bool wideGridSearch(Frame frame0, Frame frame1,
                        const std::vector<cv::Point2d>& correspondences0, const std::vector<cv::Point2d>& correspondences1,
                        Eigen::Affine3d* f0_to_f1) const;
  

    bool validate(double count, double depth_error) const;
    GridSearch setupGridSearch() const;
  };

}

#endif // FRAME_ALIGNER_H

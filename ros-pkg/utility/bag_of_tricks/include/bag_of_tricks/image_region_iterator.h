#ifndef IMAGE_REGION_ITERATOR_H
#define IMAGE_REGION_ITERATOR_H

#include <algorithm>
#include <opencv2/core/core.hpp>

class ImageRegionIterator
{
public:
  //! @param size The image size.
  //! @param radius Only iterate over points within this radius.
  ImageRegionIterator(cv::Size size, int radius);
  void setCenter(cv::Point2i center);
  void setCenter(int idx);
  ImageRegionIterator& operator++();
  const cv::Point2i& operator*() const { return pt_; }
  const cv::Point2i* operator->() const { return &pt_; }
  bool done() const { return done_; }
  int index() const { return pt_.y * size_.width + pt_.x; }
    
protected:
  cv::Size size_;
  cv::Point2i center_;
  int radius_;
  double radius_squared_;
  int d_;
  int min_x_;
  int max_x_;
  int min_y_;
  int max_y_;
  cv::Point2i pt_;
  double dx_;
  double dy_;
  bool done_;
};

#endif // IMAGE_REGION_ITERATOR_H

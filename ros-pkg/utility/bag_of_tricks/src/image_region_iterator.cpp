#include <bag_of_tricks/image_region_iterator.h>

using namespace std;


ImageRegionIterator::ImageRegionIterator(cv::Size size, int radius) :
  size_(size),
  center_(cv::Point2i(-1, -1)),
  radius_(radius),
  radius_squared_(radius_*radius_),
  done_(false)
{
}

void ImageRegionIterator::setCenter(cv::Point2i center) {
  center_ = center;
  done_ = false;

  min_x_ = max(0, center_.x - radius_);
  max_x_ = min(size_.width - 1, center_.x + radius_);
  min_y_ = max(0, center_.y - radius_);
  max_y_ = min(size_.height - 1, center_.y + radius_);
  pt_ = cv::Point2i(min_x_, min_y_);
  dx_ = pt_.x - center_.x;
  dy_ = pt_.y - center_.y;
  if(dx_*dx_ + dy_*dy_ > radius_squared_)
    ++(*this);
}

void ImageRegionIterator::setCenter(int idx)
{
  int y = idx / size_.width;
  int x = idx - y * size_.width;
  cv::Point2i center(x, y);
  setCenter(center);
}

ImageRegionIterator& ImageRegionIterator::operator++()
{
  assert(center_.x >= 0 && center_.y >= 0);
    
  if(done_)
    return *this;

  while(true) {
    ++pt_.x;
    if(pt_.x > max_x_) { 
      pt_.x = min_x_;
      ++pt_.y;
    }
      
    if(pt_.y > max_y_) { 
      done_ = true;
      break;
    }

    dx_ = pt_.x - center_.x;
    dy_ = pt_.y - center_.y;
    if(dx_*dx_ + dy_*dy_ <= radius_squared_)
      break;
  }

  return *this;
}


#include <clams/trajectory.h>

using namespace std;
using namespace Eigen;

Trajectory::Trajectory(const Trajectory& other)
{
  *this = other;
}

Trajectory& Trajectory::operator=(const Trajectory& other)
{
  resize(other.size());
  for(size_t i = 0; i < transforms_.size(); ++i)
    if(other.exists(i))
      transforms_[i] = new Affine3d(other.get(i));

  return *this;
}

Trajectory::~Trajectory()
{
  clear();
}

void Trajectory::clear()
{
  for(size_t i = 0; i < transforms_.size(); ++i)
    if(transforms_[i])
      delete transforms_[i];
  transforms_.clear();
}

void Trajectory::resize(size_t num)
{
  clear();
  transforms_.resize(num, NULL);
}

void Trajectory::set(size_t idx, const Eigen::Affine3d& transform)
{
  if(transforms_[idx])
    *transforms_[idx] = transform;
  else
    transforms_[idx] = new Affine3d(transform);
}

const Eigen::Affine3d& Trajectory::get(size_t idx) const
{
  if(idx > transforms_.size()) {
    ROS_FATAL_STREAM("Tried to get transform " << idx << " / " << transforms_.size());
    abort();
  }
  return *transforms_[idx];
}

bool Trajectory::exists(size_t idx) const
{
  return transforms_[idx];
}

void Trajectory::remove(size_t idx)
{
  if(transforms_[idx])
    delete transforms_[idx];

  transforms_[idx] = NULL;
}

void Trajectory::serialize(std::ostream& out) const
{
  eigen_extensions::serializeScalar(transforms_.size(), out);
  for(size_t i = 0; i < transforms_.size(); ++i) {
    bool ex = transforms_[i];
    eigen_extensions::serializeScalar(ex, out);
    if(ex)
      eigen_extensions::serialize(transforms_[i]->matrix(), out);
  }
}

void Trajectory::deserialize(std::istream& in)
{
  size_t num_transforms;
  eigen_extensions::deserializeScalar(in, &num_transforms);
  resize(num_transforms);
  
  for(size_t i = 0; i < transforms_.size(); ++i) {
    bool ex;
    eigen_extensions::deserializeScalar(in, &ex);
    if(ex) {
      Matrix4d tmp;
      eigen_extensions::deserialize(in, &tmp);
      set(i, Affine3d(tmp));
    }
  }
}

size_t Trajectory::numValid() const
{
  size_t num = 0;
  for(size_t i = 0; i < transforms_.size(); ++i)
    if(transforms_[i])
      ++num;
  return num;
}

std::string Trajectory::status(const std::string& prefix) const
{
  ostringstream oss;
  oss << prefix << "Num valid: " << numValid() << " / " << size() << endl;
  // for(size_t i = 0; i < transforms_.size(); ++i) {
  //   if(transforms_[i]) {
  //     oss << prefix << "Transform " << i << endl;
  //     for(int j = 0; j < 4; ++j)
  //         oss << prefix << "  " << transforms_[i]->matrix().row(j) << endl;
  //   }
  // }

  return oss.str();
}

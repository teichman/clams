#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <ros/assert.h>
#include <serializable/serializable.h>
#include <eigen_extensions/eigen_extensions.h>

namespace clams
{
  
  class Trajectory : public Serializable
  {
  public:
    Trajectory() {}
    //! Deep copy
    Trajectory(const Trajectory& other);
    //! Deep copy
    Trajectory& operator=(const Trajectory& other);
    ~Trajectory();

    void clear();
    //! Destroys everything inside.
    void resize(size_t num);
    void set(size_t idx, const Eigen::Affine3d& transform);
    const Eigen::Affine3d& get(size_t idx) const;
    bool exists(size_t idx) const;
    void remove(size_t idx);
    size_t size() const { return transforms_.size(); }
    size_t numValid() const;
    std::string status(const std::string& prefix) const;
  
    void serialize(std::ostream& out) const;
    void deserialize(std::istream& in);
  
  protected:
    std::vector<Eigen::Affine3d*> transforms_;
  };

}

#endif // TRAJECTORY_H

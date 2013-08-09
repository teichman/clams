#ifndef STREAM_SEQUENCE_BASE_H
#define STREAM_SEQUENCE_BASE_H

#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <pcl/io/pcd_io.h>
#include <timer/timer.h>
#include <stream_sequence/frame_projector.h>

namespace clams
{

  class StreamSequenceBase
  {
  public:
    typedef boost::shared_ptr<StreamSequenceBase> Ptr;
    typedef boost::shared_ptr<const StreamSequenceBase> ConstPtr;
  
    std::vector<double> timestamps_;
    FrameProjector proj_;
    
    StreamSequenceBase ():
      cache_size_(0)
    {}

    virtual ~StreamSequenceBase () {}

    static StreamSequenceBase::Ptr initializeFromDirectory (const std::string &dir);

    void load(const std::string &root_path);

    inline std::string getRootPath() const
    { return root_path_; };

    virtual size_t size() const = 0;

    void readFrame(size_t idx, Frame* frame) const;
    //! Returns the nearest frame, no matter how far away it is in time.  Check dt to find out.
    size_t readFrame(double timestamp, double* dt, Frame* frame) const;

    //! dt is signed.
    size_t seek(double timestamp, double* dt) const;

    clams::Cloud::ConstPtr operator[] (size_t idx) const;

    clams::Cloud::ConstPtr at (size_t idx) const;

    inline void
    setCacheSize (size_t cache_size) const
    { cache_size_ = cache_size; };
    
    //! Adds dt to all timestamps.  Does not save.
    void applyTimeOffset(double dt);
    //! Inefficient accessors that conceal how the projection is done.
    //! These shouldn't be used if it can be avoided.
    clams::Cloud::Ptr getCloud(size_t idx) const;
    clams::Cloud::Ptr getCloud(double timestamp, double* dt) const;
    cv::Mat3b getImage(size_t idx) const;
    cv::Mat3b getImage(double timestamp, double* dt) const;
    
  protected:
    friend class StreamSequenceAccessor;

    virtual void readFrameImpl (size_t idx, Frame* frame) const = 0;
    virtual void loadImpl(const std::string& root_path) = 0;

    void
    addCloudToCache (size_t idx, clams::Cloud::ConstPtr cloud) const;

    mutable std::deque<size_t> frames_cache_;
    mutable std::map<size_t, clams::Cloud::ConstPtr> pcds_cache_;
    mutable size_t cache_size_;
    std::string root_path_;
    bool undistort_;
  };

  //! Accessor which has "push back" functionality, remapping indices appropriately...don't ask
  class StreamSequenceAccessor : public Serializable
  {
  public:
    typedef boost::shared_ptr<StreamSequenceAccessor> Ptr;
    typedef boost::shared_ptr<const StreamSequenceAccessor> ConstPtr;

    StreamSequenceAccessor (StreamSequenceBase::ConstPtr sseq)
      : sseq_ (sseq)
    {}

    inline void
    turnOnFrameNumber (size_t idx)
    {
      frames_.push_back (idx);
    }

    inline clams::Cloud::ConstPtr operator[] (size_t idx) const
    {
      return ( (*sseq_)[frames_[idx]]);
    }

    inline clams::Cloud::ConstPtr at (size_t idx) const
    {
      ROS_ASSERT (idx < frames_.size ());
      return (operator[] (idx));
    }
    
    inline void serialize (std::ostream& out) const
    {
      out << sseq_->getRootPath () << std::endl;
      size_t sz = frames_.size ();
      out.write ((char*)&sz, sizeof (size_t));
      out.write ((char*)&frames_[0], sz * sizeof (size_t));
      out.write ((char*)&sseq_->undistort_, sizeof(bool));
    }

    inline void deserialize (std::istream& in)
    {
      std::string root_path;
      in >> root_path;
      sseq_ = StreamSequenceBase::initializeFromDirectory (root_path);
      size_t sz;
      in.read((char*)&sz, sizeof(size_t));
      frames_.resize (sz);
      in.read((char*)&frames_[0], sz * sizeof (size_t));
      in.read((char*)&sseq_->undistort_, sizeof(bool));
    }

    inline void setCacheSize (size_t cache_size) const
    { sseq_->setCacheSize (cache_size); }

    inline size_t size () const
    { return frames_.size (); }

  protected:
    StreamSequenceBase::ConstPtr sseq_;
    std::vector<size_t> frames_;
  };


  inline bool isFinite(const Point& pt)
  {
    return (pcl_isfinite(pt.x) && pcl_isfinite(pt.y) && pcl_isfinite(pt.z));
  }

}

#endif // STREAM_SEQUENCE_BASE_H




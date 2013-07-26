#ifndef AGENT_H
#define AGENT_H

#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/thread.hpp>
#include <bag_of_tricks/lockable.h>

typedef boost::shared_ptr<boost::thread> ThreadPtr;

class Agent : public SharedLockable
{
public:
  Agent() : SharedLockable(), quitting_(false), running_(false) {}
  virtual ~Agent() {}

  //! TODO: should probably be 'stop'.
  void quit() { scopeLockWrite; quitting_ = true; }
  bool running() { scopeLockRead; return running_; }
  void run() { running_ = true; _run(); running_ = false; }
  virtual void _run() = 0;
  ThreadPtr launch() { return ThreadPtr(new boost::thread(boost::bind(&Agent::run, this))); }
  
protected:
  bool quitting_;
  bool running_;
};

#endif // AGENT_H

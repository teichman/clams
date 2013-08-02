#ifndef LOCKABLE_H
#define LOCKABLE_H

#include <pthread.h>
#include <errno.h>
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/locks.hpp>

#define scopeLockRead boost::shared_lock<boost::shared_mutex> lockable_shared_lock(shared_mutex_)
#define scopeLockWrite boost::unique_lock<boost::shared_mutex> lockable_unique_lock(shared_mutex_)

/* SharedLockable is based on the uncopyable boost::shared_mutex.
   This presents a dilemma when assigning or copy constructing.
   Right now, the state of the mutex in the other SharedLockable
   does not get copied to the target SharedLockable.
   I'm not sure yet if this is the desired behavior.
*/
class SharedLockable
{
public:
  SharedLockable() {}
  //! Copy constructor will make a new shared_mutex that is unlocked.
  SharedLockable(const SharedLockable& other) {}
  //! Assignment operator will *not* copy the shared_mutex_ or the state of shared_mutex_ from other.
  SharedLockable& operator=(const SharedLockable& other) { return *this; }

  void lockWrite();
  void unlockWrite();
  bool trylockWrite();
  
  void lockRead();
  void unlockRead();
  bool trylockRead();

protected:
  //! For the first time ever, I'm tempted to make this mutable.
  //! It'd make user methods still be able to be const even if they are locking.
  boost::shared_mutex shared_mutex_;
};

class Lockable
{
public:
  pthread_mutex_t mutex_;
    
  Lockable();
  void lock();
  void unlock();
  bool trylock();
};

#endif // LOCKABLE_H

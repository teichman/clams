#include <bag_of_tricks/lockable.h>

Lockable::Lockable() :
  mutex_(pthread_mutex_t())
{
}

void Lockable::lock()
{
  pthread_mutex_lock(&mutex_);
}

void Lockable::unlock()
{
  pthread_mutex_unlock(&mutex_);
}

bool Lockable::trylock()
{
  if(pthread_mutex_trylock(&mutex_) == EBUSY)
    return false;
  else
    return true;
}


SharedLockable::SharedLockable()
{
}

void SharedLockable::lockWrite()
{
  shared_mutex_.lock();
}

void SharedLockable::unlockWrite()
{
  shared_mutex_.unlock();
}

bool SharedLockable::trylockWrite()
{
  return shared_mutex_.try_lock();
}

void SharedLockable::lockRead()
{
  shared_mutex_.lock_shared();
}

void SharedLockable::unlockRead()
{
  shared_mutex_.unlock_shared();
}

bool SharedLockable::trylockRead()
{
  return shared_mutex_.try_lock_shared();
}


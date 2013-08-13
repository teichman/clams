#ifndef SERIALIZABLE_H
#define SERIALIZABLE_H

#include <iostream>
#include <fstream>
#include <assert.h>
#include <vector>
#include <map>


/** \brief @b Serializable is an abstract base class which represents
 * objects that can be serialized and deserialized.
 *
 * See test_serializable.cpp for example usage.
 */
class Serializable
{
public:
  virtual ~Serializable() {};
  virtual void serialize(std::ostream& out) const = 0;
  virtual void deserialize(std::istream& in) = 0;
  virtual void save(const std::string& path) const;
  virtual void load(const std::string& path);
};

std::ostream& operator<<(std::ostream& out, const Serializable& ser);
std::istream& operator>>(std::istream& in, Serializable& ser);


/* Class for making it easy to use stream constructors.
 * For example, assume OnlineLearner doesn't have a default constructor,
 * but does have a OnlineLearner(istream& in) constructor.
 * Then to load a new one from disk, you can do this:
 * OnlineLearner learner((IfstreamWrapper(path)));
 * Note the extra parens.  They are unfortunately necessary due to the
 * possibility that you are declaring a function "learner" that
 * takes an IfstreamWrapper called path and returns an OnlineLearner.
 * Really, C++?  Why?  It's almost perfect.  Alas.
 */
class IfstreamWrapper
{
public:
  IfstreamWrapper(const std::string& path);
  ~IfstreamWrapper();
  operator std::istream&() { return ifstream_; }
  
private:
  std::ifstream ifstream_;
};

#endif // SERIALIZABLE_H

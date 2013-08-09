#ifndef PARAMS_H
#define PARAMS_H

#include <map>
#include <boost/any.hpp>
#include <serializable/serializable.h>

namespace clams
{


#define ABORT(x)                             \
  do {                                          \
    std::cerr << "\033[1;31m[PIPELINE] " << x   \
              << "\033[0m" << std::endl;        \
    abort();                                    \
  } while(0)

#define ASSERT(cond)                                 \
  do {                                                  \
    if(!(cond)) {                                       \
      ABORT("ASSERTION FAILED" << std::endl          \
               << "\tfile = " << __FILE__ << std::endl  \
               << "\tline = " << __LINE__ << std::endl  \
               << "\tcond = " << #cond);                \
    }                                                   \
  } while(0)

  inline bool isInvalidChar(char c)
  {
    // TODO: Are the underscore and colon ok?
    //return (!isalnum(c) && c != '_' && c != '<' && c != '>' && c != ':');
    return (!isalnum(c) && c != '_' && c != ':');  
  }

  inline bool isValidName(const std::string &name)
  {
    if(name.size() == 0)
      return false;
    
    return (find_if(name.begin(), name.end(), isInvalidChar) == name.end());
  }

  //! Asserts that this name won't screw something up in serialization.
  inline void assertValidName(const std::string& name)
  {
    if(!isValidName(name))
      ABORT("Name \"" << name << "\" is invalid.  See isValidName().");
  }
  
  class Params : public Serializable, public YAMLizable
  {
  public:
    std::map<std::string, boost::any> storage_;

    Params() {}
    ~Params() {}
    
    bool exists(const std::string& name) const;
    size_t size() const { return storage_.size(); }
    bool empty() const { return storage_.empty(); }
    bool operator==(const Params& other) const;
    bool operator<(const Params& other) const;

    void serialize(std::ostream& out) const;
    void deserialize(std::istream& in);
    YAML::Node YAMLize() const;
    void deYAMLize(const YAML::Node& in);
    
    template<typename T> bool isType(const std::string& name) const
    {
      if(storage_.count(name) == 0) { 
        ABORT("Tried to check type of non-existent param \"" << name << "\".");
      }
            
      bool is_T = true;
      try {
        boost::any_cast<T>(storage_.find(name)->second);
      }
      catch(boost::bad_any_cast& e) {
        is_T = false;
      }

      return is_T;
    }
    
    template<typename T> void set(const std::string& name, const T& val)
    {
      assertValidName(name);
      if(storage_.count(name) != 0 && !isType<T>(name))
        ABORT("Tried to change type of param \"" << name << "\".");
      
      storage_[name] = val;
    }
    
    template<typename T> T get(const std::string& name) const
    {
      assertValidName(name);
      if(storage_.count(name) == 0) { 
        ABORT("Params does not have a member named \"" << name << "\"");
      }

      try {
        boost::any_cast<T>(storage_.find(name)->second);
      }
      catch(boost::bad_any_cast& e) {
        ABORT("Bad type when getting param \"" << name << "\"");
      }
      return boost::any_cast<T>(storage_.find(name)->second);
    }
  };
}
  
/* YAML encoder / decoder for Params.
 * Limited type support.
 */
namespace YAML {
  template<>
  struct convert<clams::Params> {
    static Node encode(const clams::Params& params) {
      return params.YAMLize();
    }
      
    static bool decode(const Node& node, clams::Params& params) {
      params.storage_.clear();
      if(!node.IsNull())
        params.deYAMLize(node);
      
      return true;
    }
  };
}

#endif // PARAMS_H

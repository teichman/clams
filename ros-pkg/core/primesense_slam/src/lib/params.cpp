#include <primesense_slam/params.h>

using namespace std;
using boost::any;
using boost::any_cast;

namespace clams
{
  
  void Params::serialize(std::ostream& out) const
  {
    out << "Params (" << storage_.size() << ")" << endl;
    map<string, any>::const_iterator it;
    for(it = storage_.begin(); it != storage_.end(); ++it) {
      string name = it->first;
      any a = it->second;
      bool success = false;
      try {
        any_cast<string>(a);
        out << "string " << name << " " << any_cast<string>(a) << endl;
        success = true;
      }
      catch(boost::bad_any_cast& e) {}
      try {
        any_cast<int>(a);
        out << "int " << name << " " << any_cast<int>(a) << endl;
        success = true;
      }
      catch(boost::bad_any_cast& e) {}
      try {
        any_cast<double>(a);
        out << "double " << name << " " << any_cast<double>(a) << endl;
        success = true;
      }
      catch(boost::bad_any_cast& e) {}
      try {
        any_cast<bool>(a);
        out << "bool " << name << " " << any_cast<bool>(a) << endl;
        success = true;
      }
      catch(boost::bad_any_cast& e) {}

      if(!success) { 
        ABORT("Tried to serialize Param field \"" << name << "\", but Params does not know how to serialize its data type.");
      }
    }
  }

  void Params::deserialize(std::istream& in)
  {
    string buf;
    in >> buf;
    ASSERT(buf.compare("Params") == 0);
    in >> buf;
    size_t num_params = atoi(buf.substr(1, buf.size() - 1).c_str());
    getline(in, buf);

    for(size_t i = 0; i < num_params; ++i) {
      string type;
      string name;
      in >> type;
      in >> name;
      
      if(type.compare("string") == 0) {
        string val;
        in >> val;
        set<string>(name, val);
      }
      else if(type.compare("int") == 0) {
        int val;
        in >> val;
        set<int>(name, val);
      }
      else if(type.compare("double") == 0) {
        double val;
        in >> val;
        set<double>(name, val);
      }
      else if(type.compare("bool") == 0) {
        bool val;
        in >> val;
        set<bool>(name, val);
      }
      else {
        ABORT("Deserialization failed: type \"" << type << "\" not recognized.");
      }
      getline(in, buf);
    }
            
  }

  YAML::Node Params::YAMLize() const
  {
    YAML::Node node;
    std::map<std::string, boost::any>::const_iterator it;
    for(it = storage_.begin(); it != storage_.end(); ++it) {
      std::string name = it->first;
      boost::any any = it->second;
      bool success = false;
      try {
        std::string val = boost::any_cast<std::string>(any);
        node[name] = val;
        success = true;
      }
      catch(...) {
      }

      if(!success) { 
        try {
          double val = boost::any_cast<double>(any);
          node[name] = val;
          success = true;
        }
        catch(...) {
        }
      }

      if(!success) { 
        try {
          bool val = boost::any_cast<bool>(any);
          if(val)
            node[name] = "True";
          else
            node[name] = "False";
          success = true;
        }
        catch(...) {
        }
      }
        
      if(!success)
        ABORT("YAML format only supports params of string, double, or bool type.  Offending param: \"" << name << "\".");

    }
    
    return node;
  }
  
  void Params::deYAMLize(const YAML::Node& in)
  {
    storage_.clear();

    for(YAML::const_iterator it = in.begin(); it != in.end(); ++it) {
      std::string name = it->first.as<std::string>();
      bool success = false;
      try {
        double val = it->second.as<double>();
        set(name, val);
        success = true;
      }
      catch(...) {
      }

      if(!success) {
        try {
          std::string val = it->second.as<std::string>();
          if(val == "True")
            set<bool>(name, true);
          else if(val == "False")
            set<bool>(name, false);
          else
            set(name, val);
          success = true;
        }
        catch(...) {
        }
      }

      if(!success)
        ABORT("YAML format only supports params of string, double, or bool type.  Offending param: \"" << name << "\".");
        
    }
  }
  
  bool Params::exists(const std::string& name) const
  {
    return (storage_.count(name) != 0);
  }

  bool Params::operator==(const Params& other) const
  {
    std::map<std::string, boost::any>::const_iterator it;
    for(it = storage_.begin(); it != storage_.end(); ++it) {
      string name = it->first;
      if(other.storage_.find(name) == other.storage_.end())
        ABORT("Tried to compare two Params objects with different fields.");

      if(isType<double>(name)) {
        if(get<double>(name) != other.get<double>(name))
          return false;
      }
      else if(isType<string>(name)) {
        if(get<string>(name) != other.get<string>(name))
          return false;
      }
      else if(isType<int>(name)) {
        if(get<int>(name) != other.get<int>(name))
          return false;
      }
      else if(isType<bool>(name)) {
        if(get<bool>(name) != other.get<bool>(name))
          return false;
      }
      else
        ABORT("Unknown Param type.");
    }
    return true;
  }

  bool Params::operator<(const Params& other) const
  {
    std::map<std::string, boost::any>::const_iterator it;
    for(it = storage_.begin(); it != storage_.end(); ++it) {
      string name = it->first;
      if(other.storage_.find(name) == other.storage_.end())
        ABORT("Tried to compare two Params objects with different fields.");

      if(isType<double>(name)) {
        if(get<double>(name) < other.get<double>(name))
          return true;
        if(get<double>(name) > other.get<double>(name))
          return false;
      }
      else if(isType<string>(name)) {
        if(get<string>(name) < other.get<string>(name))
          return true;
        if(get<string>(name) > other.get<string>(name))
          return false;
      }
      else if(isType<int>(name)) {
        if(get<int>(name) < other.get<int>(name))
          return true;
        if(get<int>(name) > other.get<int>(name))
          return false;
      }
      else if(isType<bool>(name)) {
        if((int)get<bool>(name) < (int)other.get<bool>(name))
          return true;
        if((int)get<bool>(name) > (int)other.get<bool>(name))
          return false;
      }
      else
        ABORT("Unknown Param type.");
    }
    return false;
  }
  
} // namespace pl

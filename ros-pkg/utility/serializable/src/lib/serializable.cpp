#include <serializable/serializable.h>

using namespace std;

void Serializable::load(const std::string& path)
{
  ifstream f;
  f.open(path.c_str());
  if(!f.is_open()) {
    cerr << "Failed to open " << path << endl;
    assert(f.is_open());
  }
  deserialize(f);
  f.close();
}

void Serializable::save(const std::string& path) const
{
  ofstream f;
  f.open(path.c_str());
  if(!f.is_open()) {
    cerr << "Failed to open " << path << endl;
    assert(f.is_open());
  }
  serialize(f);
  f.close();
}

std::ostream& operator<<(std::ostream& out, const Serializable& ser)
{
  ser.serialize(out);
  return out;
}

std::istream& operator>>(std::istream& in, Serializable& ser)
{
  ser.deserialize(in);
  return in;
}

void YAMLizable::loadYAML(const std::string& path)
{
  YAML::Node doc = YAML::LoadFile(path);
  deYAMLize(doc);
}

void YAMLizable::saveYAML(const std::string& path) const
{
  ofstream f;
  f.open(path.c_str());
  if(!f.is_open()) {
    cerr << "Failed to open " << path << endl;
    assert(f.is_open());
  }

  YAML::Node doc = YAMLize();
  f << YAML::Dump(doc) << endl;
  f.close();
}

IfstreamWrapper::IfstreamWrapper(const std::string& path)
{
  cout << "Constructing IfstreamWrapper" << endl;
  
  ifstream_.open(path.c_str());
  if(!ifstream_.is_open()) {
    cerr << "Failed to open " << path << endl;
  }
  assert(ifstream_.is_open());
}

IfstreamWrapper::~IfstreamWrapper()
{
  cout << "Destroying IfstreamWrapper." << endl;
  
  if(ifstream_.is_open())
    ifstream_.close();
}

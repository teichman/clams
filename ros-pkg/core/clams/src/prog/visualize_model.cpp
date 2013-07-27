#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <clams/slam_calibrator.h>

using namespace std;
using namespace Eigen;
using namespace clams;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  namespace bfs = boost::filesystem;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  string model_path;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("intrinsics", bpo::value(&model_path)->required(), "Distortion model.")
    ;

  p.add("intrinsics", 1);

  bpo::variables_map opts;
  bool badargs = false;
  try {
    bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
    bpo::notify(opts);
  }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << bfs::basename(argv[0]) << " MODEL" << endl;
    cout << "  Saves output visualizations to MODEL-visualization/" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  DiscreteDepthDistortionModel model;
  model.load(model_path);
  string vis_dir = model_path + "-visualization";
  model.visualize(vis_dir);
  cout << "Saved visualization of distortion model to " << vis_dir << endl;
    
  return 0;
}

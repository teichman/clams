#include <clams/trajectory_visualizer.h>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

using namespace std;
using namespace Eigen;
using namespace clams;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  namespace bfs = boost::filesystem;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  string sseq_path;
  string traj_path;
  string map_path;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("sseq", bpo::value(&sseq_path)->required(), "StreamSequence, i.e. asus data.")
    ("traj", bpo::value(&traj_path)->required(), "Trajectory from slam in CLAMS format.")
    ("map", bpo::value(&map_path)->required(), "Cached map.")
    ;

  p.add("sseq", 1);
  p.add("traj", 1);
  p.add("map", 1);
  
  bpo::variables_map opts;
  bool badargs = false;
  try {
    bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
    bpo::notify(opts);
  }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << bfs::basename(argv[0]) << " [OPTS] SSEQ TRAJ MAP" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  Trajectory traj;
  traj.load(traj_path);
  StreamSequenceBase::ConstPtr sseq = StreamSequenceBase::initializeFromDirectory(sseq_path);
  Cloud::Ptr map(new Cloud);
  pcl::io::loadPCDFile<pcl::PointXYZRGB>(map_path, *map);

  TrajectoryVisualizer tv(sseq, traj, map, sseq_path);
  tv.run();
  
  return 0;
}

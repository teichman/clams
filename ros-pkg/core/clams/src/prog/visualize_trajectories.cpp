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

  string workspace;
  double resolution;
  double max_range;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("workspace", bpo::value(&workspace)->default_value("."), "CLAMS workspace.")
    ("resolution", bpo::value(&resolution)->default_value(0.02), "Resolution of the voxel grid used for display, in meters.")
    ("max-range", bpo::value(&max_range)->default_value(MAX_RANGE_MAP), "Maximum range to use when building the map from the given trajectory, in meters.")
    ;

  p.add("workspace", 1);
  
  bpo::variables_map opts;
  bool badargs = false;
  try {
    bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
    bpo::notify(opts);
  }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << bfs::basename(argv[0]) << " [OPTS] CLAMS_WORKSPACE" << endl;
    cout << "  This program will visualize all SLAM results in a given CLAMS workspace." << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  // -- Check for existence of CLAMS_WORKSPACE/slam_results.
  string sequences_path = workspace + "/sequences";
  string results_path = workspace + "/slam_results";
  ROS_ASSERT(bfs::exists(results_path));

  // -- Get names of sequences that have corresponding results.
  vector<string> sseq_names;
  bfs::directory_iterator it(results_path), eod;
  BOOST_FOREACH(const bfs::path& p, make_pair(it, eod)) {
    string path = results_path + "/" + p.leaf().string() + "/trajectory";
    if(bfs::exists(path))
      sseq_names.push_back(p.leaf().string());
  }
  sort(sseq_names.begin(), sseq_names.end());

  // -- Display each trajectory.
  for(size_t i = 0; i < sseq_names.size(); ++i) { 
    string sseq_path = sequences_path + "/" + sseq_names[i];
    string traj_path = results_path + "/" + sseq_names[i] + "/trajectory";
    string map_path = results_path + "/" + sseq_names[i] + "/map.pcd";
    StreamSequenceBase::ConstPtr sseq = StreamSequenceBase::initializeFromDirectory(sseq_path);
    Trajectory traj;
    traj.load(traj_path);

    TrajectoryVisualizer tv(sseq, traj);
    tv.max_range_ = max_range;
    tv.vgsize_ = resolution;

    cout << "==========" << endl;
    cout << "== Displaying " << traj_path << endl;
    cout << "==========" << endl;
    cout << "Press ESC to advance to next map." << endl;

    tv.setTitle(sseq_names[i]);
    tv.run();
    usleep(1e5);
  }

  return 0;
}

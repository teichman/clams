#include <clams/slam_calibration_visualizer.h>
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
  double resolution;
  double max_range;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("sseq", bpo::value(&sseq_path)->required(), "StreamSequence, i.e. asus data.")
    ("traj", bpo::value(&traj_path)->required(), "Trajectory from slam in CLAMS format.")
    ("resolution", bpo::value(&resolution)->default_value(0.02), "Resolution of the voxel grid used for display, in meters.")
    ("max-range", bpo::value(&max_range)->default_value(MAX_RANGE_MAP), "Maximum range to use when building the map from the given trajectory, in meters.")
    ;

  p.add("sseq", 1);
  p.add("traj", 1);
  
  bpo::variables_map opts;
  bool badargs = false;
  try {
    bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
    bpo::notify(opts);
  }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << bfs::basename(argv[0]) << " [OPTS] SSEQ TRAJ" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  Trajectory traj;
  traj.load(traj_path);

  StreamSequenceBase::ConstPtr sseq = StreamSequenceBase::initializeFromDirectory(sseq_path);
  
  SlamCalibrator::Ptr calibrator(new SlamCalibrator(sseq->proj_, max_range, resolution));
  cout << "Using " << calibrator->max_range_ << " for max range." << endl;
  cout << "Using " << calibrator->vgsize_ << " for voxel grid size." << endl;

  calibrator->trajectories_.push_back(traj);
  calibrator->sseqs_.push_back(sseq);
  SlamCalibrationVisualizer vis(calibrator);
  vis.run();

  return 0;
}

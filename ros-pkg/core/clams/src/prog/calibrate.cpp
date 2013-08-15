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

  string workspace;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("workspace", bpo::value(&workspace)->default_value("."), "CLAMS workspace.")
    ("increment", bpo::value<int>()->default_value(1), "Use every kth frame for calibration.")
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
    cout << "Usage: " << bfs::basename(argv[0]) << " [ OPTS ] CLAMS_WORKSPACE " << endl;
    cout << "  This program will calibrate using all slam results in CLAMS_WORKSPACE/." << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  // -- Check for existence of CLAMS_WORKSPACE/slam_results.
  string sequences_path = workspace + "/sequences";
  string results_path = workspace + "/slam_results";
  if(!bfs::exists(results_path)) {
    cout << "Expected results path \"" << results_path << "\" does not exist." << endl;
    cout << "Are you running this program from within a CLAMS workspace?" << endl;
    cout << "Have you run \"rosrun clams slam\" yet?" << endl;
    return 0;
  }

  // -- Get names of sequences that have corresponding results.
  vector<string> sseq_names;
  bfs::directory_iterator it(results_path), eod;
  BOOST_FOREACH(const bfs::path& p, make_pair(it, eod)) {
    string path = results_path + "/" + p.leaf().string() + "/trajectory";
    if(bfs::exists(path))
      sseq_names.push_back(p.leaf().string());
  }
  sort(sseq_names.begin(), sseq_names.end());

  // -- Construct sseqs with corresponding trajectories.
  vector<StreamSequenceBase::ConstPtr> sseqs;
  vector<Trajectory> trajs;
  vector<Cloud::ConstPtr> maps;
  for(size_t i = 0; i < sseq_names.size(); ++i) { 
    string sseq_path = sequences_path + "/" + sseq_names[i];
    string traj_path = results_path + "/" + sseq_names[i] + "/trajectory";
    string map_path = results_path + "/" + sseq_names[i] + "/calibration_map.pcd";

    cout << "Sequence " << i << endl;
    cout << "  StreamSequence:" << sseq_path << endl;
    cout << "  Trajectory: " << traj_path << endl;
    cout << "  Map: " << map_path << endl;

    sseqs.push_back(StreamSequenceBase::initializeFromDirectory(sseq_path));

    Trajectory traj;
    traj.load(traj_path);
    trajs.push_back(traj);

    Cloud::Ptr map(new Cloud);
    pcl::io::loadPCDFile<pcl::PointXYZRGB>(map_path, *map);
    maps.push_back(map);
  }

  // -- Run the calibrator.
  SlamCalibrator::Ptr calibrator(new SlamCalibrator(sseqs[0]->proj_));

  cout << endl;
  if(opts.count("increment")) {
    calibrator->increment_ = opts["increment"].as<int>();
    cout << "Using frame increment of " << calibrator->increment_ << endl;
  }
  cout << endl;
  
  calibrator->trajectories_ = trajs;
  calibrator->sseqs_ = sseqs;
  calibrator->maps_ = maps;
  
  DiscreteDepthDistortionModel model = calibrator->calibrate();
  string output_path = workspace + "/distortion_model";
  model.save(output_path);
  cout << "Saved distortion model to " << output_path << endl;

  string vis_dir = output_path + "-visualization";
  model.visualize(vis_dir);
  cout << "Saved visualization of distortion model to " << vis_dir << endl;
    
  return 0;
}

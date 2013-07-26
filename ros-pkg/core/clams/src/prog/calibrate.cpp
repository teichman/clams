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

  //vector<string> paths;
  vector<string> sequence_paths;
  vector<string> trajectory_paths;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("sseqs", bpo::value(&sequence_paths)->required()->multitoken(), "StreamSequences, i.e. asus data.")
    ("trajs", bpo::value(&trajectory_paths)->required()->multitoken(), "Trajectories from slam.")
    //    ("paths", bpo::value< vector<string> >(&paths)->required()->multitoken(), "Recorded logs and trajectories")
    ;


  //p.add("paths", -1);

    bpo::variables_map opts;
  bool badargs = false;
  try {
    bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
    bpo::notify(opts);
  }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << bfs::basename(argv[0]) << " OPTS --sseqs SSEQ [SSEQ ...] --trajs TRAJ [TRAJ ...]" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  // vector<string> sequence_paths;
  // vector<string> trajectory_paths;
  // for(size_t i = 0; i < paths.size(); ++i) {
  //   if(i % 2 == 0)
  //     sequence_paths.push_back(paths[i]);
  //   else
  //     trajectory_paths.push_back(paths[i]);
  // }

  ROS_ASSERT(sequence_paths.size() == trajectory_paths.size());
  cout << "Using sequence / trajectory pairs: " << endl;
  vector<Trajectory> trajectories(trajectory_paths.size());
  vector<StreamSequenceBase::ConstPtr> sseqs;
  ROS_ASSERT(sequence_paths.size() == trajectory_paths.size());
  for(size_t i = 0; i < sequence_paths.size(); ++i) {
    cout << "=== Log " << i << endl;
    cout << " Sequence: " << sequence_paths[i] << endl;
    cout << " Trajectory: " << trajectory_paths[i] << endl;

    trajectories[i].load(trajectory_paths[i]);
    cout << trajectories[i].status("  ");
    StreamSequenceBase::Ptr sseq = StreamSequenceBase::initializeFromDirectory(sequence_paths[i]);
    sseqs.push_back(sseq);
  }

  SlamCalibrator::Ptr calibrator(new SlamCalibrator(sseqs[0]->proj_));
  cout << "Using " << calibrator->max_range_ << " as max range." << endl;
  calibrator->trajectories_ = trajectories;
  calibrator->sseqs_ = sseqs;
  
  DiscreteDepthDistortionModel model = calibrator->calibrate();
  string output_path = "distortion_model";
  model.save(output_path);
  cout << "Saved distortion model to " << output_path << endl;

  string vis_dir = output_path + "-visualization";
  model.visualize(vis_dir);
  cout << "Saved visualization of distortion model to " << vis_dir << endl;
    
  return 0;
}

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <primesense_slam/primesense_slam.h>

using namespace std;
using namespace g2o;
using namespace clams;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  namespace bfs = boost::filesystem;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  string sseq_path;
  string output_dir;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("sseq", bpo::value(&sseq_path)->required(), "StreamSequence, i.e. asus data.")
    ("output-dir", bpo::value(&output_dir)->required(), "Directory to put output files in.")
    ("max-loopclosures", bpo::value<int>())
    ;

  p.add("sseq", 1);
  p.add("output-dir", 1);
  
  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << bfs::basename(argv[0]) << " SSEQ OPCD OTRAJ OGRAPH [OPTS]" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  cout << "Using " << sseq_path << endl;
  cout << "Saving output in " << output_dir << endl;
  ROS_ASSERT(!bfs::exists(output_dir));
  StreamSequenceBase::Ptr sseq = StreamSequenceBase::initializeFromDirectory(sseq_path);

  PrimeSenseSlam pss;
  if(opts.count("max-loopclosures")) {
    pss.max_loopclosures_ = opts["max-loopclosures"].as<int>();
    cout << "Using " << pss.max_loopclosures_ << " loop closures per frame maximum." << endl;
  }
  pss.sseq_ = sseq;

  // -- Run slam.
  ThreadPtr slamthread = pss.launch();
  slamthread->join();
  pss.pgs_->save(output_dir + "/graph");
  
  // -- Find the largest subgraph.
  vector< pair<size_t, size_t> > index(pss.trajs_.size());
  for(size_t i = 0; i < pss.trajs_.size(); i++)
    index.push_back(pair<size_t, size_t>(pss.trajs_[i].numValid(), i));
  sort(index.begin(), index.end(), std::greater< pair<size_t, size_t> >());  // Descending sort.

  // -- Save outputs in order of size.
  if(!bfs::exists(output_dir))
    boost::filesystem::create_directory(output_dir);
  
  for(size_t i = 0; i < index.size(); ++i) {
    size_t idx = index[i].first;
    ostringstream oss;
    oss << output_dir << "/map_" << i << ".pcd";
    pcl::io::savePCDFileBinary(oss.str(), *pss.maps_[idx]);
  }

  for(size_t i = 0; i < index.size(); ++i) {
    size_t idx = index[i].first;
    ostringstream oss;
    oss << output_dir << "/traj_" << i << ".traj";
    pss.trajs_[idx].save(oss.str());
  }
  
  return 0;
}

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

  string workspace;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("workspace", bpo::value(&workspace)->required(), "CLAMS workspace.")
    ("max-loopclosures", bpo::value<int>())
    ;

  p.add("workspace", 1);
  
  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << bfs::basename(argv[0]) << " [OPTS] CLAMS_WORKSPACE" << endl;
    cout << "  This program will run slam on all sequences in CLAMS_WORKSPACE/sequences/." << endl;
    cout << "  Results will be placed in CLAMS_WORKSPACE/slam_results/." << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  string sequences_path = workspace + "/sequences";
  string results_path = workspace + "/slam_results";
  if(!bfs::exists(results_path))
    bfs::create_directory(results_path);

  // -- Get sequence names in order.
  vector<string> sseq_names;
  bfs::directory_iterator it(sequences_path), eod;
  BOOST_FOREACH(const bfs::path& p, make_pair(it, eod)) {
    string sseq_path = sequences_path + "/" + p.leaf().string();
    if(bfs::is_directory(sseq_path))
      sseq_names.push_back(p.leaf().string());
  }
  sort(sseq_names.begin(), sseq_names.end());
                                              
  for(size_t i = 0; i < sseq_names.size(); ++i) { 
    string sseq_path = sequences_path + "/" + sseq_names[i];
    string output_dir = results_path + "/" + sseq_names[i];
  
    cout << endl;
    cout << "Running slam on sequence in " << sseq_path << endl;
    cout << "Saving output in " << output_dir << endl;
    
    if(!bfs::exists(output_dir))
      boost::filesystem::create_directory(output_dir);
    
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
    vector< pair<size_t, size_t> > index;
    for(size_t i = 0; i < pss.trajs_.size(); i++)
      index.push_back(pair<size_t, size_t>(pss.trajs_[i].numValid(), i));
    sort(index.begin(), index.end(), std::greater< pair<size_t, size_t> >());  // Descending sort.
    
    // -- Save outputs in order of size.  
    for(size_t i = 0; i < index.size(); ++i) {
      size_t idx = index[i].second;
      ostringstream oss;
      oss << output_dir << "/traj_" << i << ".traj";
      pss.trajs_[idx].save(oss.str());
    }
    
    for(size_t i = 0; i < index.size(); ++i) {
      size_t idx = index[i].second;
      ostringstream oss;
      oss << output_dir << "/map_" << i << ".pcd";
      pcl::io::savePCDFileBinary(oss.str(), *pss.maps_[idx]);
    }
  }
  
  return 0;
}

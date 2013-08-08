#include <stream_sequence/stream_sequence_base.h>
#include <iomanip>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>


using namespace std;
namespace bfs = boost::filesystem;
using namespace clams;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  string src;
  string dst;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("src", bpo::value(&src)->required(), "StreamSequence, i.e. asus data")
    ("dst", bpo::value(&dst)->required(), "Output path for Freiburg-format dataset")
    ;

  p.add("src", 1);
  p.add("dst", 1);

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS] SRC DST" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  ROS_ASSERT(!bfs::exists(dst));
  StreamSequenceBase::Ptr sseq = StreamSequenceBase::initializeFromDirectory(src);
  bfs::create_directory(dst);
  bfs::create_directory(dst + "/depth");
  bfs::create_directory(dst + "/rgb");
  Frame frame;
  ofstream assoc;
  assoc.open((dst + "/assoc.txt").c_str());
  for(size_t i = 0; i < sseq->size(); ++i) {
    cout << i << " / " << sseq->size() << endl;
    sseq->readFrame(i, &frame);
    ros::Time ts(sseq->timestamps_[i]);
    
    ostringstream ossrgb;
    ossrgb << dst << "/rgb/" << ts << ".png";
    cv::imwrite(ossrgb.str(), frame.img_);

    cv::Mat_<ushort> depth(cv::Size(frame.depth_->cols(), frame.depth_->rows()), 0);
    // Freiburg data is 5 units / mm, ours is 1 / mm.
    for(int y = 0; y < depth.rows; ++y)
      for(int x = 0; x < depth.cols; ++x)
        depth(y, x) = frame.depth_->coeffRef(y, x) * 5;  

    ostringstream ossd;
    ossd << dst << "/depth/" << ts << ".png";
    cv::imwrite(ossd.str(), depth);

    assoc << ts << " rgb/" << ts << ".png "
          << ts << " depth/" << ts << ".png" << endl;
  }

  assoc.close();

  return 0;
}

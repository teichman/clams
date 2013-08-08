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
    ostringstream ossrgb;
    ossrgb << dst << "/rgb/"
        << setiosflags(ios::fixed) << setprecision(6) << setfill('0') << setw(16)
        << sseq->timestamps_[i] << ".png";
    cv::imwrite(ossrgb.str(), frame.img_);

    cv::Mat_<ushort> depth(cv::Size(frame.depth_->cols(), frame.depth_->rows()), 0);
    for(int y = 0; y < depth.rows; ++y) {
      for(int x = 0; x < depth.cols; ++x) {
        depth(y, x) = frame.depth_->coeffRef(y, x);
      }
    }

    ostringstream ossd;
    ossd << dst << "/depth/"
        << setiosflags(ios::fixed) << setprecision(6) << setfill('0') << setw(16)
        << sseq->timestamps_[i] << ".png";
    cv::imwrite(ossd.str(), depth);
    cout << "Saved to " << ossd.str() << endl;

    // There must be a better way.
    assoc << setiosflags(ios::fixed) << setprecision(6) << setfill('0') << setw(16)
          << sseq->timestamps_[i] << " rgb/"
          << setiosflags(ios::fixed) << setprecision(6) << setfill('0') << setw(16)
          << sseq->timestamps_[i] << ".png "
          << setiosflags(ios::fixed) << setprecision(6) << setfill('0') << setw(16)
          << sseq->timestamps_[i] << " depth/"
          << setiosflags(ios::fixed) << setprecision(6) << setfill('0') << setw(16)
          << sseq->timestamps_[i] << ".png" << endl;
  }

  assoc.close();

  return 0;
}

#include <bag_of_tricks/high_res_timer.h>
#include <bag_of_tricks/bag_of_tricks.h>
#include <gtest/gtest.h>

using namespace std;
using boost::shared_ptr;

TEST(HighResTimer, HighResTimer)
{
  timespec t;
  t.tv_sec = 0;
  t.tv_nsec = 345098;
  
  HighResTimer hrt;
  hrt.start();
  nanosleep(&t, NULL);
  hrt.stop();
  cout << hrt.report() << endl;
}

TEST(ScopedTimer, ScopedTimer)
{
  HighResTimer hrt;
  timespec t;
  t.tv_sec = 0;
  t.tv_nsec = 1e6;

  {
    ScopedTimer st;
    hrt.start();
    nanosleep(&t, NULL);
  }
  hrt.stop();    
  cout << hrt.report() << endl;
}

TEST(HighResTimer, AllocationTime)
{
  int num = 640 * 480;
  int reps = 100;
  HighResTimer hrt;
  hrt.start();
  for(int i = 0; i < reps; ++i) {
    double data[num];
    data[13] = 1.13;  // Make sure that the compiler doesn't optimize this out?
  }
  hrt.stop();
  cout << "Time to allocate " << num << " doubles on the stack: " << hrt.getMicroseconds() / (double)reps << " us." << endl;

  hrt.reset();
  hrt.start();
  vector<double*> vd(reps);
  for(int i = 0; i < reps; ++i) {
    vd[i] = new double[num];
    vd[i][13] = 1.14;
  }
  hrt.stop();
  cout << "Time to allocate " << num << " doubles on the heap: " << hrt.getMicroseconds() / (double)reps << " us." << endl;

  for(int i = 0; i < reps; ++i)
    delete[] vd[i];
}

template<typename S, typename T>
void accessConstDictionary(const Dictionary<S, T>& nm)
{
  cout << "The value of \"bar\" is: " << nm["bar"] << endl;
}

TEST(Dictionary, Dictionary)
{
  Dictionary<string, int>* dptr = new Dictionary<string, int>;
  //dptr->leak_.resize(1e6);
  (*dptr)["foo"] = 1;
  map<string, int>* bptr = dptr;
  (*bptr)["bar"] = 2;
  accessConstDictionary(*dptr);
  cout << "The value of \"bar\" is: " << (*dptr)["bar"] << endl;
  
  dptr->save("dict");
  Dictionary<string, int> dict;
  dict.load("dict");
  cout << dict << endl;
  EXPECT_TRUE(dict["foo"] == (*dptr)["foo"]);
  EXPECT_TRUE(dict["bar"] == (*dptr)["bar"]);
                
  // This test only applied when Dictionary was just a map.  Now it's also Serializable.
  // 
  // This would be a memory leak if Dictionary had members that map did not.
  // valgrind --leak-check=full --manx-stackframe=2457616 bin/test_hrt confirms this is fine.
  //delete bptr;
  delete dptr;
}

void foo(const std::vector< boost::shared_ptr< const Dictionary<string, int> > >& dicts)
{
  cout << "There are: " << dicts.size() << endl;
}

void foo(const std::vector< boost::shared_ptr<Serializable> >& dicts)
{
  cout << "There are: " << dicts.size() << endl;
}

TEST(Cast, Cast)
{
  vector< shared_ptr< Dictionary<string, int> > > dicts(3);
  vector< shared_ptr<Serializable> > sers(3);
  foo(cast< const Dictionary<string, int> >(dicts));
  foo(cast<Serializable>(dicts));
  vector< shared_ptr< Dictionary<string, int> > > baz = cast< Dictionary<string, int> >(sers);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

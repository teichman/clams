#include <bag_of_tricks/lockable.h>
#include <gtest/gtest.h>

using namespace std;

class Foo : public SharedLockable
{
public:
  Foo() : SharedLockable() {}
  void deadlock() { scopeLockWrite; lockWrite(); }
  int val() { scopeLockRead; return val_; }
  void setVal(int val) { scopeLockWrite; val_ = val; }
  
protected:
  int val_;
};

// TEST(Lockable, foo)
// {
//   Foo foo;
//   cout << "Trying to deadlock..." << endl;
//   foo.deadlock();
//   cout << "Did not deadlock." << endl;
//   EXPECT_TRUE(false);
// }

TEST(Lockable, Copy)
{
  Foo foo;
  foo.setVal(37);
  foo.lockWrite();
  Foo bar = foo;
  bar.lockWrite();  // This should not deadlock; copying the SharedLockable does *not* copy the lock state.
  Foo baz;
  baz.setVal(12);
  baz = bar;

  EXPECT_TRUE(baz.val() == 37);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

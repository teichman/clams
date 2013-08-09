#include <gtest/gtest.h>
#include <pose_graph_slam/pose_graph_slam.h>

using namespace std;
using namespace g2o;

TEST(PoseGraphSlam, SingleLink)
{
  PoseGraphSlam pgs(2);
  Matrix6d covariance = Matrix6d::Identity() * 1e-6;
  
  Eigen::Vector3d axis((double)rand() / RAND_MAX, (double)rand() / RAND_MAX, (double)rand() / RAND_MAX);
  axis.normalize();
  double angle = (double)rand() / RAND_MAX;
  Eigen::Vector3d translation((double)rand() / RAND_MAX, (double)rand() / RAND_MAX, (double)rand() / RAND_MAX);
  Affine3d rotation(AngleAxis<double>(angle, axis));
  EXPECT_TRUE(fabs((rotation * Eigen::Vector4d(1, 0, 0, 1)).head(3).norm() - 1) < 1e-6);
  
  Affine3d transform = Translation3d(translation) * rotation;
  pgs.addEdge(0, 1, transform, covariance);
  pgs.solve();

  cout << pgs.transform(0).matrix() << endl << endl;
  cout << transform.matrix() << endl << endl;
  cout << pgs.transform(1).matrix() << endl << endl;
  cout << pgs.transform(1).matrix().inverse() << endl << endl;
  EXPECT_TRUE((pgs.transform(1).matrix() - transform.matrix()).norm() < 1e-3);
}

TEST(PoseGraphSlam, MultiLinkOdometryOnly)
{
  PoseGraphSlam pgs(4);
  Matrix6d covariance = Matrix6d::Identity();

  pgs.addEdge(0, 1,
              (Affine3d)Translation3d(Eigen::Vector3d(0, 1, 0)),
              covariance);
  pgs.addEdge(1, 2,
              Translation3d(Eigen::Vector3d(0, 1, 0)) * Affine3d(AngleAxis<double>(M_PI / 4, Eigen::Vector3d(1, 0, 0))),
              covariance);
  pgs.addEdge(2, 3,
              Translation3d(Eigen::Vector3d(0, 1, 0)) * Affine3d(AngleAxis<double>(M_PI / 4, Eigen::Vector3d(1, 0, 0))),
              covariance);  
  pgs.solve();

  Eigen::Vector3d translation;
  Quaterniond quat;
  for(size_t i = 0; i < pgs.numNodes(); ++i) {
    cout << "--------------------" << endl;
    pgs.vertexData(i, &translation, &quat);
    cout << "Translation: " << translation.transpose() << endl;
    cout << "Quaternion: " << quat.vec().transpose() << endl;
    cout << "Quaternion (w, x, y, z): " << quat.w() << " " << quat.x() << " " << quat.y() << " " << quat.z() << endl;
    cout << "Quaternion in matrix form:" << endl << quat.toRotationMatrix() << endl;
    cout << "Transform " << i << endl;
    cout << pgs.transform(i).matrix() << endl;
  }

  EXPECT_TRUE((pgs.transform(0).matrix() - Matrix4d::Identity()).norm() < 1e-6);

  // For each pose, if the sensor sees a point 1 unit ahead (along y), make sure it appears
  // where we expect it to in global coordinates.
  Eigen::Vector4d transformed;
  Eigen::Vector4d expected;
  transformed = pgs.transform(1) * Eigen::Vector4d(0, 1, 0, 1);
  expected = Eigen::Vector4d(0, 2, 0, 1);
  cout << "1: " << transformed.transpose() << ", expected " << expected.transpose() << endl;
  EXPECT_TRUE((transformed - expected).norm() < 1e-3);

  transformed = pgs.transform(2) * Eigen::Vector4d(0, 1, 0, 1);
  expected = Eigen::Vector4d(0, 2.70711, .70711, 1);
  cout << "2: " << transformed.transpose() << ", expected " << expected.transpose() << endl;
  EXPECT_TRUE((transformed - expected).norm() < 1e-3);

  transformed = pgs.transform(3) * Eigen::Vector4d(0, 1, 0, 1);
  expected = Eigen::Vector4d(0, 2.70711, 1.70711, 1);
  cout << "3: " << transformed.transpose() << ", expected " << expected.transpose() << endl;
  EXPECT_TRUE((transformed - expected).norm() < 1e-3);

  Affine3d transform2to0 = (Translation3d(Eigen::Vector3d(0, 1, 0))
                            * Translation3d(Eigen::Vector3d(0, 1, 0)) * AngleAxis<double>(M_PI / 4, Eigen::Vector3d(1, 0, 0)));
  cout << transform2to0.matrix() << endl << endl;
  cout << pgs.transform(2).matrix() << endl << endl;
  EXPECT_TRUE((pgs.transform(2).matrix() - transform2to0.matrix()).norm() < 1e-3);

  Affine3d transform3to0 = (Translation3d(Eigen::Vector3d(0, 1, 0))
                            * Translation3d(Eigen::Vector3d(0, 1, 0)) * AngleAxis<double>(M_PI / 4, Eigen::Vector3d(1, 0, 0))
                            * Translation3d(Eigen::Vector3d(0, 1, 0)) * AngleAxis<double>(M_PI / 4, Eigen::Vector3d(1, 0, 0)));
  cout << transform3to0.matrix() << endl << endl;
  cout << pgs.transform(3).matrix() << endl << endl;
  EXPECT_TRUE((pgs.transform(3).matrix() - transform3to0.matrix()).norm() < 1e-3);
}

TEST(PoseGraphSlam, Covariances)
{
  PoseGraphSlam pgs(4);
  // Very high confidence all around.
  Matrix6d cov0 = Matrix6d::Identity() * 1e-3;  
  // x translation can slide, everything else wants to be fixed.
  Matrix6d cov1 = Matrix6d::Identity() * 1e-3;
  cov1(0, 0) = 1;

  /*
   *
   *  ---- x
   *  |
   *  |
   *  y
   *
   *
   *   1 --- 0 --- 2
   *     \   |   /
   *       \ | /
   *         3
   *
   *  The horizontal links can slide around on the x axis.
   *  The "diagonal" links all want to be perfectly vertical.
   * 
   */
  
  pgs.addEdge(1, 0, (Affine3d)Translation3d(Eigen::Vector3d(1, 0, 0)), cov1);
  pgs.addEdge(0, 2, (Affine3d)Translation3d(Eigen::Vector3d(1, 0, 0)), cov1);
  pgs.addEdge(0, 3, (Affine3d)Translation3d(Eigen::Vector3d(0, 1, 0)), cov0);
  pgs.addEdge(1, 3, (Affine3d)Translation3d(Eigen::Vector3d(0, 1, 0)), cov0);
  pgs.addEdge(2, 3, (Affine3d)Translation3d(Eigen::Vector3d(0, 1, 0)), cov0);

  pgs.solve();
  for(size_t i = 0; i < pgs.numNodes(); ++i) {
    cout << "--------------------" << endl;
    cout << "Location of node " << i << endl;
    cout << pgs.transform(i).translation() << endl;
  }

  // 1 and 2 should have moved to 0.
  // 3 should have stayed fixed.
  EXPECT_TRUE(pgs.transform(1).translation().norm() < 1e-2);
  EXPECT_TRUE(pgs.transform(2).translation().norm() < 1e-2);
  EXPECT_TRUE((pgs.transform(3).translation() - Eigen::Vector3d(0, 1, 0)).norm() < 1e-3);
}

TEST(PoseGraphSlam, IO)
{
  PoseGraphSlam pgs(5);
  // Very high confidence all around.
  Matrix6d cov0 = Matrix6d::Identity() * 1e-3;  
  // x translation can slide, everything else wants to be fixed.
  Matrix6d cov1 = Matrix6d::Identity() * 1e-3;
  cov1(0, 0) = 1;

  
  pgs.addEdge(1, 0, (Affine3d)Translation3d(Eigen::Vector3d(1, 0, 0)), cov1);
  pgs.addEdge(0, 2, (Affine3d)Translation3d(Eigen::Vector3d(1, 0, 0)), cov1);
  pgs.addEdge(0, 3, (Affine3d)Translation3d(Eigen::Vector3d(0, 1, 0)), cov0);
  pgs.addEdge(1, 3, (Affine3d)Translation3d(Eigen::Vector3d(0, 1, 0)), cov0);
  pgs.addEdge(2, 3, (Affine3d)Translation3d(Eigen::Vector3d(0, 1, 0)), cov0);


  pgs.save("/tmp/foo.graph");
  pgs.solve();
  PoseGraphSlam pgs2;
  pgs2.load("/tmp/foo.graph");
  pgs2.solve();
  for(size_t i = 0; i < pgs.numNodes(); ++i) {
    EXPECT_TRUE( (pgs.transform(i).matrix() - pgs2.transform(i).matrix()).norm() < 1e-2 );
  }

}

TEST(PoseGraphSlam, Subgraphs)
{
  PoseGraphSlam pgs(10);
  Matrix6d covariance = Matrix6d::Identity();
  //Even
  pgs.addEdge(0, 2,
              (Affine3d)Translation3d(Eigen::Vector3d(0, 1, 0)),
              covariance);
  pgs.addEdge(2, 4,
              Translation3d(Eigen::Vector3d(0, 1, 0)) * Affine3d(AngleAxis<double>(M_PI / 4, Eigen::Vector3d(1, 0, 0))),
              covariance);
  pgs.addEdge(4, 6,
              Translation3d(Eigen::Vector3d(0, 1, 0)) * Affine3d(AngleAxis<double>(M_PI / 4, Eigen::Vector3d(1, 0, 0))),
              covariance);  
  //Odd
  pgs.addEdge(1, 3,
              (Affine3d)Translation3d(Eigen::Vector3d(0, 1, 0)),
              covariance);
  pgs.addEdge(3, 5,
              Translation3d(Eigen::Vector3d(0, 1, 0)) * Affine3d(AngleAxis<double>(M_PI / 4, Eigen::Vector3d(1, 0, 0))),
              covariance);
  pgs.addEdge(5, 7,
              Translation3d(Eigen::Vector3d(0, 1, 0)) * Affine3d(AngleAxis<double>(M_PI / 4, Eigen::Vector3d(1, 0, 0))),
              covariance);  
  size_t minsize = 2;
  int numsubgraphs = pgs.solve(2);
  EXPECT_TRUE(numsubgraphs == 2);
  Eigen::Vector3d translation;
  Quaterniond quat;
  for(size_t i = 0; i < pgs.numNodes(); ++i) {
    cout << "--------------------" << endl;
    pgs.vertexData(i, &translation, &quat);
    cout << "Translation: " << translation.transpose() << endl;
    cout << "Quaternion: " << quat.vec().transpose() << endl;
    cout << "Quaternion (w, x, y, z): " << quat.w() << " " << quat.x() << " " << quat.y() << " " << quat.z() << endl;
    cout << "Quaternion in matrix form:" << endl << quat.toRotationMatrix() << endl;
    cout << "Transform " << i << endl;
    cout << pgs.transform(i).matrix() << endl;
  }

  //Even
  EXPECT_TRUE((pgs.transform(0).matrix() - Matrix4d::Identity()).norm() < 1e-6);

  // For each pose, if the sensor sees a point 1 unit ahead (along y), make sure it appears
  // where we expect it to in global coordinates.
  Eigen::Vector4d transformed;
  Eigen::Vector4d expected;
  transformed = pgs.transform(2) * Eigen::Vector4d(0, 1, 0, 1);
  expected = Eigen::Vector4d(0, 2, 0, 1);
  cout << "1: " << transformed.transpose() << ", expected " << expected.transpose() << endl;
  EXPECT_TRUE((transformed - expected).norm() < 1e-3);

  transformed = pgs.transform(4) * Eigen::Vector4d(0, 1, 0, 1);
  expected = Eigen::Vector4d(0, 2.70711, .70711, 1);
  cout << "2: " << transformed.transpose() << ", expected " << expected.transpose() << endl;
  EXPECT_TRUE((transformed - expected).norm() < 1e-3);

  transformed = pgs.transform(6) * Eigen::Vector4d(0, 1, 0, 1);
  expected = Eigen::Vector4d(0, 2.70711, 1.70711, 1);
  cout << "3: " << transformed.transpose() << ", expected " << expected.transpose() << endl;
  EXPECT_TRUE((transformed - expected).norm() < 1e-3);

  Affine3d transform4to0 = (Translation3d(Eigen::Vector3d(0, 1, 0))
                            * Translation3d(Eigen::Vector3d(0, 1, 0)) * AngleAxis<double>(M_PI / 4, Eigen::Vector3d(1, 0, 0)));
  cout << transform4to0.matrix() << endl << endl;
  cout << pgs.transform(4).matrix() << endl << endl;
  EXPECT_TRUE((pgs.transform(4).matrix() - transform4to0.matrix()).norm() < 1e-3);

  Affine3d transform6to0 = (Translation3d(Eigen::Vector3d(0, 1, 0))
                            * Translation3d(Eigen::Vector3d(0, 1, 0)) * AngleAxis<double>(M_PI / 4, Eigen::Vector3d(1, 0, 0))
                            * Translation3d(Eigen::Vector3d(0, 1, 0)) * AngleAxis<double>(M_PI / 4, Eigen::Vector3d(1, 0, 0)));
  cout << transform6to0.matrix() << endl << endl;
  cout << pgs.transform(6).matrix() << endl << endl;
  EXPECT_TRUE((pgs.transform(6).matrix() - transform6to0.matrix()).norm() < 1e-3);
  //Odd
  EXPECT_TRUE((pgs.transform(1).matrix() - Matrix4d::Identity()).norm() < 1e-6);

  // For each pose, if the sensor sees a point 1 unit ahead (along y), make sure it appears
  // where we expect it to in global coordinates.
  transformed = pgs.transform(3) * Eigen::Vector4d(0, 1, 0, 1);
  expected = Eigen::Vector4d(0, 2, 0, 1);
  cout << "1: " << transformed.transpose() << ", expected " << expected.transpose() << endl;
  EXPECT_TRUE((transformed - expected).norm() < 1e-3);

  transformed = pgs.transform(5) * Eigen::Vector4d(0, 1, 0, 1);
  expected = Eigen::Vector4d(0, 2.70711, .70711, 1);
  cout << "2: " << transformed.transpose() << ", expected " << expected.transpose() << endl;
  EXPECT_TRUE((transformed - expected).norm() < 1e-3);

  transformed = pgs.transform(7) * Eigen::Vector4d(0, 1, 0, 1);
  expected = Eigen::Vector4d(0, 2.70711, 1.70711, 1);
  cout << "3: " << transformed.transpose() << ", expected " << expected.transpose() << endl;
  EXPECT_TRUE((transformed - expected).norm() < 1e-3);

  Affine3d transform5to1 = (Translation3d(Eigen::Vector3d(0, 1, 0))
                            * Translation3d(Eigen::Vector3d(0, 1, 0)) * AngleAxis<double>(M_PI / 4, Eigen::Vector3d(1, 0, 0)));
  cout << transform5to1.matrix() << endl << endl;
  cout << pgs.transform(5).matrix() << endl << endl;
  EXPECT_TRUE((pgs.transform(5).matrix() - transform5to1.matrix()).norm() < 1e-3);

  Affine3d transform7to1 = (Translation3d(Eigen::Vector3d(0, 1, 0))
                            * Translation3d(Eigen::Vector3d(0, 1, 0)) * AngleAxis<double>(M_PI / 4, Eigen::Vector3d(1, 0, 0))
                            * Translation3d(Eigen::Vector3d(0, 1, 0)) * AngleAxis<double>(M_PI / 4, Eigen::Vector3d(1, 0, 0)));
  cout << transform7to1.matrix() << endl << endl;
  cout << pgs.transform(7).matrix() << endl << endl;
  EXPECT_TRUE((pgs.transform(7).matrix() - transform7to1.matrix()).norm() < 1e-3);
  //Get roots
  int root;
  pgs.transform(0,&root); EXPECT_TRUE(root == 0);
  pgs.transform(2,&root); EXPECT_TRUE(root == 0);
  pgs.transform(4,&root); EXPECT_TRUE(root == 0);
  pgs.transform(6,&root); EXPECT_TRUE(root == 0);
  pgs.transform(1,&root); EXPECT_TRUE(root == 1);
  pgs.transform(3,&root); EXPECT_TRUE(root == 1);
  pgs.transform(5,&root); EXPECT_TRUE(root == 1);
  pgs.transform(7,&root); EXPECT_TRUE(root == 1);
  pgs.transform(8,&root); EXPECT_TRUE(root < 0);
  pgs.transform(9,&root); EXPECT_TRUE(root < 0);
  //Try adding island
  pgs.addEdge(8, 9,
              Translation3d(Eigen::Vector3d(0, 1, 0)) * Affine3d(AngleAxis<double>(M_PI / 4, Eigen::Vector3d(1, 0, 0))),
              covariance);  
  numsubgraphs = pgs.solve();
  EXPECT_EQ(numsubgraphs, 3);
  pgs.transform(9,&root); EXPECT_EQ(root, 8);
  numsubgraphs = pgs.solve(3);
  EXPECT_EQ(numsubgraphs, 2);
  pgs.transform(9,&root); EXPECT_EQ(root, -1);
  pgs.removeEdge(pgs.edges_.size()-1);
  numsubgraphs = pgs.solve(2);
  EXPECT_EQ(numsubgraphs,2);
  //Check subgraphs
  vector<vector<int> > subgraphs;
  pgs.getSubgraphs(subgraphs);
  EXPECT_EQ(subgraphs.size(),2);
  EXPECT_EQ(subgraphs[0].size(),4);
  for(size_t i = 0; i < subgraphs[0].size(); i++)
    EXPECT_TRUE(subgraphs[0][i] % 2 == 0);
  EXPECT_EQ(subgraphs[1].size(),4);
  for(size_t i = 0; i < subgraphs[1].size(); i++)
    EXPECT_TRUE(subgraphs[1][i] % 2 == 1);
  // Link together
  pgs.addEdge(1,2, Eigen::Affine3d(Eigen::Translation3d(0,0,1)), covariance);
  numsubgraphs = pgs.solve();
  EXPECT_EQ(numsubgraphs, 1);
  for(size_t i = 0; i < 8; i++)
  {
    pgs.transform(i, &root); EXPECT_EQ(root,0);
    if(i != 0)
      EXPECT_TRUE((pgs.transform(i).matrix() - Eigen::Matrix4d::Identity()).norm() > 1e-3);
  }
    

}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

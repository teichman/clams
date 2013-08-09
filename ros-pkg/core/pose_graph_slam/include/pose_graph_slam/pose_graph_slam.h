#ifndef POSE_GRAPH_SLAM_H
#define POSE_GRAPH_SLAM_H

#include <iostream>
#include <vector>
#include <string>
#include <ros/console.h>
#include <ros/assert.h>
#include <g2o/types/slam3d/vertex_se3_quat.h>
#include <g2o/types/slam3d/edge_se3_quat.h>
#include <g2o/core/graph_optimizer_sparse.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <eigen_extensions/eigen_extensions.h>
#include <serializable/serializable.h>
#include <pose_graph_slam/connected_components.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef Eigen::Matrix<double, 6, 6> Matrix6d;

struct EdgeStruct
{
  int idx0;
  int idx1;
  Eigen::Affine3d transform;
  Matrix6d covariance;
};

class PoseGraphSlam : public Serializable
{
public:
  typedef boost::shared_ptr<PoseGraphSlam> Ptr;
  typedef boost::shared_ptr<const PoseGraphSlam> ConstPtr;
  
  //! The first node is assumed to be at the origin with identity rotation.
  PoseGraphSlam(size_t num_nodes=0); // Adding default constructor for loading case
  //! Factoring out constructor logic for deserialization
  void initialize(size_t num_nodes);
  //! transform takes points seen by idx1 and puts them in the idx0 frame.
  //! Applied to (0, 0, 0, 0, 0, 0), this gives the pose of idx1 in the idx0 frame.
  //! The covariance matrix orders the variables as translation, then rotation.
  //! covariance must be positive definite.
  void addEdge(int idx0, int idx1,
               const Eigen::Affine3d& transform,
               const Matrix6d& covariance);
  //! Remove the given edge from the map.
  void removeEdge(size_t idx);
  //! Solve all subgraphs above size min_size, return # subgraphs
  size_t solve(size_t min_size = 2, int num_iters = 10);
  //! Get the different subgraphs present since the last solve
  void getSubgraphs(std::vector<std::vector<int> > &subgraphs); //Returns the nodes
  
  //! Gets the transform that will send points from idx's local coordinate frame
  //! to the global one which the first node defines.
  //! global = transform(idx) * local
  //! transform rotates, then translates.
  Eigen::Affine3d transform(int idx, int *root_idx=NULL);
  void vertexData(int idx, Eigen::Vector3d* translation, Eigen::Quaterniond* quat,
      int *root_idx=NULL);
  
  size_t numNodes() const { return nodes_.size(); }
  size_t numEdges() const { return edges_.size(); }
  //! Number of edges for vertex idx.
  size_t numEdges(int idx);
  //! Serialization
  void serialize(std::ostream& out) const; 
  void deserialize(std::istream& in); 
  //! Prunes all edges which are perfectly satisfied . Return the number removed
  size_t pruneAllSatisfiedEdges();
  //! Prune all edges which are unsatisfied within a certain translation and/or rotation threshold
  size_t pruneUnsatisfiedEdgesBatch(float max_translation, float max_rotation);
  //! Prune all edges which are unsatisfied within a certain translation or rotation threshold
  //! where "step" says after how many prunings you will re-solve the pose graph. 
  size_t pruneUnsatisfiedEdges(float max_translation, float max_rotation, size_t step=1);
  //! Visualize in the given PCLVisualizer
  void visualize(pcl::visualization::PCLVisualizer &vis, double max_error=0.2);

  std::vector<int> nodes_;
  std::vector<EdgeStruct> edges_;
  //! Set if you want print statements about the optimization
  bool verbose_; 
protected:
  typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  SlamBlockSolver;
  typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
  typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearCholmodSolver;
  struct G2OStruct
  {
    G2OStruct();
    ~G2OStruct();
    g2o::SparseOptimizer *optimizer_;
    SlamLinearCholmodSolver *linear_solver_;
    SlamBlockSolver *solver_;
  };
  typedef boost::shared_ptr<G2OStruct> G2OPtr;
  typedef boost::shared_ptr<const G2OStruct> G2OConstPtr;
  //! Compute the current graph structure
  Graph_t::Ptr getGraph();
  //! Compute the current subgraphs
  void getSubgraphs(std::vector<Graph_t::Ptr> &subgraphs, size_t min_size = 2);
  //! Prepare a g2o solver to solve the given subgraph
  G2OPtr prepareSolver(const std::vector<int> &nodes, const std::vector<EdgeStruct> &edges);

  Graph_t::Ptr graph_cache_;
  std::vector<Graph_t::Ptr> subgraph_cache_;
  std::map<int, bool> has_subgraph_;
  std::map<int, size_t> idx_to_subgraph_idx_;
  std::vector<G2OPtr> optimizers_cache_;
  std::vector<SlamLinearCholmodSolver* > linear_solvers_cache_;
  std::vector<SlamBlockSolver* > block_solvers_cache_;
  bool update_graph_;
  bool update_subgraphs_;
  size_t cached_min_size_;
};

#endif // POSE_GRAPH_SLAM_H

#include <pose_graph_slam/pose_graph_slam.h>
#include <eigen_extensions/eigen_extensions.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace g2o;

typedef pcl::PointXYZ PointBW_t;
typedef pcl::PointCloud<PointBW_t> CloudBW_t;

PoseGraphSlam::PoseGraphSlam(size_t num_nodes)
{
  initialize(num_nodes);
}

void
PoseGraphSlam::initialize(size_t num_nodes)
{
  verbose_ = false;
  nodes_.resize(num_nodes);
  for(size_t i = 0; i < num_nodes; i++)
    nodes_[i] = (int)i;
}

void PoseGraphSlam::addEdge(int idx0, int idx1,
                            const Eigen::Affine3d& transform,
                            const Matrix6d& covariance)
{
  EdgeStruct edge_struct;
  edge_struct.idx0 = idx0;
  edge_struct.idx1 = idx1;
  edge_struct.transform = transform;
  edge_struct.covariance = covariance;
  edges_.push_back(edge_struct); 
  update_graph_ = true;
  update_subgraphs_ = true;
}
  
void PoseGraphSlam::removeEdge(size_t idx)
{
  edges_.erase(edges_.begin()+idx);
  update_graph_ = true;
  update_subgraphs_ = true;
}

size_t PoseGraphSlam::solve(size_t min_size, int num_iters)
{
  ROS_ASSERT(min_size > 1);
  vector<Graph_t::Ptr> subgraphs; getSubgraphs(subgraphs, min_size);
  optimizers_cache_.resize(subgraphs.size());
  for(size_t i = 0; i <  subgraphs.size(); i++)
  {
    Graph_t::Ptr subgraph = subgraphs[i];
    vector<int> nodes(subgraph->nodes_.size());
    vector<EdgeStruct> edges(subgraph->edges_.size());
    for(size_t j = 0; j < subgraph->nodes_.size(); j++)
      nodes[j] = subgraph->nodes_[j];
    for(size_t j = 0; j < subgraph->edges_.size(); j++)
      edges[j] = *((EdgeStruct*) subgraph->edges_[j].data_ptr);
    optimizers_cache_[i] = prepareSolver(nodes, edges);
    if(verbose_)
      cout << "Optimizing subgraph " << i+1 << "/" << subgraphs.size() << endl;
    optimizers_cache_[i]->optimizer_->initializeOptimization();
    optimizers_cache_[i]->optimizer_->optimize(num_iters);
    if(verbose_)
      cout << "done." << endl;
  }
  return subgraphs.size();
}
  
void PoseGraphSlam::getSubgraphs(vector<vector<int> > &subgraph_nodes)
{
  ROS_ASSERT(!update_graph_ && !update_subgraphs_);
  vector<Graph_t::Ptr> subgraphs; getSubgraphs(subgraphs, cached_min_size_);
  subgraph_nodes.resize(subgraphs.size());
  for(size_t i = 0; i < subgraphs.size(); i++)
    subgraph_nodes[i] = subgraphs[i]->nodes_;
}

Eigen::Affine3d PoseGraphSlam::transform(int idx, int *root_idx)
{
  //// Fail if called before solve() was called
  //ROS_ASSERT(!update_graph_ && !update_subgraphs_);
  if(!has_subgraph_[idx])
  {
    if(root_idx) *root_idx = -1;
    return Eigen::Affine3d(Eigen::Matrix4d::Identity());
  }
  size_t graph_idx = idx_to_subgraph_idx_[idx];
  const VertexSE3* v = dynamic_cast<const VertexSE3*>(
      optimizers_cache_[graph_idx]->optimizer_->vertex(idx));
  ROS_ASSERT(v);
  SE3Quat se3 = v->estimate();
  if(root_idx)
    *root_idx = subgraph_cache_[graph_idx]->nodes_[0];
  return Eigen::Affine3d(se3.to_homogenious_matrix());  // awesome.
}

void PoseGraphSlam::vertexData(int idx, Vector3d* translation, Quaterniond* quat, 
    int *root_idx)
{
  if(!has_subgraph_[idx])
  {
    if(root_idx) *root_idx = -1;
    return;
  }
  int graph_idx = idx_to_subgraph_idx_[idx];
  double data[7];
  optimizers_cache_[graph_idx]->optimizer_->vertex(idx)->getEstimateData(data);
  *translation << data[0], data[1], data[2];
  *quat = Quaterniond(data[6], data[3], data[4], data[5]);
  if(root_idx)
    *root_idx = subgraph_cache_[graph_idx]->nodes_[0];
}

// TODO Alter so it respects the subgraph way of doing things?
size_t PoseGraphSlam::numEdges(int idx)
{
  vector<int> neighbors; getGraph()->getNeighbors(idx, neighbors);
  return neighbors.size();
}

void PoseGraphSlam::serialize(std::ostream& out) const
{
  //Output num vertices
  out << numNodes() << endl;
  //Output num edges
  out << edges_.size() << endl;
  for(size_t i = 0; i < edges_.size(); i++)
  {
    out << edges_[i].idx0 << " " << edges_[i].idx1 << endl;
    eigen_extensions::serializeASCII(edges_[i].transform.matrix(), out);
    eigen_extensions::serializeASCII(edges_[i].covariance, out);
  }
}
void PoseGraphSlam::deserialize(std::istream& in)
{
  size_t n_vert; in >> n_vert;
  if(numNodes() != n_vert)
  {
    ROS_ASSERT(numNodes() == 0);
    initialize(n_vert);
    ROS_ASSERT(numNodes() == n_vert);
  }
  size_t n_edges; in >> n_edges;
  for(size_t i = 0; i < n_edges; i++)
  {
    int idx0; in >> idx0;
    int idx1; in >> idx1;
    Eigen::Matrix4d m; eigen_extensions::deserializeASCII(in, &m);
    Eigen::Affine3d transform; transform.matrix() = m;
    Matrix6d covariance; eigen_extensions::deserializeASCII(in, &covariance);
    addEdge(idx0, idx1, transform, covariance);
  }
}
  
PoseGraphSlam::G2OPtr 
PoseGraphSlam::prepareSolver(const vector<int> &nodes, const vector<EdgeStruct> &edges)
{
  //Create the optimizer
  G2OPtr solver(new G2OStruct);
  if(verbose_) solver->optimizer_->setVerbose(true);
  //Add all nodes
  for(size_t i = 0; i < nodes.size(); i++)
  {
    VertexSE3* v = new VertexSE3;
    v->setId(nodes[i]);
    if(i == 0)
    {
      v->setToOrigin();
      v->setFixed(true);
    }
    solver->optimizer_->addVertex(v);
  }
  //Add all edges
  for(size_t i = 0; i < edges.size(); i++)
  {
    const EdgeStruct &edge = edges[i];
    EdgeSE3* edgeptr = new EdgeSE3;
    edgeptr->vertices()[0] = solver->optimizer_->vertex(edge.idx0);
    edgeptr->vertices()[1] = solver->optimizer_->vertex(edge.idx1);
    Matrix3d rotation = edge.transform.matrix().block(0, 0, 3, 3);
    Vector3d translation = edge.transform.translation();
    SE3Quat se3(rotation, translation);
    edgeptr->setMeasurement(se3);
    edgeptr->setInverseMeasurement(se3.inverse());
    edgeptr->setInformation(edge.covariance.inverse());
    solver->optimizer_->addEdge(edgeptr);
  }
  return solver;
}

PoseGraphSlam::G2OStruct::G2OStruct()
{
  linear_solver_ = new SlamLinearCholmodSolver();
  linear_solver_->setBlockOrdering(false);
  optimizer_ = new g2o::SparseOptimizer;
  optimizer_->setVerbose(false);
  solver_ = new SlamBlockSolver(optimizer_, linear_solver_);
  optimizer_->setSolver(solver_);
}

PoseGraphSlam::G2OStruct::~G2OStruct()
{
  //delete linear_solver_;
  //delete solver_;
  delete optimizer_;
}

Graph_t::Ptr PoseGraphSlam::getGraph()
{
  if(update_graph_)
  {
    if(verbose_)
      cout << "Updating graph" << endl;
    graph_cache_ = Graph_t::Ptr(new Graph_t);
    for(size_t i = 0; i < nodes_.size(); i++)
      graph_cache_->addNode(nodes_[i]);
    for(size_t i = 0; i < edges_.size(); i++)
      graph_cache_->addEdge(edges_[i].idx0, edges_[i].idx1, (void*) &edges_[i]);
    update_graph_ = false;
    update_subgraphs_ = true; //Must update subgraphs after doing this
  }
  return graph_cache_;
}

void PoseGraphSlam::getSubgraphs(vector<Graph_t::Ptr> &subgraphs, size_t min_size)
{
  if(update_subgraphs_ || min_size != cached_min_size_)
  {
    if(verbose_)
      cout << "Updating subgraph" << endl;
    Graph_t::Ptr graph = getGraph();
    subgraph_cache_.clear();
    graph->getSubgraphs(subgraph_cache_, min_size);
    update_subgraphs_ = false;
    cached_min_size_ = min_size;
    has_subgraph_.clear();
    idx_to_subgraph_idx_.clear();
    // Update index map
    for(size_t i = 0; i < subgraph_cache_.size(); i++)
      for(size_t j = 0; j < subgraph_cache_[i]->nodes_.size(); j++)
      {
        has_subgraph_[subgraph_cache_[i]->nodes_[j]] = true;
        idx_to_subgraph_idx_[subgraph_cache_[i]->nodes_[j]] = i;
      }
  }
  subgraphs = subgraph_cache_;
}
  
//! Prunes all edges which are perfectly satisfied. Returns the number removed
size_t PoseGraphSlam::pruneAllSatisfiedEdges()
{
  solve();
  vector<size_t> to_remove;
  for(size_t i = 0; i < edges_.size(); i++)
  {
    const EdgeStruct &e = edges_[i];
    Eigen::Affine3d actual0 = transform(e.idx0);
    Eigen::Affine3d actual1 = transform(e.idx1);
    Eigen::Affine3d actual_pairwise = actual0.inverse()*actual1;
    if((actual_pairwise.matrix() - e.transform.matrix()).norm() < 1E-4)
      to_remove.push_back(i);
  }
  for(size_t i = 0; i < to_remove.size(); i++)
    removeEdge(i);
  return to_remove.size();
}
//! Prune all edges which are unsatisfied within a certain translation or rotation threshold
size_t PoseGraphSlam::pruneUnsatisfiedEdgesBatch(float max_translation, float max_rotation)
{
  solve();
  vector<size_t> to_remove;
  for(size_t i = 0; i < edges_.size(); i++)
  {
    const EdgeStruct &e = edges_[i];
    Eigen::Affine3d actual0 = transform(e.idx0);
    Eigen::Affine3d actual1 = transform(e.idx1);
    Eigen::Affine3d actual_pairwise = actual0.inverse()*actual1;
    Eigen::Affine3d error = actual_pairwise.inverse()*e.transform;
    if(error.translation().norm() > max_translation || 
        fabs(Eigen::AngleAxisd(error.rotation()).angle()) > max_rotation)
      to_remove.push_back(i);
  }
  for(size_t i = 0; i < to_remove.size(); i++)
    removeEdge(i);
  return to_remove.size();
}
//! Prune all edges which are unsatisfied within a certain translation or rotation threshold
size_t PoseGraphSlam::pruneUnsatisfiedEdges(float max_translation, float max_rotation, size_t step)
{
  solve();
  size_t num_removed = 0;
  while(true)
  {
    double max_trans = -1*std::numeric_limits<float>::infinity();
    double max_rot = -1*std::numeric_limits<float>::infinity();
    int max_trans_idx = -1;
    int max_rot_idx = -1;
    for(size_t i = 0; i < edges_.size(); i++)
    {
      const EdgeStruct &e = edges_[i];
      Eigen::Affine3d actual0 = transform(e.idx0);
      Eigen::Affine3d actual1 = transform(e.idx1);
      Eigen::Affine3d actual_pairwise = actual0.inverse()*actual1;
      Eigen::Affine3d error = actual_pairwise.inverse()*e.transform;
      double trans = error.translation().norm();
      if(trans > max_trans && trans > max_translation)
      {
        max_trans = trans;
        max_trans_idx = i;
      }
      double rot = fabs(Eigen::AngleAxisd(error.rotation()).angle());
      if(rot > max_rot && rot > max_rotation)
      {
        max_rot = rot;
        max_rot_idx = i;
      }
    }
    if(max_trans_idx >= 0 && max_rot_idx >= 0)
    {
      removeEdge( max_trans/max_translation > max_rot / max_rotation ? 
          max_trans_idx : max_rot_idx );
    }
    else if(max_trans_idx >= 0)
      removeEdge(max_trans_idx);
    else if(max_rot_idx >= 0)
      removeEdge(max_rot_idx);
    else
      break;
    num_removed++;
    if(num_removed % step == 0)
      solve();
  }
  return num_removed;
}
  
//! Visualize in the given PCLVisualizer
void PoseGraphSlam::visualize(pcl::visualization::PCLVisualizer &vis, double max_error)
{
  vis.removeAllShapes();
  vis.removeAllPointClouds();
  float dtheta = (7*M_PI/4) / numNodes();
  CloudBW_t::Ptr nodes(new CloudBW_t);
  CloudBW_t::Ptr active_nodes(new CloudBW_t);
  float r = 5;
  for(size_t i = 0; i < numNodes(); i++)
  {
    float x = r*cos(dtheta*i);
    float y = r*sin(dtheta*i);
    float z = 0.5;
    nodes->points.push_back(PointBW_t(x, y, z));
    if(numEdges(i) > 0)
      active_nodes->points.push_back(PointBW_t(x,y,z));
  }
  pcl::visualization::PointCloudColorHandlerCustom<PointBW_t> handler(nodes, 0, 0, 255);
  vis.addPointCloud(nodes, handler, "nodes");
  vis.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "nodes");
  // Draw active (connected) nodes in green
  int pt_r = 2;
  pcl::visualization::PointCloudColorHandlerCustom<PointBW_t> active_handler(active_nodes, 0, 255, 0);
  vis.addPointCloud(active_nodes, active_handler, "active_nodes");
  vis.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pt_r, "active_nodes");
  // Highlight the start point
  vis.addSphere(nodes->at(0), 0.05, 0, 255, 255, "start");
  // Draw all edges, colored by max violation where gray is nothing and red is max_error
  for(size_t i = 0; i < edges_.size(); i++)
  {
    const EdgeStruct &e = edges_[i];
    Eigen::Affine3d actual0 = transform(e.idx0);
    Eigen::Affine3d actual1 = transform(e.idx1);
    Eigen::Affine3d actual_pairwise = actual0.inverse()*actual1;
    double error = (actual_pairwise.matrix() - e.transform.matrix()).norm();
    const PointBW_t &start = nodes->at(e.idx0);
    const PointBW_t &end = nodes->at(e.idx1);
    ostringstream oss;
    oss << "edge_" << i;
    vis.addLine(start, end, (0.9*error/max_error)+0.1, 0.1, 0.1, oss.str());
  }

}

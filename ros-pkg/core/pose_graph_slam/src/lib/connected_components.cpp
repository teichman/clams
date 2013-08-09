#include <pose_graph_slam/connected_components.h>
using std::vector;

Graph_t::Graph_t()
{
}

void
Graph_t::addNode(int id)
{
  nodes_.push_back(id);
  id_to_idx_[id] = nodes_.size()-1;
}
void
Graph_t::addEdge(int id0, int id1, void* data_ptr)
{
  Edge_t e; e.id0 = id0; e.id1 = id1; e.data_ptr = data_ptr;
  edges_.push_back(e);
  // Cache
  neighbor_cache_[id0].push_back(id1);
  neighbor_cache_[id1].push_back(id0);
  outgoing_neighbor_cache_[id0].push_back(id1);
  edge_ptr_cache_[id0].push_back(data_ptr);
}
  
bool 
Graph_t::getNeighbors(int id, vector<int> &neighbors)
{
  const vector<Node_t> &cached_neighbors = neighbor_cache_[id];
  for(size_t i = 0; i < cached_neighbors.size(); i++)
    neighbors.push_back(cached_neighbors[i]);
  return cached_neighbors.size() > 0;
}

void 
Graph_t::getSubgraphs(vector<Graph_t::Ptr> &subgraphs, size_t min_size)
{
  subgraphs.clear();
  std::map<Node_t, bool> accounted_for;
  //First pass to get rid of nodes which couldn't possibly qualify
  if(min_size > 1)
  {
    for(size_t i = 0; i < nodes_.size(); i++)
    {
      if(neighbor_cache_[nodes_[i]].size() == 0)
        accounted_for[nodes_[i]] = true;
    }
  }
  for(size_t current_root_idx = 0; current_root_idx < nodes_.size(); current_root_idx++)
  {
    Node_t current_root = nodes_[current_root_idx];
    // Find the first elligible root
    if(accounted_for[current_root]) continue;
    // Find connected components
    vector<Node_t> connected; getConnectedNodes(current_root, connected);
    if(connected.size() < min_size) continue;
    // Create subgraph
    subgraphs.resize(subgraphs.size()+1);
    Graph_t::Ptr &subgraph = subgraphs[subgraphs.size()-1];
    subgraph = Graph_t::Ptr(new Graph_t);
    // Nodes
    for(size_t i = 0; i < connected.size(); i++)
    {
      subgraph->addNode(connected[i]);
    }
    // Edges
    for(size_t i = 0; i < connected.size(); i++)
    {
      const Node_t &node = connected[i];
      accounted_for[node] = true;
      const vector<Node_t> &outgoing_neighbors = outgoing_neighbor_cache_[node];
      const vector<void*> &edge_ptrs = edge_ptr_cache_[node];
      for(size_t i = 0; i < outgoing_neighbors.size(); i++)
        subgraph->addEdge(node, outgoing_neighbors[i], edge_ptrs[i]);
    }
  }
}
  
int
Graph_t::getConnectedNodes(Node_t root, vector<Node_t> &connected)
{
  std::map<Node_t, bool> visited;
  vector<Node_t> to_visit;
  to_visit.push_back(root);
  while(to_visit.size() > 0)
  {
    Node_t curnode = to_visit[to_visit.size()-1];
    to_visit.pop_back();
    if(visited[curnode]) continue;
    visited[curnode] = true;
    connected.push_back(curnode);
    vector<Node_t> neighbors; getNeighbors(curnode, neighbors);
    for(size_t i = 0; i < neighbors.size(); i++)
    {
      Node_t neighbor = neighbors[i];
      if(!visited[neighbor])
        to_visit.push_back(neighbor);
    }
  }
  return (int)connected.size() - 1;
}

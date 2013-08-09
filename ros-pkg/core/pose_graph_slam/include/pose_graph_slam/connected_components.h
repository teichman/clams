#ifndef CONNECTED_COMPONENTS_H
#define CONNECTED_COMPONENTS_H

#include <vector>
#include <map>
#include <Eigen/Eigen>
#include <boost/shared_ptr.hpp>


class Graph_t
{
public:
  typedef boost::shared_ptr<Graph_t> Ptr;
  typedef boost::shared_ptr<const Graph_t> ConstPtr;
  Graph_t();
  
  void addNode(int id);
  void addEdge(int id0, int id1, void* data_ptr=NULL);
  bool getNeighbors(int id, std::vector<int> &neighbors);
  void getSubgraphs(std::vector<Graph_t::Ptr> &subgraphs, size_t min_size=2);
  
  typedef int Node_t;
  struct Edge_t
  {
    Node_t id0;
    Node_t id1;
    void* data_ptr; //Optional pointer to some richer struct you'd like to have
  };
  
  // Stores actual node
  std::vector<Node_t> nodes_;
  // Edge by id
  std::vector<Edge_t> edges_;
protected:
  
  int getConnectedNodes(Node_t root, std::vector<Node_t> &connected);

  // Node by ID
  std::map<Node_t, size_t> id_to_idx_;
  // Cache of neighboring IDs
  std::map<Node_t, std::vector<Node_t> > neighbor_cache_;
  std::map<Node_t, std::vector<Node_t> > outgoing_neighbor_cache_;
  std::map<Node_t, std::vector<void*> > edge_ptr_cache_;
};

#endif //CONNECTED_COMPONENTS_H

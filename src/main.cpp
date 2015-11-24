#include <iostream>
#include "traffic_flows/traffic_flows.h"

int main() {

  Graph graph;
  for(int i = 0; i < 4; ++i)
  {
    graph.insert(std::pair<ulong, Edges>(i, Edges()));
  }
  
  EdgeInfo edge_info_0_1(2., 3., 1.);
  graph[0].insert(Edge(1, EdgeInfo(0., 3., 1.)));
  graph[0].insert(Edge(2, EdgeInfo(0., 3., 1.)));

  graph[1].insert(Edge(2, EdgeInfo(0., 3., 1.)));

  graph[1].insert(Edge(3, EdgeInfo(0., 3., 1.)));

  graph[2].insert(Edge(3, EdgeInfo(0., 3., 1.)));

  print_graph(graph);

  // auto path_ptr = Dijkstra::get_shortest_path(graph, 0, 3);
  
  // print_path_in_graph(path_ptr, graph);

  CorrespondenceVec corr_vec;
  corr_vec.push_back(Correspondence(0, 3, 3.));
  corr_vec.push_back(Correspondence(1, 2, 3.));
  GraphOptimizer opt(graph, corr_vec, 0.001);
  
  std::cout<<"Graph Optimizition"<<"\n";
  std::cout<<"===================\n";
  while(!opt.step())
  {
    std::cout<<"stats\n";
    std::cout<<"iterations: "<<opt.get_iteration_step()<<"\n";
    std::cout<<"psi value: "<<opt.get_psi()<<"\n";
    std::cout<<"===================\n";
    print_graph(graph);
  }
  std::cout<<"stats\n";
  std::cout<<"iterations: "<<opt.get_iteration_step()<<"\n";
  std::cout<<"psi value: "<<opt.get_psi()<<"\n";
  std::cout<<"===================\n";

  return 0;
}
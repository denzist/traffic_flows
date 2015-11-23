#include <iostream>
#include "traffic_flows/traffic_flows.h"

int main() {

  Graph graph;
  std::vector<PVertex> v_ptr_vec(4);
  for(int i = 0; i < 4; ++i)
  {
    graph.insert(std::pair<PVertex, EdgeVec>(v_ptr_vec[i], EdgeVec()));  
  }
  

  // EdgeInfo edge_info_0_1(2., 3., 1.);
  // graph[0]->add_edge(Edge(graph[1], EdgeInfo(1., 3., 1.)));
  // graph[0]->add_edge(Edge(graph[2], EdgeInfo(4., 3., 1.)));

  // graph[1]->add_edge(Edge(graph[2], EdgeInfo(1., 3., 1.)));
  // graph[1]->add_edge(Edge(graph[3], EdgeInfo(4., 3., 1.)));

  // graph[2]->add_edge(Edge(graph[3], EdgeInfo(1., 3., 1.)));


  // for(int i = 0; i < graph.size(); ++i)
  // {
  //   std::cout<<"i -- "<<graph[i]<<" : ";   
  //   for(auto it = graph[i]->get_edges().begin(); it != graph[i]->get_edges().end(); ++it)
  //   {
  //     std::cout<<it->second.get_cost()<<"; ";   
  //   }
  //   std::cout<<"\n";

  // }

  // auto path_ptr = Dijkstra::get_shortest_path(graph, graph[0], graph[3]);
  // double sum = 0;
  // std::cout<<"size: "<<path_ptr->size()<<"\n";
  // auto end = path_ptr->end();
  // --end;
  // std::cout<<"i: "<<*(path_ptr->begin())<<"\n";
  // for (auto it = path_ptr->begin(); it != end; ++it)
  // {
  //   auto next = it;
  //   ++next;
  //   EdgeInfo& info = (*it)->get_edges()[*next];
  //   std::cout<<"cost: "<<info.get_cost()<<"\n";
  //   std::cout<<"i: "<<*next<<"\n";
  //   sum += info.get_cost();
  // }

  // std::cout<<"path cost: "<<sum<<"\n";


  return 0;
}
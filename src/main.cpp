#include <iostream>
#include "traffic_flows/traffic_flows.h"

int main() {

  Graph graph;
  std::vector<PVertex> v_ptr_vec(4);
  for(int i = 0; i < 4; ++i)
  {
    v_ptr_vec[i] = Vertex::createPVertex();
    graph.insert(std::pair<PVertex, Edges>(v_ptr_vec[i], Edges()));
  }
  
  EdgeInfo edge_info_0_1(2., 3., 1.);
  graph[v_ptr_vec[0]].insert(Edge(v_ptr_vec[1], EdgeInfo(1., 3., 1.)));
  graph[v_ptr_vec[0]].insert(Edge(v_ptr_vec[2], EdgeInfo(4., 3., 1.)));

  graph[v_ptr_vec[1]].insert(Edge(v_ptr_vec[2], EdgeInfo(1., 3., 1.)));
  graph[v_ptr_vec[1]].insert(Edge(v_ptr_vec[3], EdgeInfo(4., 3., 1.)));

  graph[v_ptr_vec[2]].insert(Edge(v_ptr_vec[3], EdgeInfo(1., 3., 1.)));

  for (auto it = graph.begin(); it != graph.end(); ++it)
  {
    std::cout<<"i -- "<<it->first->get_id()<<" : ";   
    for(auto edge_it = it->second.begin(); edge_it != it->second.end(); ++edge_it)
    {
      std::cout<<edge_it->first->get_id()<<" -- ";   
      std::cout<<edge_it->second.get_cost()<<"; ";   
    }
    std::cout<<"\n";
  }

  auto path_ptr = Dijkstra::get_shortest_path(graph, v_ptr_vec[0], v_ptr_vec[3]);
  double sum = 0;
  std::cout<<"size: "<<path_ptr->size()<<"\n";
  auto end = path_ptr->end();
  --end;
  std::cout<<"i: "<<(*path_ptr->begin())->get_id()<<"\n";
  for (auto it = path_ptr->begin(); it != end; ++it)
  {
    auto next = it;
    ++next;
    EdgeInfo& info = graph[*it][*next];
    std::cout<<"cost: "<<info.get_cost()<<"\n";
    std::cout<<"i: "<<(*next)->get_id()<<"\n";
    sum += info.get_cost();
  }

  std::cout<<"path cost: "<<sum<<"\n";


  return 0;
}
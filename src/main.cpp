#include <iostream>
#include "traffic_flows/traffic_flows.h"

int main() {

  Graph graph;
  std::vector<PVertex> v_ptr_vec(4);
  for(int i = 0; i < 4; ++i)
  {
    v_ptr_vec[i] = Vertex::createPVertex();
    graph.insert(std::pair<PVertex, EdgeVec>(v_ptr_vec[i], EdgeVec()));  
  }
  
  EdgeInfo edge_info_0_1(2., 3., 1.);
  graph[v_ptr_vec[0]].push_back(Edge(v_ptr_vec[1], EdgeInfo(1., 3., 1.)));
  graph[v_ptr_vec[0]].push_back(Edge(v_ptr_vec[2], EdgeInfo(4., 3., 1.)));

  graph[v_ptr_vec[1]].push_back(Edge(v_ptr_vec[2], EdgeInfo(1., 3., 1.)));
  graph[v_ptr_vec[1]].push_back(Edge(v_ptr_vec[3], EdgeInfo(4., 3., 1.)));

  graph[v_ptr_vec[2]].push_back(Edge(v_ptr_vec[3], EdgeInfo(1., 3., 1.)));

  for (auto it = graph.begin(); it != graph.end(); ++it)
  {
    std::cout<<"i -- "<<it->first->get_id()<<" : ";   
    for(auto it_e = it->second.begin(); it_e != it->second.end(); ++it_e)
    {
      std::cout<<it_e->first->get_id()<<" -- ";   
      std::cout<<it_e->second.get_cost()<<"; ";   
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
    EdgeInfo& info = graph[*next].second;
    std::cout<<"cost: "<<info.get_cost()<<"\n";
    std::cout<<"i: "<<*next<<"\n";
    sum += info.get_cost();
  }

  std::cout<<"path cost: "<<sum<<"\n";


  return 0;
}
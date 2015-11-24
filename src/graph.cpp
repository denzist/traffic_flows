#include "traffic_flows/graph.h"

void print_graph(const Graph& graph)
{
  for (auto it = graph.begin(); it != graph.end(); ++it)
  {
    std::cout<<"Vertex: "<<it->first<<" :\n";   
    for(auto edge_it = it->second.begin(); edge_it != it->second.end(); ++edge_it)
    {
      std::cout<<"Edge "<<it->first<<"-->"<<edge_it->first<<" : ";   
      std::cout<<"flow: "<<edge_it->second.get_flow();
      std::cout<<", cost: "<<edge_it->second.get_cost()<<"\n";   
    }
    std::cout<<"\n-----------\n";
  }
}

void print_path_in_graph(const PPath path_ptr, Graph& graph)
{
  double sum = 0;
  std::cout<<"Path size: "<<path_ptr->size()<<"\n";
  auto end = path_ptr->end();
  --end;
  for (auto it = path_ptr->begin(); it != end; ++it)
  {
    auto next = it;
    ++next;
    std::cout<<"Edge "<<*it<<"-->"<<*next;   
    EdgeInfo& info = graph[*it][*next];
    std::cout<<" cost: "<<info.get_cost()<<"\n";
    sum += info.get_cost();
  }
  std::cout<<"Path cost: "<<sum<<"\n";
}
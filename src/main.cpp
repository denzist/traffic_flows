#include <iostream>
#include "traffic_flows/traffic_flows.h"

int main() {

  PGraph graph_ptr(new Graph());
  Graph& graph = *graph_ptr;
  for(int i = 0; i < 4; ++i)
  {
    graph.insert(std::pair<ulong, Edges>(i, Edges()));
  }
  
  EdgeInfo edge_info_0_1(2., 3., 1.);
  graph[0].insert(Edge(1, EdgeInfo(0., 3., 1.)));
  graph[0].insert(Edge(2, EdgeInfo(0., 3., 1.)));

  graph[1].insert(Edge(3, EdgeInfo(0., 3., 1.)));

  graph[2].insert(Edge(3, EdgeInfo(0., 3., 1.)));

  for (auto it = graph.begin(); it != graph.end(); ++it)
  {
    std::cout<<"i -- "<<it->first<<" : ";   
    for(auto edge_it = it->second.begin(); edge_it != it->second.end(); ++edge_it)
    {
      std::cout<<edge_it->first<<" -- ";   
      std::cout<<edge_it->second.get_flow()<<"; "; 
    }
    std::cout<<"\n";
  }

  auto path_ptr = Dijkstra::get_shortest_path(graph, 0, 3);
  double sum = 0;
  std::cout<<"size: "<<path_ptr->size()<<"\n";
  auto end = path_ptr->end();
  --end;
  std::cout<<"i: "<<*path_ptr->begin()<<"\n";
  for (auto it = path_ptr->begin(); it != end; ++it)
  {
    auto next = it;
    ++next;
    EdgeInfo& info = graph[*it][*next];
    std::cout<<"cost: "<<info.get_cost()<<"\n";
    std::cout<<"i: "<<*next<<"\n";
    sum += info.get_cost();
  }

  std::cout<<"path cost: "<<sum<<"\n";

  std::shared_ptr<PCorrespondenceVec> pcorr_vec_ptr(new PCorrespondenceVec(1));
  (*pcorr_vec_ptr)[0] = Correspondence::createPCorrespondence(std::pair<ulong, ulong>(0, 3),3.);
  GraphOptimizer opt(graph_ptr, pcorr_vec_ptr);

  for(int  i = 0; i < 100; ++i)
  {
    opt.step();

    for (auto it = graph.begin(); it != graph.end(); ++it)
    {
      std::cout<<"i -- "<<it->first<<" : ";   
      for(auto edge_it = it->second.begin(); edge_it != it->second.end(); ++edge_it)
      {
        std::cout<<edge_it->first<<" -- ";   
        std::cout<<edge_it->second.get_flow()<<"; ";   
      }
      std::cout<<"\n";
    }
  }

  return 0;
}
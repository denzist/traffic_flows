#pragma once

#include <vector>
#include <memory>
#include <boost/utility.hpp>
#include <math.h>
#include <utility>
#include <map>
#include <limits>
#include "traffic_flows/graph.h"


class Dijkstra
{
public:
  static PPath get_shortest_path(const Graph& graph, const Correspondence& corr)
  {
    return get_shortest_path(graph, corr.get_start(), corr.get_end());
  }

  static PPath get_shortest_path(const Graph& graph, const PVertex start_ptr, const PVertex finish_ptr)
  {
    //init dijkstra info map of graph
    DijkstraInfoMap info_map;    
    for (auto it = graph.begin(); it != graph.end(); ++it)
    {
      DijkstraInfo info;
      info.dist = std::numeric_limits<double>::max();
      info_map.insert(std::pair<PVertex, DijkstraInfo>(it->first, info));
    }
    info_map[start_ptr].dist = 0.;
    
    //starting finding the min path
    auto curr_info = find_info_with_min_dist(info_map);

    while(curr_info->first != finish_ptr)
    {
      DijkstraInfo prev_info(curr_info->second);
      PVertex prev_vertex_ptr = curr_info->first;
      Edges edges = graph.find(curr_info->first)->second;
      info_map.erase(curr_info);

      for(auto it_edge = edges.begin(); it_edge != edges.end(); ++it_edge)
      {
        auto end_node_info = info_map.find(it_edge->first);
        if(end_node_info == info_map.end())
        {
          continue;
        }
        double dist = prev_info.dist + it_edge->second.get_cost();

        if(dist < end_node_info->second.dist)
        {
          end_node_info->second = prev_info;
          end_node_info->second.dist = dist;
          end_node_info->second.path_ptr->push_back(prev_vertex_ptr);
        }
      }
      if(!info_map.empty())
      {
        curr_info = find_info_with_min_dist(info_map);
      }
      else
      {
        break;
      }
    }
    curr_info->second.path_ptr->push_back(finish_ptr);
    return curr_info->second.path_ptr;
  }

private:
  struct DijkstraInfo
  {
    DijkstraInfo():
    dist(0.),
    path_ptr(new Path())
    {}

    DijkstraInfo(const DijkstraInfo& to_copy)
    {
      dist = to_copy.dist;
      path_ptr = std::make_shared<Path>(*(to_copy.path_ptr));
    }

    DijkstraInfo& operator=( const DijkstraInfo& to_copy)
    {
      dist = to_copy.dist;
      path_ptr = std::make_shared<Path>(*(to_copy.path_ptr));
      return *this;
    }

    double dist;
    PPath path_ptr;
  };

  typedef std::map<PVertex, DijkstraInfo> DijkstraInfoMap;
  
  static std::map<PVertex, DijkstraInfo>::iterator find_info_with_min_dist(DijkstraInfoMap& info_map)
  {
    double min_dist = info_map.begin()->second.dist;
    auto info_with_min_dist = info_map.begin();
    auto start = info_with_min_dist;
    ++start;
    for(auto it = start; it != info_map.end(); ++it)
    {
      if(it->second.dist < min_dist)
      {
        min_dist = it->second.dist;
        info_with_min_dist = it;
      }
    }
    return info_with_min_dist;
  }
};


class GraphOptimizer
{
public:
  GraphOptimizer(
    std::shared_ptr<Graph> graph_ptr,
    std::shared_ptr<CorrespondenceVec> corr_vec_ptr):
  graph_ptr_(graph_ptr),
  corr_vec_ptr_(corr_vec_ptr),
  iteration_number_(0),
  gamma_(1.)
  {}

  void step()
  {
    Graph& graph = *graph_ptr_;
    CorrespondenceVec& corr_vec = *corr_vec_ptr_;
    increase_iteration();

    std::vector<PPath> path_ptr_vec(corr_vec.size());
    for(int i = 0; i < corr_vec.size(); ++i)
    {
      path_ptr_vec[i] = Dijkstra::get_shortest_path(graph, corr_vec[i]);
    }

    // discont vertex costs by gamma_
    for (auto graph_it = graph.begin(); graph_it != graph.end(); ++graph_it)
    {
      for(auto edge_it = graph_it->second.begin(); edge_it != graph_it->second.end(); ++edge_it)
      {
        EdgeInfo& curr_info = edge_it->second;
        double disc_flow = (1. - gamma_)*curr_info.get_flow();
        curr_info.set_flow(disc_flow);  
      }
    }

    //walk around the graph and update flows
    for(int i = 0; i < path_ptr_vec.size(); ++i)
    {
      auto start = path_ptr_vec[i]->begin();
      auto end = path_ptr_vec[i]->end();
      --end;

      for (auto it = start; it != end; ++it)
      {
        auto next = it;
        ++next;

        EdgeInfo& curr_info = graph[*it][*next];
        double new_flow = curr_info.get_flow() + gamma_*corr_vec[i].get_total_flow();
        curr_info.set_flow(new_flow); //update cost
      }

    }

  }
private:
  std::shared_ptr<Graph> graph_ptr_;
  std::shared_ptr<CorrespondenceVec> corr_vec_ptr_;
  int iteration_number_;
  double gamma_;

  void increase_iteration()
  {
    gamma_ = 1./(1. + double(iteration_number_));
    ++iteration_number_;
  }
};


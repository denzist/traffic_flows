#pragma once

#include <vector>
#include <memory>
#include <boost/utility.hpp>
#include <math.h>
#include <utility>
#include <map>
#include <set>
#include <limits>
#include "traffic_flows/graph.h"


class Dijkstra
{
public:
  static PPath get_shortest_path(const Graph& graph, PCorrespondence corr_ptr)
  {
    return get_shortest_path(graph, corr_ptr->get_start(), corr_ptr->get_end());
  }

  static PPath get_shortest_path(const Graph& graph, const PVertex start_ptr, const PVertex finish_ptr)
  {
    //init dijkstra info map of graph

    DijkstraInfoMap info_map;
    VeretexSet vertices;    

    for (auto it = graph.begin(); it != graph.end(); ++it)
    {
      DijkstraInfo info;
      info_map.insert(std::pair<PVertex, DijkstraInfo>(it->first, info));
      vertices.insert(it->first);
    }
    info_map[start_ptr].dist_ = 0.;
    
    //starting finding the min path
    auto vertex_it = find_info_with_min_dist(info_map, vertices);

    while(*vertex_it != finish_ptr)
    {
      DijkstraInfo& prev_info = info_map[*vertex_it];
      PVertex prev_pvertex = *vertex_it;
      const Edges& edges = graph.find(prev_pvertex)->second;

      vertices.erase(vertex_it);

      for(auto it_edge = edges.begin(); it_edge != edges.end(); ++it_edge)
      {
        PVertex end_vertex = it_edge->first;
        DijkstraInfo& end_vertex_info = info_map[end_vertex];
        double dist = prev_info.dist_ + it_edge->second.get_cost();

        if(dist < end_vertex_info.dist_)
        {
          end_vertex_info.dist_ = dist;
          end_vertex_info.prev_pvertex_ = prev_pvertex;
        }
      }
      if(!vertices.empty())
      {
        vertex_it = find_info_with_min_dist(info_map, vertices);
      }
      else
      {
        break;
      }
    }
    PPath path_ptr = std::make_shared<Path>();
    path_ptr->insert(path_ptr->begin(), finish_ptr);
    PVertex prev_pvertex = info_map[*vertex_it].prev_pvertex_;
    while(prev_pvertex)
    {
      path_ptr->insert(path_ptr->begin(), prev_pvertex);
      prev_pvertex = info_map[prev_pvertex].prev_pvertex_;

    }
    return path_ptr;
  }

private:
  struct DijkstraInfo
  {
    DijkstraInfo():
    dist_(std::numeric_limits<double>::max()),
    prev_pvertex_(nullptr)
    {}

    double dist_;
    PVertex prev_pvertex_;
  };

  typedef std::map<PVertex, DijkstraInfo> DijkstraInfoMap;
  typedef std::set<PVertex> VeretexSet;
  
  static std::set<PVertex>::iterator find_info_with_min_dist(
    DijkstraInfoMap& info_map,
    VeretexSet& vertices)
  {
    auto vertex_with_min_dist = vertices.begin();
    double min_dist = info_map[*vertex_with_min_dist].dist_;
    auto start = vertex_with_min_dist;
    ++start;
    for(auto it = start; it != vertices.end(); ++it)
    {
      double dist = info_map[*it].dist_; 
      if(dist < min_dist)
      {
        min_dist = dist;
        vertex_with_min_dist = it;
      }
    }
    return vertex_with_min_dist;
  }
};


class GraphOptimizer
{
public:
  GraphOptimizer(
    std::shared_ptr<Graph> graph_ptr,
    std::shared_ptr<PCorrespondenceVec> pcorr_vec_ptr):
  graph_ptr_(graph_ptr),
  pcorr_vec_ptr_(pcorr_vec_ptr),
  iteration_number_(0),
  gamma_(1.)
  {}

  void step()
  {
    Graph& graph = *graph_ptr_;
    PCorrespondenceVec& pcorr_vec = *pcorr_vec_ptr_;
    increase_iteration();

    std::vector<PPath> path_ptr_vec(pcorr_vec.size());
    for(int i = 0; i < pcorr_vec.size(); ++i)
    {
      path_ptr_vec[i] = Dijkstra::get_shortest_path(graph, pcorr_vec[i]);
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
        double new_flow = curr_info.get_flow() + gamma_*pcorr_vec[i]->get_total_flow();
        curr_info.set_flow(new_flow); //update cost
      }

    }

  }
private:
  std::shared_ptr<Graph> graph_ptr_;
  std::shared_ptr<PCorrespondenceVec> pcorr_vec_ptr_;
  int iteration_number_;
  double gamma_;

  void increase_iteration()
  {
    gamma_ = 1./(1. + double(iteration_number_));
    ++iteration_number_;
  }
};


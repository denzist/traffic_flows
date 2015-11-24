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
  static PPath get_shortest_path(const Graph& graph, const Correspondence& corr)
  {
    return get_shortest_path(graph, corr.get_start(), corr.get_end());
  }

  static PPath get_shortest_path(const Graph& graph, const ulong start_ptr, const ulong finish_ptr);

private:
  struct DijkstraInfo
  {
    DijkstraInfo():
    dist_(std::numeric_limits<double>::max()),
    prev_vertex_(std::numeric_limits<ulong>::max())
    {}

    double dist_;
    ulong prev_vertex_;
  };

  typedef std::map<ulong, DijkstraInfo> DijkstraInfoMap;
  typedef std::set<ulong> VeretexSet;
  
  static std::set<ulong>::iterator find_info_with_min_dist(
    DijkstraInfoMap& info_map,
    VeretexSet& vertices);
};


class GraphOptimizer
{
public:
  GraphOptimizer(
    Graph& graph,
    const CorrespondenceVec& corr_vec,
    double relative_eps = 0.001):
  graph_(graph),
  corr_vec_(corr_vec),
  iteration_step_(0),
  gamma_(1.),
  max_psi_and_linear_apprx_(std::numeric_limits<double>::lowest()),
  psi_(0.),
  psi_linear_aprx_(0.),
  relative_eps_(relative_eps),
  end_flg_(false)
  {
    //init opt_curr_flow_ for every edge
    for (auto graph_it = graph_.begin(); graph_it != graph_.end(); ++graph_it)
    {
      for(auto edge_it = graph_it->second.begin(); edge_it != graph_it->second.end(); ++edge_it)
      {
        EdgeInfo& edge_info = edge_it->second;
        edge_info.opt_prev_flow_ = edge_info.flow_;
        edge_info.opt_curr_flow_ = 0.;
      }
    }

    std::vector<PPath> path_ptr_vec(corr_vec_.size());
    for(int i = 0; i < corr_vec_.size(); ++i)
    {
      path_ptr_vec[i] = Dijkstra::get_shortest_path(graph_, corr_vec_[i]);
    }

    for(int i = 0; i < path_ptr_vec.size(); ++i)
    {
      go_through_path(path_ptr_vec[i], corr_vec_[i].get_total_flow());
    }
  }

  bool step();

  int get_iteration_step() const
  {
    return iteration_step_;
  }

  double get_psi() const
  {
    return psi_;
  }

  double get_max_psi_and_linear_aprx() const
  {
    return max_psi_and_linear_apprx_;
  }

  double get_border_acc_val() const
  {
    return border_acc_val_;
  }

private:
  Graph& graph_;
  const CorrespondenceVec& corr_vec_;
  int iteration_step_;
  double gamma_;
  double relative_eps_;

  double max_psi_and_linear_apprx_;
  double psi_start_;
  double border_acc_val_;
  double psi_;
  double psi_linear_aprx_;
  bool end_flg_;

  void increase_iteration_step()
  {
    gamma_ = 1./(1. + double(iteration_step_));
    ++iteration_step_;
  }

  void walk_through_graph();

  void go_through_path(PPath path_ptr, double corr_flow);

  void revert_graph();
};


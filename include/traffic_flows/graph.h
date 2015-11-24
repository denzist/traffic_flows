#pragma once

#include <vector>
#include <memory>
#include <boost/utility.hpp>
#include <math.h>
#include <utility>
#include <map>
#include <limits>
#include <iostream>

class EdgeInfo
{
public:
  EdgeInfo(){}

  EdgeInfo(
    double flow,
    double max_flow,
    double time_cost,
    double gamma = 1.):
  flow_(flow),
  max_flow_(max_flow),
  time_cost_(time_cost),
  mu_(0.25),
  inv_mu_(1./mu_),
  gamma_(gamma)
  {
    update_cost();
  }

  double get_flow() const
  {
    return flow_;
  }

  void set_flow(double flow)
  {
    flow_ = flow;
  }

  double get_cost() const
  {
    return cost_;
  }

private:
  double flow_;
  double prev_flow_;
  double max_flow_;
  double mu_;
  double inv_mu_;
  double gamma_;
  double time_cost_;
  double cost_;
  double prev_cost_;

  //optimizer things
  friend class GraphOptimizer;
  double opt_curr_flow_;
  double opt_prev_flow_;

  void update_cost()
  {
    cost_ = bpr_fun();
  }

  double bpr_fun() const
  {
    return bpr_fun(flow_);
  }

  double bpr_fun(double flow) const
  {
    return time_cost_*(
      1. + gamma_* pow(flow/max_flow_, inv_mu_)
    );
  }

  double bpr_integral_fun(double flow) const
  {
    return flow*time_cost_*(
      1. + gamma_*pow(flow/max_flow_, inv_mu_)*
        (mu_/1.+ mu_)
      );
  }

};

typedef std::pair<ulong, EdgeInfo> Edge;
typedef std::shared_ptr<Edge> PEdge;
typedef std::map<ulong, EdgeInfo> Edges;


typedef std::map<ulong, Edges> Graph;
typedef std::vector<ulong> Path;

typedef std::shared_ptr<Graph> PGraph;
typedef std::shared_ptr<Path> PPath;


class Correspondence;

typedef std::shared_ptr<Correspondence> PCorrespondence;
typedef std::vector<Correspondence> CorrespondenceVec;

class Correspondence
{
public:
  Correspondence(
    ulong start,
    ulong end,
    int total_flow):
  Correspondence(std::pair<const ulong, const ulong>(start, end), total_flow)
  {}

  Correspondence(
    std::pair<const ulong, const ulong>  start_end,
    int total_flow):
  start_end_(start_end),
  total_flow_(total_flow)
  {}

  std::pair<const ulong, const ulong>& get_start_end()
  {
    return start_end_;
  }

  const ulong get_start() const
  {
    return start_end_.first;
  }

  const ulong get_end() const
  {
    return start_end_.second;
  }

  int get_total_flow() const
  {
    return total_flow_;
  }

private:
  std::pair<const ulong, const ulong>  start_end_;
  int total_flow_;
};


void print_graph(const Graph& graph);

void print_path_in_graph(const PPath path_ptr, Graph& graph);
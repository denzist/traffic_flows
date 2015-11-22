#pragma once

#include <vector>
#include <memory>
#include <boost/utility.hpp>
#include <math.h>
#include <utility>
#include <map>
#include <limits>
#include <iostream>


class Vertex;
typedef std::shared_ptr<Vertex> PVertex;

class EdgeInfo
{
public:
  EdgeInfo(){}

  EdgeInfo(
    double flow,
    double max_flow,
    double time_cost):
  flow_(flow),
  max_flow_(max_flow),
  time_cost_(time_cost),
  inv_mu_(4.),
  gamma_(1.)
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
    update_cost();
  }

  double get_cost() const
  {
    return cost_;
  }
private:
  double flow_;
  double max_flow_;
  double inv_mu_;
  double gamma_;
  double time_cost_;
  double cost_;

  void update_cost()
  {
    cost_ = bpr_fun();
  }

  double bpr_fun()
  {
    return time_cost_*(
      1. + gamma_* pow(flow_/max_flow_, inv_mu_)
    );
  }

};

typedef std::pair<PVertex, EdgeInfo> Edge;
typedef std::shared_ptr<Edge> PEdge;
typedef std::map<PVertex, EdgeInfo> EdgeSet;


class Vertex: private boost::noncopyable
{
public:

  static PVertex createPVertex()
  {
    return std::shared_ptr<Vertex>(new Vertex());
  }

  static PVertex createPVertex(const EdgeSet& edges)
  {
    return std::shared_ptr<Vertex>(new Vertex(edges));
  }

  EdgeSet& get_edges()
  {
    return edges_;
  }

  void add_edge(const Edge& edge)
  {
    edges_.insert(edge);
    return;
  }

private:
  Vertex(){}

  Vertex(const EdgeSet edges):
  edges_(edges)
  {}

  EdgeSet edges_;
};

class Correspondence;

typedef std::shared_ptr<Correspondence> PCorrespondence;
typedef std::vector<PCorrespondence> PCorrespondenceVec;


class Correspondence: private boost::noncopyable
{
public:
  static PCorrespondence createPCorrespondence(
   std::pair<PVertex, PVertex>  start_end,
    int total_flow)
  {
    return std::shared_ptr<Correspondence>(new Correspondence(start_end, total_flow));
  }

  std::pair<const PVertex, const PVertex>& get_start_end()
  {
    return start_end_;
  }

  const PVertex get_start() const
  {
    return start_end_.first;
  }

  const PVertex get_end() const
  {
    return start_end_.second;
  }

  int get_total_flow()
  {
    return total_flow_;
  }

private:
  Correspondence(
    std::pair<const PVertex, const PVertex>  start_end,
    int total_flow):
  start_end_(start_end),
  total_flow_(total_flow)
  {}

  std::pair<const PVertex, const PVertex>  start_end_;
  int total_flow_;
};

typedef std::vector<PVertex> Graph;
typedef std::shared_ptr<Graph> PGraph;

typedef std::vector<Correspondence> CorrespondenceVec;

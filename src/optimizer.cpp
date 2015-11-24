#include "traffic_flows/optimizer.h"


PPath Dijkstra::get_shortest_path(const Graph& graph, const ulong start_ptr, const ulong finish_ptr)
{
  //init dijkstra info map of graph
  DijkstraInfoMap info_map;
  VeretexSet vertices;    

  for (auto it = graph.begin(); it != graph.end(); ++it)
  {
    DijkstraInfo info;
    info_map.insert(std::pair<ulong, DijkstraInfo>(it->first, info));
    vertices.insert(it->first);
  }
  info_map[start_ptr].dist_ = 0.;
  
  //starting finding the min path
  auto vertex_it = find_info_with_min_dist(info_map, vertices);

  while(*vertex_it != finish_ptr)
  {
    DijkstraInfo& prev_info = info_map[*vertex_it];
    ulong prev_vertex = *vertex_it;
    const Edges& edges = graph.find(prev_vertex)->second;

    vertices.erase(vertex_it);

    for(auto it_edge = edges.begin(); it_edge != edges.end(); ++it_edge)
    {
      ulong end_vertex = it_edge->first;
      DijkstraInfo& end_vertex_info = info_map[end_vertex];
      double dist = prev_info.dist_ + it_edge->second.get_cost();

      if(dist < end_vertex_info.dist_)
      {
        end_vertex_info.dist_ = dist;
        end_vertex_info.prev_vertex_ = prev_vertex;
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
  ulong prev_vertex = info_map[*vertex_it].prev_vertex_;
  while(prev_vertex != std::numeric_limits<ulong>::max())
  {
    path_ptr->insert(path_ptr->begin(), prev_vertex);
    prev_vertex = info_map[prev_vertex].prev_vertex_;

  }
  return path_ptr;
}

std::set<ulong>::iterator Dijkstra::find_info_with_min_dist(
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


bool GraphOptimizer::step()
{
  walk_through_graph();
  if(end_flg_)
  {
    return end_flg_;
  }
  increase_iteration_step();

  std::vector<PPath> path_ptr_vec(corr_vec_.size());
  for(int i = 0; i < corr_vec_.size(); ++i)
  {
    path_ptr_vec[i] = Dijkstra::get_shortest_path(graph_, corr_vec_[i]);
  }

  //go through every vertex in the paths
  for(int i = 0; i < path_ptr_vec.size(); ++i)
  {
    go_through_path(path_ptr_vec[i], corr_vec_[i].get_total_flow());
  }
  return end_flg_;
}

void GraphOptimizer::walk_through_graph()
{
  if(iteration_step_ == 0)
  {
    for (auto graph_it = graph_.begin(); graph_it != graph_.end(); ++graph_it)
    {
      for(auto edge_it = graph_it->second.begin(); edge_it != graph_it->second.end(); ++edge_it)
      {
        EdgeInfo& edge_info = edge_it->second;
        //save prev_flow_ and prev_cost_
        edge_info.prev_flow_ = edge_info.flow_;
        edge_info.prev_cost_ = edge_info.cost_;
        //update flow using opt vals 
        edge_info.flow_ = (1 - gamma_)*edge_info.opt_prev_flow_ + edge_info.opt_curr_flow_*gamma_;
        edge_info.update_cost();
        //save prev_flow and set zero curr_flow
        edge_info.opt_prev_flow_ = edge_info.flow_;
        edge_info.opt_curr_flow_ = 0.;
      }
    }
  }
  else
  {
    psi_ = 0.;
    psi_linear_aprx_ = 0.;
    for (auto graph_it = graph_.begin(); graph_it != graph_.end(); ++graph_it)
    {
      for(auto edge_it = graph_it->second.begin(); edge_it != graph_it->second.end(); ++edge_it)
      {
        EdgeInfo& edge_info = edge_it->second;
        //save prev_flow_ and prev_cost_
        edge_info.prev_flow_ = edge_info.flow_;
        edge_info.prev_cost_ = edge_info.cost_;
        //calc psi_ using saved opt_prev_flow_ value
        psi_ += edge_info.bpr_integral_fun(edge_info.prev_flow_);
        //cost still doesn't updated, use it for psi_linear_aprx_ calcs          
        psi_linear_aprx_ += edge_info.cost_*(edge_info.opt_curr_flow_ - edge_info.opt_prev_flow_);
        //now we've got all what we need, update cost and flows for next iterations
        edge_info.flow_ = (1 - gamma_)*edge_info.opt_prev_flow_ + edge_info.opt_curr_flow_*gamma_;
        edge_info.update_cost();
        //save prev_flow and set zero curr_flow
        edge_info.opt_prev_flow_ = edge_info.flow_;
        edge_info.opt_curr_flow_ = 0.;
      }
    }
    //max_psi_and_linear_apprx_
    if(iteration_step_ == 1)
    {
      psi_start_ = psi_;
      border_acc_val_ = psi_start_*relative_eps_;
    }
    double psi_and_linear_apprx = psi_ + psi_linear_aprx_;
    if(psi_and_linear_apprx >= max_psi_and_linear_apprx_)
    {
      max_psi_and_linear_apprx_ = psi_and_linear_apprx;
    }
    if(psi_ - max_psi_and_linear_apprx_ < border_acc_val_)
    {
      end_flg_ = true;
      revert_graph();
    }
  }
}

void GraphOptimizer::go_through_path(PPath path_ptr, double corr_flow)
{
  auto start = path_ptr->begin();
  auto end = path_ptr->end();
  --end;
  for (auto it = start; it != end; ++it)
  {
    auto next = it;
    ++next;
    EdgeInfo& edge_info = graph_[*it][*next];
    edge_info.opt_curr_flow_ += corr_flow;
  }
}

void GraphOptimizer::revert_graph()
{
  for (auto graph_it = graph_.begin(); graph_it != graph_.end(); ++graph_it)
    {
      for(auto edge_it = graph_it->second.begin(); edge_it != graph_it->second.end(); ++edge_it)
      {
        EdgeInfo& edge_info = edge_it->second;
        //revert vals
        edge_info.flow_ = edge_info.prev_flow_;
        edge_info.cost_ = edge_info.prev_cost_;

      }
    }
}
#include "graph.h"
#include "grid.h"
#include <boost/graph/dijkstra_shortest_paths_no_color_map.hpp>
#include <boost/graph/visitors.hpp>
#include <stdexcept>
#include <optional>

namespace naex {
namespace grid {

struct GoalReached : public std::exception {};

template <typename Vertex>
class GoalVisitor : public boost::dijkstra_visitor<boost::null_visitor> {
public:
  GoalVisitor(Vertex goal) : goal_(goal) {}
  template <typename Graph>
  void examine_vertex(Vertex u, const Graph &) {
    if (u == goal_) throw GoalReached();
  }
private:
  Vertex goal_;
};

class ShortestPaths {
public:
  ShortestPaths(const Grid &grid, VertexId start,
                std::optional<VertexId> goal = std::nullopt,
                uint8_t neighborhood = 8, const Costs &max_costs_ = Costs(0.0))
      : graph_(grid, neighborhood, max_costs_), edge_costs_(graph_),
        predecessor_(graph_.num_vertices(),
                     std::numeric_limits<VertexId>::max()),
        path_costs_(graph_.num_vertices(),
                    std::numeric_limits<Cost>::infinity()) {
    boost::typed_identity_property_map<VertexId> index_map;
    try {
      if (goal) {
        boost::dijkstra_shortest_paths_no_color_map(
            graph_, start, predecessor_.data(), path_costs_.data(), edge_costs_,
            index_map, std::less<Cost>(), boost::closed_plus<Cost>(),
            std::numeric_limits<Cost>::infinity(), Cost(0.),
            GoalVisitor<VertexId>(*goal));
      } else {
        boost::dijkstra_shortest_paths_no_color_map(
            graph_, start, predecessor_.data(), path_costs_.data(), edge_costs_,
            index_map, std::less<Cost>(), boost::closed_plus<Cost>(),
            std::numeric_limits<Cost>::infinity(), Cost(0.),
            boost::dijkstra_visitor<boost::null_visitor>());
      }
    } catch (const GoalReached &) {
      // Goal reached, early exit
    }
  }

  const std::vector<VertexId> &predecessors() const { return predecessor_; }
  const std::vector<Cost> &pathCosts() const { return path_costs_; }

  const VertexId &predecessor(VertexId v) const { return predecessor_[v]; }
  const Cost &pathCost(VertexId v) const { return path_costs_[v]; }

protected:
  Graph graph_;
  EdgeCosts edge_costs_;
  std::vector<VertexId> predecessor_;
  std::vector<Cost> path_costs_;
};

} // namespace grid
} // namespace naex
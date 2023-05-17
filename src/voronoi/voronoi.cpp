#include "voronoi/voronoi.hpp"

#include <glog/logging.h>

#include <algorithm>
#include <boost/polygon/voronoi.hpp>
#include <queue>
#include <unordered_map>
#include <unordered_set>

#include "shared/util/timer.h"

using point = boost::polygon::point_data<double>;
using voronoi_diagram = boost::polygon::voronoi_diagram<double>;

DEFINE_double(edge_threshold, 0.25, "Threshold for pruning edges");
DEFINE_double(midline_lookahead, 4, "Lookahead distance for midline");
DEFINE_double(scale, 100, "Scale factor for voronoi diagram");

DEFINE_double(off_graph_multiplier, 10, "Multiplier for off graph edges");

namespace voronoi {

CumulativeFunctionTimer voronoi_timer_("UpdatePointCloud");

bool kDebug = true;

float PointDistance(const boost::polygon::voronoi_edge<double>& edge,
                    const Eigen::Vector2f& point) {
  Eigen::Vector2f s(edge.vertex0()->x(), edge.vertex0()->y());
  Eigen::Vector2f e(edge.vertex1()->x(), edge.vertex1()->y());

  Eigen::Vector2f v = e - s;
  Eigen::Vector2f w = point - s;

  float c1 = w.dot(v);
  float c2 = v.dot(v);

  if (c1 <= 0)
    return (w).norm();
  else if (c2 <= c1)
    return (point - e).norm();
  else {
    float b = c1 / c2;
    Eigen::Vector2f pb = s + b * v;
    return (point - pb).norm();
  }
}

bool CompareVectors(const Eigen::Vector2f& a, const Eigen::Vector2f& b) {
  return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
}

void Voronoi::UpdatePointcloud(const std::vector<Eigen::Vector2f>& pointcloud) {
  CumulativeFunctionTimer::Invocation invoke(&voronoi_timer_);
  pointcloud_ = pointcloud;

  // TODO: sample these constant density instead of every 10
  std::vector<point> vor_points;
  for (size_t i = 0; i < pointcloud_.size(); i += 10) {
    // multiply here bc boost voronoi uses integers internally idk why
    vor_points.emplace_back(pointcloud_[i].x() * FLAGS_scale,
                            pointcloud_[i].y() * FLAGS_scale);
  }

  voronoi_diagram vd;
  boost::polygon::construct_voronoi(vor_points.begin(), vor_points.end(), &vd);

  std::vector<Eigen::Vector2f> points;
  for (const auto& vertex : vd.vertices()) {
    points.emplace_back(vertex.x(), vertex.y());
  }
  sort(points.begin(), points.end(), CompareVectors);
  PruneEdges(points, vd.edges());
  voronoi_vertices_ = points;

  UpdateMidline();
}

void Voronoi::PruneEdges(
    const std::vector<Eigen::Vector2f>& vor_points,
    const std::vector<boost::polygon::voronoi_edge<double>>& vor_edges) {
  Eigen::MatrixXf edges =
      Eigen::MatrixXf::Zero(vor_points.size(), vor_points.size());
  std::vector<bool> pruned_vertices(vor_points.size(), true);
  int count = 0;
  // (loop is parallelizable)
  for (const auto& edge : vor_edges) {
    if (edge.is_primary() && edge.is_finite()) {
      if (edge.is_curved()) {
        printf("curved edge\n");
        continue;
      }

      // this loop could be parallelized too
      bool prune = false;
      float min_clearance = 1000;
      for (const auto& point : pointcloud_) {
        float clearance =
            PointDistance(edge, point * FLAGS_scale) / FLAGS_scale;
        if (clearance < min_clearance) min_clearance = clearance;
        if (clearance < FLAGS_edge_threshold) {
          prune = true;
          count++;
          break;
        }
      }

      if (!prune) {
        int u = std::lower_bound(
                    vor_points.begin(), vor_points.end(),
                    Eigen::Vector2f(edge.vertex0()->x(), edge.vertex0()->y()),
                    CompareVectors) -
                vor_points.begin();
        int v = std::lower_bound(
                    vor_points.begin(), vor_points.end(),
                    Eigen::Vector2f(edge.vertex1()->x(), edge.vertex1()->y()),
                    CompareVectors) -
                vor_points.begin();
        edges(u, v) = min_clearance;
        edges(v, u) = min_clearance;
        pruned_vertices[u] = false;
        pruned_vertices[v] = false;
      }
    }
  }
  LOG_IF(INFO, kDebug) << "pruned " << count << " edges out of "
                       << vor_edges.size();

  pruned_voronoi_edges_ = edges;
  pruned_voronoi_vertices_ = pruned_vertices;
}

int Voronoi::FindStartVertex() const {
  int best = -1;
  float best_dist = 1e6;
  for (size_t i = 1; i < voronoi_vertices_.size(); i++) {
    if (pruned_voronoi_vertices_[i]) continue;
    if (pruned_voronoi_edges_.row(i).sum() == 0) continue;
    float dist = voronoi_vertices_[i].norm();  // optimize for closest to car
    if (dist < best_dist) {
      best = i;
      best_dist = dist;
    }
  }

  return best;
}

// TODO: improve goal point selection (gap follower algorithm maybe?)
Eigen::Vector2f Voronoi::FindGoalPoint() const {
  if (pointcloud_.empty()) return {FLAGS_midline_lookahead, 0};
  // only look in front of the car (this is pretty bad)
  static const size_t start_i = 180;
  static const size_t end_i = 900;
  std::vector<float> disparities(pointcloud_.size());
  float maxd = 0;
  int maxi = 0;
  for (size_t i = start_i; i < end_i; i++) {
    disparities[i] = (pointcloud_[i + 1] - pointcloud_[i]).norm();
    if (disparities[i] > maxd) {
      maxd = disparities[i];
      maxi = i;
    }
  }
  return (pointcloud_[maxi] + pointcloud_[maxi + 1]) / 2;
}

void Voronoi::UpdateMidline() {
  int best = FindStartVertex();
  if (best == -1) {
    LOG(WARNING) << "no unpruned vertices";
    midline_.clear();
    return;
  }

  goal_ = FindGoalPoint();
  std::vector<Eigen::Vector2f> midline;
  std::priority_queue<std::pair<float, int>> q;
  std::unordered_map<int, float> costs;
  std::unordered_map<int, int> parents;

  q.emplace(0, best);
  costs[best] = 0;
  parents[best] = best;

  while (!q.empty()) {
    int cnode = q.top().second;
    q.pop();

    Eigen::Vector2f cpos = voronoi_vertices_[cnode] / FLAGS_scale;

    if (cnode == -1) {
      // don't include the off-graph goal in the midline
      cnode = parents[cnode];
      while (cnode != best) {
        midline.push_back(voronoi_vertices_[cnode] / FLAGS_scale);
        cnode = parents[cnode];
      }
      std::reverse(midline.begin(), midline.end());
      break;
    }

    // make neighbors
    std::vector<int> neighbors;
    for (size_t i = 0; i < voronoi_vertices_.size(); i++) {
      if (i == (size_t)cnode) continue;
      if (pruned_voronoi_edges_(cnode, i) == 0) continue;
      neighbors.push_back(i);
    }
    // -1 represents goal, always in neighbors just has a high cost to go to
    neighbors.push_back(-1);

    for (const int n_idx : neighbors) {
      Eigen::Vector2f npos =
          n_idx == -1 ? goal_ : voronoi_vertices_[n_idx] / FLAGS_scale;
      float dist =
          (cpos - npos).norm() * (n_idx == -1 ? FLAGS_off_graph_multiplier : 1);
      float g = costs[cnode] + dist;
      if (costs.find(n_idx) == costs.end() || g < costs[n_idx]) {
        float h = (npos - goal_).norm();
        float f = g + h;
        costs[n_idx] = g;
        parents[n_idx] = cnode;
        q.emplace(-f, n_idx);
      }
    }
  }

  midline_.clear();
  if (midline.size() == 0) return;
  float t = 0;
  midline_.push_back(midline[0]);
  for (size_t i = 1; i < midline.size(); i++) {
    t += (midline[i] - midline[i - 1]).norm();
    if (t > FLAGS_midline_lookahead) break;
    midline_.push_back(midline[i]);
  }

  LOG_IF(INFO, kDebug) << "midline size: " << midline_.size();
}

}  // namespace voronoi

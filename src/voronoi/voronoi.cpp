#include "voronoi/voronoi.hpp"

#include <glog/logging.h>

#include <algorithm>
#include <boost/polygon/voronoi.hpp>
#include <unordered_set>

#include "shared/util/timer.h"

using point = boost::polygon::point_data<double>;
using voronoi_diagram = boost::polygon::voronoi_diagram<double>;

namespace voronoi {

DEFINE_double(edge_threshold, 0.25, "Threshold for pruning edges");
DEFINE_double(midline_lookahead, 4, "Lookahead distance for midline");
DEFINE_double(scale, 100, "Scale factor for voronoi diagram");

CumulativeFunctionTimer voronoi_timer_("point_cloud_update");

bool kDebug = false;

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
  for (size_t i = 0; i < pointcloud_.size(); i += 10)
    // multiply here bc boost voronoi uses integers internally idk why
    vor_points.emplace_back(pointcloud_[i].x() * FLAGS_scale,
                            pointcloud_[i].y() * FLAGS_scale);

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

void Voronoi::UpdateMidline() {
  // find the unpruned point closest to (0, 0), which is car's location
  int best = -1;
  float best_dist = 1e6;
  for (size_t i = 1; i < voronoi_vertices_.size(); i++) {
    if (pruned_voronoi_vertices_[i]) continue;
    float dist = voronoi_vertices_[i].norm();
    if (dist < best_dist) {
      best = i;
      best_dist = dist;
    }
  }

  if (best == -1) {
    LOG(WARNING) << "no unpruned vertices";
    midline_.clear();
    return;
  }

  // greedily pick the edge with the highest clearance until the cumulative
  // length of the edges is greater than the lookahead distance
  float total_length = 0;
  std::vector<Eigen::Vector2f> midline;
  midline.push_back(voronoi_vertices_[best] / FLAGS_scale);
  std::unordered_set<int> visited;
  visited.insert(best);
  // TODO: don't revisit edges !!!!
  while (total_length < FLAGS_midline_lookahead) {
    float best_clearance = 0;
    int best_edge = -1;
    for (size_t i = 0; i < voronoi_vertices_.size(); i++) {
      // force path forward for a little bit
      if (total_length == 0 &&
          voronoi_vertices_[i].x() < voronoi_vertices_[best].x())
        continue;
      if (visited.find(i) != visited.end()) continue;  // don't revisit
      if (pruned_voronoi_edges_(best, i) > best_clearance) {
        best_clearance = pruned_voronoi_edges_(best, i);
        best_edge = i;
      }
    }
    if (best_edge == -1) {
      LOG_IF(INFO, kDebug) << "no more edges";
      break;
    }
    visited.insert(best_edge);
    total_length +=
        (voronoi_vertices_[best] - voronoi_vertices_[best_edge]).norm() /
        FLAGS_scale;
    midline.push_back(voronoi_vertices_[best_edge] / FLAGS_scale);
    best = best_edge;
  }
  LOG_IF(INFO, kDebug) << "midline length: " << total_length;

  midline_ = midline;
}

}  // namespace voronoi

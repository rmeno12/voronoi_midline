#include "voronoi/voronoi.hpp"

#include <glog/logging.h>

#include <algorithm>
#include <boost/polygon/voronoi.hpp>

#include "shared/util/timer.h"

using point = boost::polygon::point_data<double>;
using voronoi_diagram = boost::polygon::voronoi_diagram<double>;

namespace voronoi {

DEFINE_double(edge_threshold, 0.25, "Threshold for pruning edges");
DEFINE_double(midline_lookahead, 4, "Lookahead distance for midline");
DEFINE_double(scale, 100, "Scale factor for voronoi diagram");

CumulativeFunctionTimer voronoi_timer_("point_cloud_update");

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
  Eigen::MatrixXf edges(points.size(), points.size());
  PruneEdges(pointcloud_, points, vd.edges(), edges);
  voronoi_vertices_ = points;
  pruned_voronoi_edges_ = edges;

  UpdateMidline();
}

void Voronoi::UpdateMidline() {
  // find the point closest to (0, 0), which is car's location in local frame
  // int best = 0;
  // float best_dist = voronoi_vertices_[0].norm();
  // for (size_t i = 1; i < voronoi_vertices_.size(); i++) {
  //   float dist = voronoi_vertices_[i].norm();
  //   if (dist < best_dist) {
  //     best = i;
  //     best_dist = dist;
  //   }
  // }

  // greedily pick the edge with the highest clearance until the cumulative
  // length of the edges is greater than the lookahead distance
}

void PruneEdges(
    const std::vector<Eigen::Vector2f>& clearance_points,
    const std::vector<Eigen::Vector2f>& vor_points,
    const std::vector<boost::polygon::voronoi_edge<double>>& vor_edges,
    Eigen::MatrixXf& edges) {
  // (loop is parallelizable)
  for (const auto& edge : vor_edges) {
    if (edge.is_primary() && edge.is_finite()) {
      if (edge.is_curved()) {
        printf("curved edge\n");
        continue;
      }

      // this loop could be parallelized too
      bool prune = false;
      for (const auto& point : clearance_points) {
        if (PointDistance(edge, point) < FLAGS_scale * FLAGS_edge_threshold) {
          prune = true;
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
        edges(u, v) = 1;
        edges(v, u) = 1;
      }
    }
  }
}

}  // namespace voronoi

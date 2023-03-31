#include "voronoi/voronoi.hpp"

#include <glog/logging.h>

#include <algorithm>
#include <boost/polygon/voronoi.hpp>

#include "shared/util/timer.h"

using point = boost::polygon::point_data<double>;
using voronoi_diagram = boost::polygon::voronoi_diagram<double>;

namespace voronoi {

const float THRESHOLD = 0.25;
CumulativeFunctionTimer voronoi_timer_("point_cloud_update");

float PointDistance(const boost::polygon::voronoi_edge<double>& edge,
                    const point& point) {
  Eigen::Vector2f s(edge.vertex0()->x(), edge.vertex0()->y());
  Eigen::Vector2f e(edge.vertex1()->x(), edge.vertex1()->y());
  Eigen::Vector2f p(point.x(), point.y());

  Eigen::Vector2f v = e - s;
  Eigen::Vector2f w = p - s;

  float c1 = w.dot(v);
  float c2 = v.dot(v);

  if (c1 <= 0)
    return (w).norm();
  else if (c2 <= c1)
    return (p - e).norm();
  else {
    float b = c1 / c2;
    Eigen::Vector2f pb = s + b * v;
    return (p - pb).norm();
  }
}

bool CompareVectors(const Eigen::Vector2f& a, const Eigen::Vector2f& b) {
  return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
}

void Voronoi::UpdatePointcloud(const std::vector<Eigen::Vector2f>& pointcloud) {
  CumulativeFunctionTimer::Invocation invoke(&voronoi_timer_);
  pointcloud_ = pointcloud;

  std::vector<point> vor_points;
  for (size_t i = 0; i < pointcloud_.size(); i += 10)
    vor_points.emplace_back(pointcloud_[i].x(), pointcloud_[i].y());

  voronoi_diagram vd;
  boost::polygon::construct_voronoi(vor_points.begin(), vor_points.end(), &vd);
  CHECK_EQ(vor_points.size(), vd.num_vertices());

  // prune edges (loop is parallelizable)
  std::vector<Eigen::Vector2f> points;
  for (const auto& vertex : vd.vertices()) {
    points.emplace_back(vertex.x(), vertex.y());
  }
  sort(points.begin(), points.end(), CompareVectors);
  Eigen::MatrixXf edges(points.size(), points.size());
  for (const auto& edge : vd.edges()) {
    if (edge.is_primary() && edge.is_finite()) {
      if (edge.is_curved()) {
        printf("curved edge\n");
        continue;
      }

      // this loop could be parallelized too
      bool prune = false;
      for (const auto& point : vor_points) {
        if (PointDistance(edge, point) < THRESHOLD) {
          prune = true;
          break;
        }
      }

      if (!prune) {
        int u = std::lower_bound(
                    points.begin(), points.end(),
                    Eigen::Vector2f(edge.vertex0()->x(), edge.vertex0()->y()),
                    CompareVectors) -
                points.begin();
        int v = std::lower_bound(
                    points.begin(), points.end(),
                    Eigen::Vector2f(edge.vertex1()->x(), edge.vertex1()->y()),
                    CompareVectors) -
                points.begin();
        edges(u, v) = 1;
        edges(v, u) = 1;
      }
    }
  }
}

}  // namespace voronoi

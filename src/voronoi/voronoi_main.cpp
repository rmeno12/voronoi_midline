#include <gflags/gflags.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <eigen3/Eigen/Dense>
#include <memory>
#include <vector>

#include "voronoi/voronoi.hpp"

using Eigen::Vector2f;

namespace {

static const Vector2f kLaserOffset(0.2, 0.0);

std::unique_ptr<voronoi::Voronoi> vor_;

}  // namespace

void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  static std::vector<Vector2f> cloud;
  vor_->UpdatePointcloud(cloud);
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);

  vor_ = std::make_unique<voronoi::Voronoi>();

  ros::init(argc, argv, "voronoi");
  ros::NodeHandle nh;

  ros::Subscriber laser_sub = nh.subscribe("/scan", 1, LaserCallback);

  ros::spin();
}

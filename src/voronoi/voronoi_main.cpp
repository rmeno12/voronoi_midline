#include <gflags/gflags.h>
#include <nav_msgs/Path.h>
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

ros::Publisher midline_pub_;

}  // namespace

void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  static std::vector<Vector2f> cloud;

  // TODO: only do this update if the robot has moved enough
  cloud.clear();
  for (size_t i = 0; i < msg->ranges.size(); i++) {
    float angle = msg->angle_min + i * msg->angle_increment;
    float range = msg->ranges[i];
    if (range < msg->range_min || range >= msg->range_max) continue;
    cloud.emplace_back(range * cos(angle) + kLaserOffset.x(),
                       range * sin(angle) + kLaserOffset.y());
  }
  vor_->UpdatePointcloud(cloud);

  auto midline = vor_->GetMidline();
  nav_msgs::Path path;
  path.header.frame_id = "base_link";
  path.header.stamp = ros::Time::now();
  for (const auto& point : midline) {
    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = point.x();
    pose.pose.position.y = point.y();
    path.poses.push_back(pose);
  }
  midline_pub_.publish(path);
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);

  vor_ = std::make_unique<voronoi::Voronoi>();

  ros::init(argc, argv, "voronoi");
  ros::NodeHandle nh;

  midline_pub_ = nh.advertise<nav_msgs::Path>("/midline", 1);

  ros::Subscriber laser_sub = nh.subscribe("/scan", 1, LaserCallback);

  ros::spin();
}

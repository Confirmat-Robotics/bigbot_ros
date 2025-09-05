/*********************************************************************
 *
 * Software License Agreement 
 *
 *  Copyright (c) 2025 Edward Hage
 *  All rights reserved.
 *

 *********************************************************************/

#ifndef NAV2_STRAIGHTLINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_
#define NAV2_STRAIGHTLINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_

#include <string>
#include <memory>
#include <memory>
#include <vector>
#include <array>

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "GenerateForwardPath.hpp"

namespace nav2_bigbot_planner
{

class BigbotPlanner : public nav2_core::GlobalPlanner
{
  public:
    BigbotPlanner() = default;
    ~BigbotPlanner() = default;

    // plugin configure
    void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
      std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    // plugin cleanup
    void cleanup() override;

    // plugin activate
    void activate() override;

    // plugin deactivate
    void deactivate() override;

    // This method creates path for given start and goal pose.
    nav_msgs::msg::Path createPlan(
      const geometry_msgs::msg::PoseStamped & start,
      const geometry_msgs::msg::PoseStamped & goal,
      std::function<bool()> cancel_checker) override;

    nav_msgs::msg::Path generateNavMsg(
      const std::vector<std::shared_ptr<void>>& rawPath, 
      const geometry_msgs::msg::PoseStamped& startPose);

    void logPathInfo(
      const std::vector<std::shared_ptr<void>>& rawPath,
      const geometry_msgs::msg::PoseStamped& start,
      const geometry_msgs::msg::PoseStamped& goal,
      const Eigen::Vector2d& P1,
      const Eigen::Vector2d& V1,
      const Eigen::Vector2d& P2,
      const Eigen::Vector2d& V2);

  private:
    // TF buffer
    std::shared_ptr<tf2_ros::Buffer> tf_;

    // node ptr
    nav2_util::LifecycleNode::SharedPtr node_;

    // Global Costmap
    nav2_costmap_2d::Costmap2D * costmap_;

    // The global frame of the costmap
    std::string global_frame_, name_;

    geometry_msgs::msg::PoseStamped stampPose(const geometry_msgs::msg::Pose& pose);
    geometry_msgs::msg::PoseStamped getPoseRotate(
      const geometry_msgs::msg::PoseStamped& startPose, 
      double angleRot, 
      double radius, 
      bool forward);
    geometry_msgs::msg::PoseStamped getPoseTranslate(
      const geometry_msgs::msg::PoseStamped& startPose,
      double distance);
    Eigen::Vector2d quaternionToVector(const geometry_msgs::msg::Quaternion& q);

    double interpolation_resolution_;
    double minimum_radius_;
    bool strictly_forward_;
  };

}  // namespace nav2_bigbot_planner

#endif  // NAV2_STRAIGHTLINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_

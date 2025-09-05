/*********************************************************************
 *
 * Software License Agreement 
 *
 *  Copyright (c) 2025 Edward Hage
 *  All rights reserved.
 *
 *********************************************************************/

#include <cmath>
#include <string>
#include <memory>
#include <vector>
#include <array>
#include <Eigen/Dense>
#include "nav2_util/node_utils.hpp"
#include "nav2_bigbot_planner/bigbot_path_planner.hpp"
#include "nav2_bigbot_planner/GenerateForwardPath.hpp"
#include "nav2_bigbot_planner/Rotate.hpp"
#include "nav2_bigbot_planner/Translate.hpp"

namespace nav2_bigbot_planner
{

void BigbotPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, 
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  // Parameter initialization
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(
      0.1));
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".minimum_radius", rclcpp::ParameterValue(
      0.5));
  node_->get_parameter(name_ + ".minimum_radius", minimum_radius_);

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".strictly_forward", rclcpp::ParameterValue(
      false));
  node_->get_parameter(name_ + ".strictly_forward", strictly_forward_);

}

void BigbotPlanner::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type BigbotPlanner",
    name_.c_str());
}

void BigbotPlanner::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type BigbotPlanner",
    name_.c_str());
}

void BigbotPlanner::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type BigbotPlanner",
    name_.c_str());
}

geometry_msgs::msg::PoseStamped BigbotPlanner::getPoseTranslate(const geometry_msgs::msg::PoseStamped& startPose, double distance) {
  Eigen::Vector3d vec(startPose.pose.position.x, startPose.pose.position.y, startPose.pose.position.z);
  Eigen::Quaterniond rot(
    startPose.pose.orientation.w, startPose.pose.orientation.x,
    startPose.pose.orientation.y, startPose.pose.orientation.z);

  Eigen::Vector3d newVec = vec + rot * Eigen::Vector3d(distance, 0, 0);

  geometry_msgs::msg::Pose outputPose;
  outputPose.position.x = newVec.x();
  outputPose.position.y = newVec.y();
  outputPose.position.z = newVec.z();
  outputPose.orientation = startPose.pose.orientation;

  return stampPose(outputPose);
}

geometry_msgs::msg::PoseStamped BigbotPlanner::getPoseRotate(
    const geometry_msgs::msg::PoseStamped& startPose, double angleRot, double radius, bool forward) {
    Eigen::Quaterniond rotLeft(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond rotRight(Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitZ()));
  
    auto p = startPose.pose.position;
    auto q = startPose.pose.orientation;
    Eigen::Vector3d vec(p.x, p.y, p.z);
    Eigen::Quaterniond rot(q.w, q.x, q.y, q.z);

    Eigen::Quaterniond rotAngle;
    if (forward) {
        rotAngle = Eigen::AngleAxisd(angleRot, Eigen::Vector3d::UnitZ());
    } else {
        rotAngle = Eigen::AngleAxisd(-angleRot, Eigen::Vector3d::UnitZ());
    }

    Eigen::Quaterniond rr = (angleRot >= 0) ? rot * rotLeft : rot * rotRight;
    Eigen::Vector3d CP = rr * Eigen::Vector3d(radius, 0.0, 0.0);
    Eigen::Vector3d newVec = CP + vec + rotAngle * (-CP);

    Eigen::Quaterniond newRotAngle = rot * rotAngle;
    geometry_msgs::msg::Pose outputPose;
    outputPose.position.x = newVec.x();
    outputPose.position.y = newVec.y();
    outputPose.position.z = newVec.z();
    outputPose.orientation.x = newRotAngle.x();
    outputPose.orientation.y = newRotAngle.y();
    outputPose.orientation.z = newRotAngle.z();
    outputPose.orientation.w = newRotAngle.w();

    return stampPose(outputPose);
}

// from GenerateForwardPath 
geometry_msgs::msg::PoseStamped BigbotPlanner::stampPose(const geometry_msgs::msg::Pose& pose) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = global_frame_;
    ps.header.stamp = node_->now();
    ps.pose = pose;
    return ps;
}

nav_msgs::msg::Path BigbotPlanner::generateNavMsg(
    const std::vector<std::shared_ptr<void>>& rawPath, const geometry_msgs::msg::PoseStamped& startPose)
{
    nav_msgs::msg::Path outPath;
    outPath.header.frame_id = startPose.header.frame_id;
    outPath.header.stamp = node_->now();

    geometry_msgs::msg::PoseStamped currentPose = startPose;
    outPath.poses.push_back(currentPose);

    for (const auto& element : rawPath) {
        if (typeid(*std::static_pointer_cast<Translate>(element)) == typeid(Translate)) {
            auto translate = std::static_pointer_cast<Translate>(element);
            int numPoses = static_cast<int>(std::ceil(translate->getLength() / interpolation_resolution_));
            numPoses = std::max(numPoses, 2);
            double deltaLength = translate->getLength() / numPoses; //was (numPoses - 1)

            for (int i = 0; i < numPoses; ++i) {
                currentPose = getPoseTranslate(
                    currentPose, translate->isForward() ? deltaLength : -deltaLength);
                outPath.poses.push_back(currentPose);
            }
        } else if (typeid(*std::static_pointer_cast<Rotate>(element)) == typeid(Rotate)) {
            auto rotate = std::static_pointer_cast<Rotate>(element);
            double length = rotate->getRadius() * rotate->getAngle();
            int numPoses = static_cast<int>(std::ceil(std::abs(length) / interpolation_resolution_));
            numPoses = std::max(numPoses, 2);
            double deltaAngle = rotate->getAngle() / numPoses; // was (numPoses - 1)
            bool forward = rotate->isForward();
            for (int i = 0; i < numPoses; ++i) {
                if (forward == true) {
                    currentPose = getPoseRotate(
                        currentPose, deltaAngle, rotate->getRadius(), true);
                } else {
                    currentPose = getPoseRotate(
                        currentPose, -deltaAngle, rotate->getRadius(), false);
                }
                outPath.poses.push_back(currentPose);
            }
        }
    }

    return outPath;
}

Eigen::Vector2d BigbotPlanner::quaternionToVector(const geometry_msgs::msg::Quaternion& q)
{
    // Convert quaternion to yaw
    double yaw = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    );

    return Eigen::Vector2d(std::cos(yaw), std::sin(yaw));
}

void BigbotPlanner::logPathInfo(
    const std::vector<std::shared_ptr<void>>& rawPath,
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal,
    const Eigen::Vector2d& P1,
    const Eigen::Vector2d& V1,
    const Eigen::Vector2d& P2,
    const Eigen::Vector2d& V2)
{
  
    // Log the start pose
    RCLCPP_INFO(node_->get_logger(),
                "Start Pose:\n  Position: [x: %f, y: %f, z: %f]\n  Orientation: [x: %f, y: %f, z: %f, w: %f]",
                start.pose.position.x, start.pose.position.y, start.pose.position.z,
                start.pose.orientation.x, start.pose.orientation.y, start.pose.orientation.z, start.pose.orientation.w);

    // Log the goal pose
    RCLCPP_INFO(node_->get_logger(),
                "Goal Pose:\n  Position: [x: %f, y: %f, z: %f]\n  Orientation: [x: %f, y: %f, z: %f, w: %f]",
                goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
                goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w);

    RCLCPP_INFO(node_->get_logger(), "Input Vectors:");
    RCLCPP_INFO(node_->get_logger(), "  P1: [%f, %f]", P1.x(), P1.y());
    RCLCPP_INFO(node_->get_logger(), "  V1: [%f, %f]", V1.x(), V1.y());
    RCLCPP_INFO(node_->get_logger(), "  P2: [%f, %f]", P2.x(), P2.y());
    RCLCPP_INFO(node_->get_logger(), "  V2: [%f, %f]", V2.x(), V2.y());

    // Iterate through rawPath to log details
    for (size_t i = 0; i < rawPath.size(); ++i) {
        RCLCPP_INFO(node_->get_logger(), "Path element [%zu]:", i);

        // Handle Translate elements
        if (typeid(*std::static_pointer_cast<Translate>(rawPath[i])) == typeid(Translate)) {
            auto translate = std::static_pointer_cast<Translate>(rawPath[i]);
            RCLCPP_INFO(node_->get_logger(),
                        "  Type: Translate\n"
                        "  Length: %f meters\n"
                        "  Forward: %s",
                        translate->getLength(), translate->isForward() ? "true" : "false");
        }
        // Handle Rotate elements
        else if (typeid(*std::static_pointer_cast<Rotate>(rawPath[i])) == typeid(Rotate)) {
            auto rotate = std::static_pointer_cast<Rotate>(rawPath[i]);
            RCLCPP_INFO(node_->get_logger(),
                        "  Type: Rotate\n"
                        "  Angle: %f radians\n"
                        "  Radius: %f meters\n"
                        "  Forward: %s",
                        rotate->getAngle(), rotate->getRadius(), rotate->isForward() ? "true" : "false");
        }
        // Handle unknown types
        else {
            RCLCPP_WARN(node_->get_logger(), "  Unknown path element type.");
        }
    }
}

nav_msgs::msg::Path BigbotPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> /*cancel_checker*/)
{
    nav_msgs::msg::Path global_path;

    // Checking if the goal and start state is in the global frame
    if (start.header.frame_id != global_frame_) {
      RCLCPP_ERROR(
        node_->get_logger(), "Planner will only except start position from %s frame",
        global_frame_.c_str());
      return global_path;
    }

    if (goal.header.frame_id != global_frame_) {
      RCLCPP_INFO(
        node_->get_logger(), "Planner will only except goal position from %s frame",
        global_frame_.c_str());
      return global_path;
    }

    // Convert start and goal poses to Eigen vectors
    Eigen::Vector2d P1(start.pose.position.x, start.pose.position.y);
    Eigen::Vector2d P2(goal.pose.position.x, goal.pose.position.y);

    // Calculate the heading vector for the start and goal poses
    Eigen::Vector2d V1 = quaternionToVector(start.pose.orientation);
    Eigen::Vector2d V2 = quaternionToVector(goal.pose.orientation);

    // Initialize the GenerateForwardPath class
    GenerateForwardPath path_generator(P1, V1, P2, V2, minimum_radius_, strictly_forward_);
    // Get the shortest path
    auto [rawPath, total_length] = path_generator.GetShortestPath();

    RCLCPP_INFO(
    node_->get_logger(), "Found path with length %2.3f", total_length);

    //logPathInfo(rawPath, start, goal, P1, V1, P2, V2);

    global_path = generateNavMsg(rawPath,start);
    return global_path;
  }

// normally, this below is outside the namespace
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_bigbot_planner::BigbotPlanner, nav2_core::GlobalPlanner)
}  // namespace nav2_bigbot_planner
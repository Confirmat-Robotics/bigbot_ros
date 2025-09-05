// File: src/gps_fix_model.cpp

#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components.hh>
#include <gz/sim/components/SphericalCoordinates.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/math/SphericalCoordinates.hh>
#include <gz/math/Helpers.hh>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <Eigen/Core>
#include <memory>
#include <string>
#include <thread>
#include <cmath>

namespace gps_fix_simulator
{

  class GPSFixSimulatorPlugin : public gz::sim::System,
                                public gz::sim::ISystemConfigure,
                                public gz::sim::ISystemPreUpdate
  {
  public:
    GPSFixSimulatorPlugin() = default;

    void Configure(const gz::sim::Entity &entity,
                  const std::shared_ptr<const sdf::Element> &sdf,
                  gz::sim::EntityComponentManager &ecm,
                  gz::sim::EventManager &) override
    {
      this->linkName = sdf->Get<std::string>("link_name", "base_link").first;
      this->childFrame = sdf->Get<std::string>("child_frame", "TF_POI").first;
      this->parentFrame = sdf->Get<std::string>("parent_frame", "TF_ECEF").first;
      this->enuFrame = sdf->Get<std::string>("enu_frame", "FP_ENU").first;

      // 1) Preferred: plugin attached at model scope -> use the passed entity
      gz::sim::Model model(entity);
      if (model.Valid(ecm)) {
        this->modelEntity = entity;
        this->modelName = model.Name(ecm);
        this->link = gz::sim::Link(model.LinkByName(ecm, this->linkName));
      } 
      else 
      {
        // 2) World-scoped fallback: allow optional <model_name> in SDF
        const std::string modelNameParam =
            sdf->Get<std::string>("model_name", "").first;

        if (!modelNameParam.empty()) {
          auto modelEntities = ecm.EntitiesByComponents(gz::sim::components::Model());
          for (auto me : modelEntities) {
            gz::sim::Model m(me);
            if (m.Name(ecm) == modelNameParam) {
              this->modelEntity = me;
              this->modelName = m.Name(ecm);
              this->link = gz::sim::Link(m.LinkByName(ecm, this->linkName));
              break;
            }
          }
        }
      }

      if (this->modelEntity == gz::sim::kNullEntity || !this->link.Valid(ecm)) {
        std::cerr << "[GPSFixSimulatorPlugin] Failed to find model/link. "
                  << "model='" << (this->modelName.empty() ? "<unknown>" : this->modelName)
                  << "' link='" << this->linkName << "'\n";
        return;
      }

      std::cout << "[GPSFixSimulatorPlugin] Attached to model '" << this->modelName
                << "', link '" << this->linkName << "'\n";

      // Initialize ROS2
      this->ros_context = std::make_shared<rclcpp::Context>();
      this->ros_context->init(0, nullptr);
      rclcpp::NodeOptions options;
      options.context(this->ros_context);
      this->node = std::make_shared<rclcpp::Node>("gps_fix_simulator_node", options);
      this->tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this->node);

      this->ros_thread = std::thread([this]() { rclcpp::spin(this->node); });
    }

    void PreUpdate(const gz::sim::UpdateInfo &info,
                  gz::sim::EntityComponentManager &ecm) override
    {
      if (!this->link.Valid(ecm)) return;

      // Retrieve the pose of the link relative to the world using WorldInertialPose()
      auto worldPoseOpt = this->link.WorldInertialPose(ecm);
      if (!worldPoseOpt.has_value())
      {
          std::cerr << "[GPSFixSimulatorPlugin] Failed to retrieve world pose for link '" << this->linkName << "'" << std::endl;
          return;
      }

      const auto &worldPose = *worldPoseOpt; // Dereference the optional to access the Pose3 object

      auto worldEntity = gz::sim::worldEntity(ecm);
      auto sphericalComp = ecm.Component<gz::sim::components::SphericalCoordinates>(worldEntity);
      if (!sphericalComp) return;
      auto spherical = sphericalComp->Data();

      // Broadcast dynamic transform at a specified rate
      static double tf_broadcast_rate = 5.0; // Default 5 Hz
      static rclcpp::Time last_tf_broadcast_time(0, 0, RCL_ROS_TIME);

      static bool rate_initialized = false;
      if (!rate_initialized)
      {
        if (this->node->has_parameter("tf_broadcast_rate"))
          tf_broadcast_rate = this->node->get_parameter("tf_broadcast_rate").as_double();
        else if (this->node->declare_parameter("tf_broadcast_rate", tf_broadcast_rate))
          tf_broadcast_rate = this->node->get_parameter("tf_broadcast_rate").as_double();
        rate_initialized = true;
      }

      //auto now = this->node->get_clock()->now();  // this is "real" ros time
      auto now = rclcpp::Time(info.simTime.count(), RCL_ROS_TIME); // gazebo sim time
      if ((now - last_tf_broadcast_time).seconds() >= (1.0 / tf_broadcast_rate))
      {
        // Get transform H: TF_ECEF -> T_ENU0
        geometry_msgs::msg::Transform H_msg = GetEnu(spherical);
        tf2::Transform H;
        tf2::fromMsg(H_msg, H);

        // Get transform K: ENU0 -> base_link (robot pose in local frame)
        geometry_msgs::msg::Transform K_msg;
        K_msg.translation.x = worldPose.Pos().X();
        K_msg.translation.y = worldPose.Pos().Y();
        K_msg.translation.z = worldPose.Pos().Z();
        K_msg.rotation.x = worldPose.Rot().X();
        K_msg.rotation.y = worldPose.Rot().Y();
        K_msg.rotation.z = worldPose.Rot().Z();
        K_msg.rotation.w = worldPose.Rot().W();

        tf2::Transform K;
        tf2::fromMsg(K_msg, K);

        geometry_msgs::msg::TransformStamped tf_msg_rel;
        tf_msg_rel.header.stamp = now;
        tf_msg_rel.header.frame_id = "map";
        tf_msg_rel.child_frame_id = "odom";
        tf_msg_rel.transform = K_msg;
        this->tf_broadcaster->sendTransform(tf_msg_rel);

        // Compose transform: TF_ECEF -> base_link = H * K
        tf2::Transform T_ecef_base = H * K;
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = now;
        tf_msg.header.frame_id = this->parentFrame;
        tf_msg.child_frame_id = this->childFrame;
        tf_msg.transform = tf2::toMsg(T_ecef_base);
        this->tf_broadcaster->sendTransform(tf_msg);
        last_tf_broadcast_time = now;
      }

      // Broadcast static ENU transform every 5 seconds
      static tf2_ros::StaticTransformBroadcaster static_broadcaster(this->node);
      static rclcpp::Time last_static_broadcast_time(0, 0, RCL_ROS_TIME);

      if ((now - last_static_broadcast_time).seconds() >= 5.0)
      {
        // Get transform H: TF_ECEF -> T_ENU0
        geometry_msgs::msg::Transform H_msg = GetEnu(spherical);
        geometry_msgs::msg::TransformStamped static_tf;
        static_tf.header.stamp = now;
        static_tf.header.frame_id = this->parentFrame;
        static_tf.child_frame_id = this->enuFrame;
        static_tf.transform = H_msg;
        static_broadcaster.sendTransform(static_tf);

        last_static_broadcast_time = now;
      }
    }

    ~GPSFixSimulatorPlugin()
    {
      if (this->ros_thread.joinable())
      {
        this->ros_context->shutdown("plugin shutdown");
        this->ros_thread.join();
      }
    }

  private:
    gz::sim::Entity modelEntity{gz::sim::kNullEntity};
    gz::sim::Link link;
    std::string modelName;

    std::shared_ptr<rclcpp::Context> ros_context;
    std::shared_ptr<rclcpp::Node> node;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    std::thread ros_thread;

    std::string linkName;
    std::string childFrame;
    std::string parentFrame;
    std::string enuFrame;
    bool enuPublished = false;

    // Helper to compute ENU → ECEF rotation matrix
    gz::math::Matrix3d ComputeENUtoECEF(double lat_rad, double lon_rad)
    {
      double sin_lat = std::sin(lat_rad);
      double cos_lat = std::cos(lat_rad);
      double sin_lon = std::sin(lon_rad);
      double cos_lon = std::cos(lon_rad);

      gz::math::Vector3d xEast(-sin_lon, cos_lon, 0.0);
      gz::math::Vector3d yNorth(-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat);
      gz::math::Vector3d zUp(cos_lat * cos_lon, cos_lat * sin_lon, sin_lat);

      gz::math::Matrix3d R;
      R.SetCol(0, xEast);
      R.SetCol(1, yNorth);
      R.SetCol(2, zUp);

      return R;
    }

    geometry_msgs::msg::Transform GetEnu(const gz::math::SphericalCoordinates &spherical)
    {
      gz::math::Vector3d enu_origin_local(0, 0, 0);
      auto enu_origin_ecef = spherical.PositionTransform(enu_origin_local, gz::math::SphericalCoordinates::LOCAL2, gz::math::SphericalCoordinates::ECEF);
      auto lonLatAlt = spherical.PositionTransform(enu_origin_local, gz::math::SphericalCoordinates::LOCAL2, gz::math::SphericalCoordinates::SPHERICAL);
      double lon_rad = lonLatAlt.Y();  // longitude in radians, x=eAST
      double lat_rad = lonLatAlt.X();  // latitude in radians

      gz::math::Matrix3d R_enu_ecef = ComputeENUtoECEF(lat_rad, lon_rad);
      gz::math::Quaterniond q_enu_ecef(R_enu_ecef);

      geometry_msgs::msg::Transform H_msg;
      H_msg.translation.x = enu_origin_ecef.X();
      H_msg.translation.y = enu_origin_ecef.Y();
      H_msg.translation.z = enu_origin_ecef.Z();
      H_msg.rotation.x = q_enu_ecef.X();
      H_msg.rotation.y = q_enu_ecef.Y();
      H_msg.rotation.z = q_enu_ecef.Z();
      H_msg.rotation.w = q_enu_ecef.W();
      return H_msg;
    }

  }; 
} // namespace gps_fix_simulator

GZ_ADD_PLUGIN(gps_fix_simulator::GPSFixSimulatorPlugin,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(gps_fix_simulator::GPSFixSimulatorPlugin, "gps_fix_simulator")

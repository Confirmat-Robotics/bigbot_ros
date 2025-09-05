#include <functional>
#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"
#include "bigbot_base/roboclaw.h"
#include "bigbot_interfaces/msg/drive_feedback.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
// namespace Bigbot to be added later

class DriverNode : public rclcpp::Node
{
public:
  DriverNode()
  : Node("motor_driver")
  {
    got_data = false;
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&DriverNode::joy_callback, this, _1));
    vel_limit_ = this->create_subscription<std_msgs::msg::Float32>( "/velocity_limit", 10, std::bind(&DriverNode::vel_limit_callback, this, _1));
    drivefeedbackpub_ = this->create_publisher<bigbot_interfaces::msg::DriveFeedback>("/driveinfo", 10);
    this->connect_drive();
    timer_ = this->create_wall_timer(500ms, std::bind(&DriverNode::no_input_callback, this));
    pubtimer_ = this->create_wall_timer(100ms, std::bind(&DriverNode::publish_driveinfo_callback, this));
  }

private:
  struct roboclaw *rc;
  bool got_data;
  uint8_t address=0x80; //address of roboclaw unit
  const char* connection = "/dev/ttyDRIVE";
  int baudrate=115200;
  float diameter_wheel = 0.3; //[m]

  void connect_drive()
  {
	  //initialize at supplied terminal at specified baudrate
    
	  rc=roboclaw_init(connection, baudrate);
    if( rc == NULL )
	  {
		  RCLCPP_ERROR(this->get_logger(), "unable to initialize drive");
		  exit(1);
	  }
    RCLCPP_INFO(this->get_logger(), "Initializing driver");
  }
  
  int puls_speed(float meter_per_sec)
  {
    int puls_sp = (int) (meter_per_sec /diameter_wheel/3.141592*4.0*8192.0);
    return puls_sp;
  }

  void joy_callback(const geometry_msgs::msg::Twist::SharedPtr msg) 
  {
    //  RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->linear.x);
    got_data = true;
    int speed_rechts, speed_links;
    float distance_wheels = 0.47;
    
    float speedR = msg->linear.x + distance_wheels/2.0* msg->angular.z;
    float speedL = msg->linear.x - distance_wheels/2.0* msg->angular.z;
    speed_rechts =  puls_speed(speedR);
    speed_links = puls_speed(speedL);
    
    float limit_rechts = 1.0;
    float limit_links = 1.0;
    if (abs(speedR) > vel_limit_value) {
      limit_rechts = abs(vel_limit_value/speedR);
    }
    if (abs(speedL) > vel_limit_value) {
      limit_links = abs(vel_limit_value/speedL);
    }
    float minlimit = std::min(limit_links,limit_rechts);
    if (minlimit < 1.0) {
      speed_rechts = (int) (minlimit * speed_rechts);
      speed_links = (int) (minlimit * speed_links);
      RCLCPP_INFO(this->get_logger(), "Restricting speed");
    }
    // rc, address, rechts, links
    if (roboclaw_speed_m1m2(rc, address, speed_rechts, speed_links) != ROBOCLAW_OK )
    // vel_limit_value to encoder (!?)
    {
      RCLCPP_INFO(this->get_logger(), "Cannot set speed !");
    }
    else 
    {
      RCLCPP_INFO(this->get_logger(), "setting speed (R): %d (L): %d", (int)speed_rechts, (int) speed_links);
    }
  }

  void vel_limit_callback (const std_msgs::msg::Float32::SharedPtr msg) 
  {
    vel_limit_value = msg->data;
  }

  void publish_driveinfo_callback() 
  {
    auto msg = bigbot_interfaces::msg::DriveFeedback();
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "bigbot_base";
    int32_t enc_m1, enc_m2;
    if(roboclaw_encoders(rc, address, &enc_m1, &enc_m2) != ROBOCLAW_OK)
    {
      RCLCPP_WARN(this->get_logger(), "I cannot get encoder-data");
    }
    else 
    {
      msg.encoder[0] = enc_m1; 
      msg.encoder[1] = enc_m2; 
    }

    int16_t mvoltage, lvoltage;
    if(roboclaw_main_battery_voltage(rc, address, &mvoltage) != ROBOCLAW_OK) 
    {
      RCLCPP_WARN(this->get_logger(), "I cannot get main voltage data");
    }
    else 
    {
      msg.main_battery_voltage = mvoltage/10.0f;
    }

    if(roboclaw_logic_battery_voltage(rc, address, &lvoltage) != ROBOCLAW_OK) 
    {
      RCLCPP_WARN(this->get_logger(), "I cannot get logic voltage data");
    }
    else 
    {
      msg.logic_battery_voltage = lvoltage/10.0f;
    }
    int16_t cur_m1, cur_m2;
    if(roboclaw_currents(rc, address, &cur_m1, &cur_m2) != ROBOCLAW_OK)  
    {
      RCLCPP_WARN(this->get_logger(), "I cannot get current data");
    }
    else 
    {
      msg.current[0] = cur_m1/100.0;
      msg.current[1] = cur_m2/100.0;
    }

    int16_t dc_m1, dc_m2;
    if(roboclaw_dutycycles(rc, address, &dc_m1, &dc_m2) != ROBOCLAW_OK)
    {
      RCLCPP_WARN(this->get_logger(), "I cannot get dutycycle data");
    }
    else 
    {
      msg.dutycycle[0] = dc_m1 / 327.67; 
      msg.dutycycle[1] = dc_m2 / 327.67; 
    }

    int32_t sp_m1, sp_m2;
    if(roboclaw_speeds(rc, address, &sp_m1, &sp_m2) != ROBOCLAW_OK)
    {
      RCLCPP_WARN(this->get_logger(), "I cannot get speed data");
    }
    else 
    {
      msg.speed[0] = sp_m1; 
      msg.speed[1] = sp_m2; 
      msg.speedmpsec[0] = sp_m1* diameter_wheel *3.141592/4.0/8192.0; 
      msg.speedmpsec[1] = sp_m2* diameter_wheel *3.141592/4.0/8192.0; 
    }

    int16_t temp, tempalt;
    if(roboclaw_temperature(rc, address, &temp) != ROBOCLAW_OK)
    {
      RCLCPP_WARN(this->get_logger(), "I cannot get temperature data");
    }
    else 
    {
      msg.temperature_drive = temp / 10.0;
    }
    if(roboclaw_temperature_alt(rc, address, &tempalt) != ROBOCLAW_OK)
    {
      RCLCPP_WARN(this->get_logger(), "I cannot get alt temperature data");
    }
    else 
    {
      msg.temperature_drive_2nd = tempalt / 10.0;
    }
  
    int32_t sp_err_m1, sp_err_m2;
    if(roboclaw_speederrors(rc, address, &sp_err_m1, &sp_err_m2) != ROBOCLAW_OK)
    {
      RCLCPP_WARN(this->get_logger(), "I cannot get speederror data");
    }
    else 
    {
      msg.speederror[0] = sp_err_m1; 
      msg.speederror[1] = sp_err_m2; 
    }
    int32_t p_err_m1, p_err_m2;
    if(roboclaw_poserrors(rc, address, &p_err_m1, &p_err_m2) != ROBOCLAW_OK)
    {
      RCLCPP_WARN(this->get_logger(), "I cannot get poserror data");
    }
    else 
    {
      msg.poserror[0] = p_err_m1; 
      msg.poserror[1] = p_err_m2; 
    }
    drivefeedbackpub_->publish(msg);
  }
  
  void myexit() 
  {
    if(roboclaw_close(rc) != ROBOCLAW_OK) RCLCPP_ERROR(this->get_logger(), "unable to shutdown roboclaw cleanly");
  }

  void no_input_callback()
  {
    if (got_data == false)
    {
      roboclaw_duty_m1m2(rc, address, 0, 0);
      /*RCLCPP_ERROR(this->get_logger(), No input so standstill); */
    }
    got_data = false;
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr pubtimer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vel_limit_;
  rclcpp::Publisher<bigbot_interfaces::msg::DriveFeedback>::SharedPtr drivefeedbackpub_;
  float vel_limit_value = 9999999.9;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DriverNode>());
  rclcpp::shutdown();
  return 0;
}

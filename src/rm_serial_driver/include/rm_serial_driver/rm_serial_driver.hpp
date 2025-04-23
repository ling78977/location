// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <mutex>
#include <nav_msgs/nav_msgs/msg/detail/odometry__struct.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/wait_result_kind.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <rclcpp/timer.hpp>
// C++ system
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include "rm_serial_driver/packet.hpp"

// #include "auto_aim_interfaces/msg/target.hpp"

namespace rm_serial_driver
{
class RMSerialDriver : public rclcpp::Node
{
public:
  explicit RMSerialDriver(const rclcpp::NodeOptions & options);

  ~RMSerialDriver() override;

private:
  void getParams();

  void receiveData();

  // void sendData(auto_aim_interfaces::msg::Target::SharedPtr msg);

  void reopenPort();

  void twistSubCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void sendTwistCallback();

  void setParam(const rclcpp::Parameter & param);


  // Serial port
  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;


  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  // Param client to set detect_colr
  using ResultFuturePtr = std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;
  bool initial_set_param_ = false;
  rclcpp::AsyncParametersClient::SharedPtr detector_param_client_;
  ResultFuturePtr set_param_future_;

  std::mutex twist_mutes_;
  SendPacket twist_;

  float vx_=0;
  float vy_=0;
  float vyaw_=0;
  rclcpp::TimerBase::SharedPtr send_timer_;



  // Aimimg point receiving from serial port for visualization
  visualization_msgs::msg::Marker aiming_point_;

  std::thread receive_thread_;
};
}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

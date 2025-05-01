#include <cmath>
#include <cstdlib>
#include <functional>
// #include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nav_msgs/msg/path.hpp>
#include <random>
#include <rclcpp/duration.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <message_filters/subscriber.h>
#include <nav_msgs/msg/path.hpp>
#include <rcl/publisher.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/timer.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/create_timer_ros.h>
// #include <tf2_ros/message_filter.h>
#include <tf2/convert.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_ros/static_transform_broadcaster.h>
class RobotController : public rclcpp::Node {
public:
  RobotController(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("Robot_controller", options) {

    this->declare_parameter<bool>("debug_simulation", true);
    this->declare_parameter<double>("forward_look_distance", 0.15);
    this->declare_parameter<double>("max_vx", 0.8);
    this->declare_parameter<double>("max_vyaw", 1.2);
    this->get_parameter_or<bool>("debug_simulation", debug_simu_, true);
    this->get_parameter_or<double>("forward_look_distance",
                                   forward_look_distance_, 0.15);
    this->get_parameter_or<double>("max_vx", max_vx_, 0.6);
    this->get_parameter_or<double>("max_vyaw", max_vyaw_, 1.2);

    RCLCPP_INFO(this->get_logger(), "start control...");

    global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/global_path", 10,
        std::bind(&RobotController::global_path_callback, this,
                  std::placeholders::_1));
    follow_path_pub_ =
        this->create_publisher<nav_msgs::msg::Path>("follow_path", 10);
    vel_cmd_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
    tfb = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    auto control_period_ms =
        std::chrono::milliseconds(static_cast<int64_t>(1000.0 / 33.33));
    control_timer_ = rclcpp::create_timer(
        this, this->get_clock(), control_period_ms,
        std::bind(&RobotController::control_callback, this));

    if (debug_simu_) {
      tf_publisher_ =
          std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
      rclcpp::Time now;
      geometry_msgs::msg::TransformStamped t;

      t.header.stamp = now;
      t.header.frame_id = "base_link";
      t.child_frame_id = "body";

      t.transform.translation.x = 0.1;
      t.transform.translation.y = 0;
      t.transform.translation.z = 0;
      tf2::Quaternion q;
      q.setRPY(0, 0, 0);
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();

      tf_publisher_->sendTransform(t);
      auto odemetry_period_ms =
          std::chrono::milliseconds(static_cast<int64_t>(1000.0 / 10.0));
      debug_odometry_timer_ = rclcpp::create_timer(
          this, this->get_clock(), odemetry_period_ms,
          std::bind(&RobotController::odometry_callback, this));
      debug_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
          "/cmd_vel", 10,
          std::bind(&RobotController::debug_cmd_sub_callback, this,
                    std::placeholders::_1));
    }
  }

private:
  nav_msgs::msg::Path global_path_;
  nav_msgs::msg::Path follow_path_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr follow_path_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr debug_cmd_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr debug_odometry_timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfb;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_publisher_;
  geometry_msgs::msg::Twist last_twist;
  rclcpp::Time last_time_;
  bool debug_simu_ = true;
  bool is_first_twist = true;
  double debug_x_ = -1;
  double debug_y_ = -2;
  double debug_yaw_ = 1.50924 * 2;
  double forward_look_distance_ = 0.15; // 前视距离

  bool start_follow_ = false;
  double max_vx_ = 0.8;
  double max_vyaw_ = 1.2;

private:
  void global_path_callback(nav_msgs::msg::Path::SharedPtr msg) {

    if (start_follow_) {
      // 正在跟随当中
      return;
    }
    global_path_ = *msg;
    follow_path_ = global_path_;
    for(int i=0;i<(int)follow_path_.poses.size();i++){
      follow_path_.poses[i].header.frame_id="camera_init";
    }
    start_follow_ = true;
  }
  void control_callback() {
    // std::cout<<"1111\n";
    if (!start_follow_) {

      return;
    }
    geometry_msgs::msg::TransformStamped trans;
    auto now = rclcpp::Clock().now();
    try {
      trans = tf2_buffer_->lookupTransform("camera_init", "base_link",
                                           tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      return;
    }
    geometry_msgs::msg::PoseStamped point_pose;
    // 取路径末尾的点，判断距离
    while (true) {
      if (follow_path_.poses.empty()) {
        // 如果已经空了
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0;
        cmd.linear.y = 0;
        cmd.angular.z = 0;
        vel_cmd_pub_->publish(cmd);
        start_follow_ = false;
        return;
      }
      point_pose = follow_path_.poses[follow_path_.poses.size() - 1];
      auto path_point = point_pose.pose.position;
      auto dis =
          std::sqrt(std::pow(path_point.x - trans.transform.translation.x, 2) +
                    std::pow(path_point.y - trans.transform.translation.y, 2));
      if (dis < forward_look_distance_) {
        follow_path_.poses.pop_back();
      } else {
        break;
      }
    }
    // 计算方向
    tf2::Quaternion q;
    tf2::fromMsg(trans.transform.rotation, q);
    double r, p, yaw;
    tf2::Matrix3x3(q).getRPY(r, p, yaw);
    double line_vector_x =
        point_pose.pose.position.x - trans.transform.translation.x;
    double line_vector_y =
        point_pose.pose.position.y - trans.transform.translation.y;
    // 计算连线向量的角度
    double phi = atan2(line_vector_y, line_vector_x);
    // 计算夹角
    double angle = phi - yaw;
    // 标准化到 [-π, π] 范围内
    if (angle > M_PI) {
      angle -= 2 * M_PI;
    } else if (angle < -M_PI) {
      angle += 2 * M_PI;
    }
    geometry_msgs::msg::PoseStamped point_in_base_link;
    point_pose.header.frame_id = "camera_init";
    try {
      point_in_base_link = tf2_buffer_->transform(point_pose, "base_link");
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      return;
    }

    if (abs(angle) > DEG2RAD(60)) {
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = 0;
      cmd.linear.y = 0;
      cmd.angular.z = std::abs(angle) > max_vyaw_
                          ? (angle / std::abs(angle) * max_vyaw_)
                          : angle;
      vel_cmd_pub_->publish(cmd);
    } else {
      double y = point_in_base_link.pose.position.y;
      auto L =
          std::sqrt(std::pow(line_vector_x, 2) + std::pow(line_vector_y, 2));
      double a = max_vx_ / L;
      if (std::abs(a * 2 * y / L) > max_vyaw_) {
        a = max_vyaw_ * L / (2 * std::abs(y));
      }
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = a * L;
      cmd.linear.y = 0;
      cmd.angular.z = a * 2 * y / L;
      vel_cmd_pub_->publish(cmd);
    }
    follow_path_.header.frame_id = "camera_init";
    follow_path_.header.stamp = this->get_clock()->now();
    follow_path_pub_->publish(follow_path_);
  }
  void odometry_callback() {
    geometry_msgs::msg::TransformStamped tf;
    // 设置时间戳
    tf.header.stamp = this->now();
    // 设置父级坐标id
    tf.header.frame_id = "camera_init";
    // 设置子级坐标id
    tf.child_frame_id = "base_link";
    // 偏移量
    tf.transform.translation.x = debug_x_;
    tf.transform.translation.y = debug_y_;
    tf.transform.translation.z = 0.0;
    tf2::Quaternion qtn;
    // 传入roll、pitch、yaw，然后会得到对应四元数
    qtn.setRPY(0, 0, debug_yaw_);
    qtn.normalize();
    tf.transform.rotation.x = qtn.x();
    tf.transform.rotation.w = qtn.w();
    tf.transform.rotation.y = qtn.y();
    tf.transform.rotation.z = qtn.z();
    tfb->sendTransform(tf);
  }
  void debug_cmd_sub_callback(geometry_msgs::msg::Twist::SharedPtr msg) {
    std::default_random_engine generator;
    std::normal_distribution<double> linear_noise(
        0.0, 0.02); // 线速度噪声，均值为0，标准差为0.1
    std::normal_distribution<double> angular_noise(
        0.0, 0.02); // 角速度噪声，均值为0，标准差为0.05
    if (is_first_twist) {
      // 创建随机数生成器

      msg->linear.x += linear_noise(generator);
      msg->linear.y += linear_noise(generator);
      msg->angular.z += angular_noise(generator);
      last_twist = *msg;
      is_first_twist = false;
      last_time_ = this->get_clock()->now();
      return;
    }
    msg->linear.x += linear_noise(generator);
    msg->linear.y += linear_noise(generator);
    msg->angular.z += angular_noise(generator);

    // double avg_vx = (last_twist.linear.x + msg->linear.x) / 2;
    // double avg_vy = (last_twist.linear.y + msg->linear.y) / 2;
    // double avg_vyaw = (last_twist.angular.z + msg->angular.z) / 2;
    double dt = (this->get_clock()->now() - last_time_).seconds();
    last_time_ = this->get_clock()->now();
    if(dt>0.5){
      return;
    }

    last_twist = *msg;
    debug_x_ = debug_x_ + (msg->linear.x * std::cos(debug_yaw_) -
                           msg->linear.y * std::sin(debug_yaw_)) *
                              dt;
    debug_y_ = debug_y_ + (msg->linear.x * sin(debug_yaw_) +
                           msg->linear.y * cos(debug_yaw_)) *
                              dt;
    debug_yaw_ = debug_yaw_ + msg->angular.z * dt;
    // debug_x_ =
    //     debug_x_ +
    //     (avg_vx * std::cos(debug_yaw_) - avg_vy * std::sin(debug_yaw_)) * dt;
    // debug_y_ =
    //     debug_y_ + (avg_vx * sin(debug_yaw_) + avg_vy * cos(debug_yaw_)) *
    //     dt;
    // debug_yaw_ = debug_yaw_ + avg_vyaw * dt;
  }
  inline double DEG2RAD(double deg) { return deg * (M_PI / 180.0); }
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotController>());
  if (rclcpp::ok())
    rclcpp::shutdown();

  return 0;
}
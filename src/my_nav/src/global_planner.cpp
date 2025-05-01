
// #include "/opt/ros/humble/include/nav2_util/occ_grid_values.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "ccpp_planer.hpp"
#include <functional>
#include <iostream>
// #include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
// #include <std_srvs/srv/detail/trigger__struct.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <utility>
#include <vector>

class GloabalPlaner : public rclcpp::Node {

public:
  GloabalPlaner(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("global_planner", options) {

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/global_path", 10);
    map_suber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10,
        std::bind(&GloabalPlaner::mapSuberCallback, this,
                  std::placeholders::_1));
    global_planner_service_ = this->create_service<std_srvs::srv::Trigger>(
        "/start_plan", std::bind(&GloabalPlaner::global_planner_callback, this,
                                 std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "global_lanner init success ...");
  }

private:
  void mapSuberCallback(nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    if (!init_map_) {
      map_ = *msg;
      init_map_ = true;
    }
  }
  bool get_voxed_map() {
    if (!init_map_) {
      return false;
    }
    // std::cout<<"11\n";
    voxed_map_.clear();
    voxed_map_.resize(5, std::vector<int>(10, 1));
    list_.clear();
    list_.resize(5, std::vector<std::pair<double, double>>(10));
    // std::cout<<"22\n";
    for (int row = 0; row < 5; row++) {
      for (int col = 0; col < 10; col++) {
        // std::cout<<"33\n";
        double wx = x2 + 0.35 * col + 0.175;
        double wy = y2 + 0.35 * row + 0.175;
        list_[row][col].first = wx;
        list_[row][col].second = wy;
        unsigned mx, my;

        if (!worldToMap(wx - 0.175, wy - 0.175, mx, my)) {

          return false;
        }

        int num = 0;
        for (int i = 0; i < 7; i++) {
          for (int j = 0; j < 7; j++) {
            if (map_.data[map_.info.width * (my + j) + mx + i] == 0) {
              num++;
            }
          }
        }
        // std::cout<<"44\n";
        if (num > 25) {
          voxed_map_[row][col] = 0; // 可通行
        } else {
          voxed_map_[row][col] = 1; // 不可通行
        }
      }
    }
    // voxed_map_[2][3]=voxed_map_[2][4]=voxed_map_[3][3]=voxed_map_[3][4]=1;
    return true;
  }

  bool worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my) {
    if (wx < map_.info.origin.position.x || wy < map_.info.origin.position.y) {
      return false;
    }

    mx = static_cast<int>(
        std::round((wx - map_.info.origin.position.x) / map_.info.resolution));
    my = static_cast<int>(
        std::round((wy - map_.info.origin.position.y) / map_.info.resolution));

    if (mx < map_.info.width && my < map_.info.height) {
      return true;
    }

    RCLCPP_ERROR(this->get_logger(),
                 "worldToMap failed: mx,my: %d,%d, size_x,size_y: %d,%d", mx,
                 my, map_.info.width, map_.info.height);

    return false;
  }
  void
  global_planner_callback(std_srvs::srv::Trigger::Request::ConstSharedPtr req,
                          std_srvs::srv::Trigger::Response::SharedPtr res) {
    if (!get_voxed_map()) {
      res->success = false;
      res->message = "get voxed disabled.";
      return;
    }
    RCLCPP_INFO(this->get_logger(), "start plan");
    ccpp_planer_.updateMap(voxed_map_);
    std::vector<astar::Point> res_path = ccpp_planer_.DFSFindPath();
    res->success = true;
    res->message = "planer suceese.";
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id="camera_init";
    for (int i = res_path.size() - 1; i > 0; i--) {
      auto p1 = list_[res_path[i].row][res_path[i].col];
      auto p2 = list_[res_path[i - 1].row][res_path[i - 1].col];
      pose_msg.pose.position.x = list_[res_path[i].row][res_path[i].col].first;
      pose_msg.pose.position.y = list_[res_path[i].row][res_path[i].col].second;
      path_.poses.push_back(pose_msg);
      // 计算两点之间的距离
      double distance = std::sqrt(std::pow(p2.first - p1.first, 2) +
                                  std::pow(p2.second - p1.second, 2));

      // 计算采样点数量
      int num_samples = static_cast<int>(distance / 0.05);

      // 计算单位向量
      double dx = (p2.first - p1.first) / distance;
      double dy = (p2.second - p1.second) / distance;
      for (int j = 0; j <= num_samples; j++) {
        std::pair<double, double> p;
        p.first = p1.first + j * 0.05 * dx;
        p.second = p1.second + j * 0.05 * dy;
        pose_msg.pose.position.x = p.first;
        pose_msg.pose.position.y = p.second;
        path_.poses.push_back(pose_msg);
      }
    }
    pose_msg.pose.position.x = list_[res_path[0].row][res_path[0].col].first;
    pose_msg.pose.position.y = list_[res_path[0].row][res_path[0].col].second;
    path_.poses.push_back(pose_msg);

    path_.header.frame_id = "camera_init";
    path_.header.stamp = this->now();
    path_pub_->publish(path_);
    path_.poses.clear();
  }

private:
  nav_msgs::msg::Path path_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  std::vector<std::vector<std::pair<double, double>>> list_;
  std::vector<std::vector<int>> voxed_map_; // 碰撞后的栅格地图
  bool init_map_ = false;
  nav_msgs::msg::OccupancyGrid map_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_suber_;
  int grid_num_ = 7;
  double x1 = 3.5, y1 = -1.75;
  double x2 = 0, y2 = -1.75;
  double x3 = 0, y3 = 0;
  double x4 = 3.5, y4 = 0;
  CCPPPlaner ccpp_planer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr global_planner_service_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GloabalPlaner>());
  if (rclcpp::ok())
    rclcpp::shutdown();

  return 0;
}
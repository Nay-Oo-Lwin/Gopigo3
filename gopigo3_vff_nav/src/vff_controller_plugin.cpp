#include<algorithm>
#include<cmath>
#include<limits>
#include<memory>
#include<string>
#include<mutex>

#include"rclcpp/rclcpp.hpp"
#include"geometry_msgs/msg/twist.hpp"
#include"sensor_msgs/msg/laser_scan.hpp"
#include"geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/exceptions.h"
#include "laser_geometry/laser_geometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include"visualization_msgs/msg/marker_array.hpp"

#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace vff_controller{
    class VFFController: public nav2_core::Controller
    {
        public:
        VFFController() = default;

        void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, 
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros){
    auto node = parent.lock();
    node_ = parent;
    tf_ = tf;
    name_=name;
    costmap_ros_ = costmap_ros;
    logger_= node->get_logger();
    clock_= node->get_clock();

    node->declare_parameter(name_ + ".k_att", 2.0);
    node->declare_parameter(name_ + ".k_rep", 0.10);
    node->declare_parameter(name_ + ".obstacle_range", 0.5);
    node->declare_parameter(name_ + ".max_rep_force", 2.5);
    node->declare_parameter(name_ + ".max_linear_speed", 0.12);
    node->declare_parameter(name_ + ".max_angular_speed", 1.0);
    node->declare_parameter(name_ + ".k_turn", 1.5);
    node->declare_parameter(name_ + ".goal_tolerance", 0.15);
    node->declare_parameter(name_+ ".lookahead_dist",0.25);

    node->get_parameter(name_ + ".k_att", k_att_);
    node->get_parameter(name_ + ".k_rep", k_rep_);
    node->get_parameter(name_ + ".obstacle_range", obstacle_range_);
    node->get_parameter(name_ + ".max_rep_force", max_rep_force_);
    node->get_parameter(name_ + ".max_linear_speed", max_linear_speed_);
    node->get_parameter(name_ + ".max_angular_speed", max_angular_speed_);
    node->get_parameter(name_ + ".k_turn", k_turn_);
    node->get_parameter(name_ + ".goal_tolerance", goal_tolerance_);
    node->get_parameter(name_+ ".lookahead_dist",lookahead_dist_);

    scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>("/scan",10,std::bind(&VFFController::scanCallback,this,std::placeholders::_1));
    marker_pub_= node->create_publisher<visualization_msgs::msg::MarkerArray>("/vff_markers_array",10);

  }

  void cleanup() override{
    scan_sub_.reset();
    marker_pub_.reset();
    RCLCPP_INFO(logger_,"Cleaned up VFF controller plugin");
  }

  void activate() override{
    RCLCPP_INFO(logger_," Activated VFF controller plugin");
  }

  void deactivate() override{
    RCLCPP_INFO(logger_,"Deactivated VFF controller plugin");
  }

  void setPlan(const nav_msgs::msg::Path & path) override{
    global_plan_= path;
  }
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override{
    if(speed_limit <= 0.0){
      return;
    }
    if(percentage){
      max_linear_speed_ *= speed_limit / 100.0;
    }
    else{
      max_linear_speed_ = speed_limit;
    }
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity, 
    nav2_core::GoalChecker * goal_checker
  ) override
  {
    (void)velocity;
    (void)goal_checker;
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = clock_->now();
    cmd.header.frame_id = pose.header.frame_id;

    if (global_plan_.poses.empty()) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000, "Global plan is empty");
      return cmd;
    }

    sensor_msgs::msg::LaserScan::SharedPtr scan;
    {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      scan = latest_scan_;
    }

    if (!scan) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000, "Waiting for /scan");
      return cmd;
    }


geometry_msgs::msg::PoseStamped goal_in_base;
bool found_goal = false;

for (auto pose_on_path : global_plan_.poses) {
  pose_on_path.header.stamp = rclcpp::Time(0);

  geometry_msgs::msg::PoseStamped pose_in_base;

  try {
    pose_in_base = tf_->transform(
      pose_on_path,
      "base_link",
      tf2::durationFromSec(0.05));
  } catch (const tf2::TransformException & ex) {
    continue;
  }

  double x = pose_in_base.pose.position.x;
  double y = pose_in_base.pose.position.y;
  double dist = std::hypot(x, y);

  if (x > 0.0 && dist >= lookahead_dist_) {
    goal_in_base = pose_in_base;
    found_goal = true;
    break;
  }
}

if (!found_goal) {
  geometry_msgs::msg::PoseStamped last_pose = global_plan_.poses.back();
  last_pose.header.stamp = rclcpp::Time(0);

  try {
    goal_in_base = tf_->transform(
      last_pose,
      "base_link",
      tf2::durationFromSec(0.1));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "Could not transform path goal to base_link: %s", ex.what());
    return cmd;
  }
}

    double goal_x = goal_in_base.pose.position.x;
    double goal_y = goal_in_base.pose.position.y;
    double goal_dist = std::sqrt(goal_x * goal_x + goal_y * goal_y);

    if (goal_dist < goal_tolerance_) {
      cmd.twist.linear.x = 0.0;
      cmd.twist.angular.z = 0.0;
      return cmd;
    }

    double f_att_x = 0.0;
    double f_att_y = 0.0;

    if (goal_dist > 1e-6) {
      f_att_x = k_att_ * goal_x / goal_dist;
      f_att_y = k_att_ * goal_y / goal_dist;
    }

    sensor_msgs::msg::PointCloud2 cloud;

    try {
      projector_.transformLaserScanToPointCloud(
        "base_link",
        *scan,
        cloud,
        *tf_);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000,
        "Could not transform LaserScan to base_link: %s", ex.what());
      return cmd;
    }

    double f_rep_x = 0.0;
    double f_rep_y = 0.0;

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
      double x = static_cast<double>(*iter_x);
      double y = static_cast<double>(*iter_y);

      if (!std::isfinite(x) || !std::isfinite(y)) {
        continue;
      }

      double d = std::sqrt(x * x + y * y);

      if (d < 1e-6 || d > obstacle_range_) {
        continue;
      }

      double force_mag = k_rep_ / (d * d);
      force_mag = std::min(force_mag, max_rep_force_);

      f_rep_x += -force_mag * x / d;
      f_rep_y += -force_mag * y / d;
    }

    double f_total_x = f_att_x + f_rep_x;
    double f_total_y = f_att_y + f_rep_y;
    RCLCPP_INFO_THROTTLE(
  logger_, *clock_, 1000,
  "goal_in_base=(%.2f, %.2f), f_att=(%.2f, %.2f)",
  goal_x, goal_y, f_att_x, f_att_y);

    visualization_msgs::msg::MarkerArray array;

    array.markers.push_back(makeArrow(1, f_rep_x, f_rep_y, 1.0, 0.0, 0.0, 0.12));
    array.markers.push_back(makeArrow(2, f_total_x, f_total_y, 0.0, 1.0, 0.0, 0.14));
    array.markers.push_back(makeArrow(0, f_att_x, f_att_y, 0.0, 0.0, 1.0, 0.1));

    marker_pub_->publish(array);

    double desired_angle = std::atan2(f_total_y, f_total_x);
    double forward_factor = std::max(0.0, std::cos(desired_angle));

    cmd.twist.linear.x = max_linear_speed_ * forward_factor;
    cmd.twist.angular.z = clamp(
      k_turn_ * desired_angle,
      -max_angular_speed_,
      max_angular_speed_);

    return cmd;
  }

  private:


  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string name_;

  rclcpp::Logger logger_{rclcpp::get_logger("VFFController")};
  rclcpp::Clock::SharedPtr clock_;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  std::mutex scan_mutex_;

  laser_geometry::LaserProjection projector_;
  nav_msgs::msg::Path global_plan_;

  double k_att_;
  double k_rep_;
  double obstacle_range_;
  double max_rep_force_;
  double max_linear_speed_;
  double max_angular_speed_;
  double k_turn_;
  double goal_tolerance_;
  double lookahead_dist_;

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    std::lock_guard<std::mutex> lock(scan_mutex_);
    latest_scan_ = msg;
  }
  double clamp(double value, double min_value, double max_value){
    return std::max(min_value,std::min(value, max_value));
  }
  visualization_msgs::msg::Marker makeArrow(int id, double fx, double fy, float r, float g, float b, double z)
  {

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = rclcpp::Time(0);
    marker.frame_locked = true;

    marker.ns = "vff_forces";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    geometry_msgs::msg::Point start;
    geometry_msgs::msg::Point end;

    start.x = 0.0;
    start.y = 0.0;
    start.z = z;

    double scale = 0.2;

    end.x = scale * fx;
    end.y = scale * fy;
    end.z = z;
    marker.points.push_back(start);
    marker.points.push_back(end);

    marker.scale.x = 0.03;
    marker.scale.y = 0.06;
    marker.scale.z = 0.1;

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;

    marker.lifetime = rclcpp::Duration::from_seconds(0.2);

    return marker;
  }

    };
}

PLUGINLIB_EXPORT_CLASS(vff_controller::VFFController, nav2_core::Controller)
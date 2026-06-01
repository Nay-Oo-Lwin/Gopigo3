#include <cmath>
#include <memory>
#include <vector>
#include <limits>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"

#include "visualization_msgs/msg/marker.hpp"


class FrontierExplorer : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  FrontierExplorer() : Node("frontier_explorer")
  {
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map",
      10,
      std::bind(&FrontierExplorer::mapCallback, this, std::placeholders::_1)
    );

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(
      this,
      "/navigate_to_pose"
    );
    goal_marker_pub_= this->create_publisher<visualization_msgs::msg::Marker>("/exploration_goal_marker",10);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    timer_ = this->create_wall_timer(
      std::chrono::seconds(3),
      std::bind(&FrontierExplorer::explore, this)
    );

    RCLCPP_INFO(this->get_logger(), "Frontier explorer started.");
  }

private:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;

  bool goal_active_ = false;
  std::vector<std::pair<double,double>> blacklisted_goals_;
  std::vector<std::pair<double, double>> visited_goals_;
  double visited_radius_ = 0.8;
  double last_goal_x_ = 0.0;
  double last_goal_y_ = 0.0;
  double min_goal_distance_= 0.6;

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    latest_map_ = msg;
  }

  int index(int x, int y, int width)
  {
    return y * width + x;
  }

  bool isInside(int x, int y, int width, int height)
  {
    return x >= 0 && x < width && y >= 0 && y < height;
  }

  bool isFrontierCell(
    int x,
    int y,
    const nav_msgs::msg::OccupancyGrid & map)
  {
    int width = map.info.width;
    int height = map.info.height;

    int idx = index(x, y, width);

    // Frontier must be known free space
    int value = map.data[idx];
    if (value < 0 || value > 20) {
      return false;
    }

    // Check 8-neighbors
    for (int dy = -1; dy <= 1; dy++) {
      for (int dx = -1; dx <= 1; dx++) {
        if (dx == 0 && dy == 0) {
          continue;
        }

        int nx = x + dx;
        int ny = y + dy;

        if (!isInside(nx, ny, width, height)) {
          continue;
        }

        int nidx = index(nx, ny, width);

        // Unknown neighbor
        if (map.data[nidx] == -1) {
          return true;
        }
      }
    }

    return false;
  }

  bool getRobotPose(double & robot_x, double & robot_y)
  {
    try {
      auto transform = tf_buffer_->lookupTransform(
        "map",
        "base_link",
        tf2::TimePointZero
      );

      robot_x = transform.transform.translation.x;
      robot_y = transform.transform.translation.y;
      return true;
    }
    catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Could not get robot pose: %s",
        ex.what()
      );
      return false;
    }
  }

  void mapToWorld(
    int mx,
    int my,
    const nav_msgs::msg::OccupancyGrid & map,
    double & wx,
    double & wy)
  {
    double resolution = map.info.resolution;

    wx = map.info.origin.position.x + (mx + 0.5) * resolution;
    wy = map.info.origin.position.y + (my + 0.5) * resolution;
  }
  bool isNearVisited(double wx, double wy)
  {
    for (const auto & g : visited_goals_) {
      if (std::hypot(wx - g.first, wy - g.second) < visited_radius_) {
        return true;
      }
    }
  return false;
  } 
  bool isNearBlacklisted(double wx, double wy){
    for ( auto & g : blacklisted_goals_){
      if(std::hypot(wx-g.first,wy-g.second)<0.5){
        return true;
      }
    }
    return false;
  }
  int countUnknownNear(
    int x,
    int y,
    const nav_msgs::msg::OccupancyGrid & map,
    int radius
  ){
    int count = 0;
    int width = map.info.width;
    int height = map.info.height;

    for(int dy = -radius; dy <= radius ; dy++){
      for(int dx = -radius;dx<=radius; dx++){
        int nx = x + dx;
        int ny = y + dy;

        if(!isInside(nx,ny,width,height)){
          continue;
        }
        if(map.data[index(nx,ny,width)]==-1){
          count++;
        }
      }
    }
    return count;
  }

  bool isNearOccupied(
  int x,
  int y,
  const nav_msgs::msg::OccupancyGrid & map,
  int radius)
{
  int width = map.info.width;
  int height = map.info.height;

  for (int dy = -radius; dy <= radius; dy++) {
    for (int dx = -radius; dx <= radius; dx++) {
      int nx = x + dx;
      int ny = y + dy;

      if (!isInside(nx, ny, width, height)) {
        continue;
      }

      int value = map.data[index(nx, ny, width)];

      if (value > 50) {
        return true;
      }
    }
  }

  return false;
}

bool findSafeGoalBehindFrontier(
  int fx,
  int fy,
  const nav_msgs::msg::OccupancyGrid & map,
  int & safe_x,
  int & safe_y)
{
  int width = map.info.width;
  int height = map.info.height;

  double robot_x, robot_y;
  if (!getRobotPose(robot_x, robot_y)) {
    return false;
  }

  double frontier_wx, frontier_wy;
  mapToWorld(fx, fy, map, frontier_wx, frontier_wy);

  double dx = robot_x - frontier_wx;
  double dy = robot_y - frontier_wy;
  double len = std::hypot(dx, dy);

  if (len < 1e-6) {
    return false;
  }

  dx /= len;
  dy /= len;

  double step = map.info.resolution;

  // Try points 0.2m to 0.6m behind the frontier
  for (double back = 0.2; back <= 0.6; back += step) {
    double wx = frontier_wx + dx * back;
    double wy = frontier_wy + dy * back;

    int mx = static_cast<int>((wx - map.info.origin.position.x) / map.info.resolution);
    int my = static_cast<int>((wy - map.info.origin.position.y) / map.info.resolution);

    if (!isInside(mx, my, width, height)) {
      continue;
    }

    int value = map.data[index(mx, my, width)];

    if (value < 0 || value > 20) {
      continue;
    }

    if (isNearOccupied(mx, my, map,3)) {
      continue;
    }

    safe_x = mx;
    safe_y = my;
    return true;
  }

  return false;
}

  bool findNearestFrontier(double & target_x, double & target_y)
  {
    if (!latest_map_) {
      return false;
    }

    const auto & map = *latest_map_;

    int width = map.info.width;
    int height = map.info.height;

    double robot_x, robot_y;
    if (!getRobotPose(robot_x, robot_y)) {
      return false;
    }

    double best_score= std::numeric_limits<double>::infinity();
    bool found = false;

    for (int y = 1; y < height - 1; y++) {
      for (int x = 1; x < width - 1; x++) {
        if (!isFrontierCell(x, y, map)) {
          continue;
        }
        int safe_x,safe_y;
        if(!findSafeGoalBehindFrontier(x,y,map,safe_x,safe_y)){
          continue;
        }

        double wx, wy;
        mapToWorld(safe_x, safe_y, map, wx, wy);

        double dist = std::hypot(wx - robot_x, wy - robot_y);

        // Ignore very close frontiers
        if (dist < min_goal_distance_) {
          continue;
        }

        if (isNearVisited(wx, wy)) {
          continue;
        }
        if(isNearBlacklisted(wx,wy)){
          continue;
        }
        if(isNearOccupied(safe_x,safe_y,map,2.5)){
          continue;
        }

        int unknown_count = countUnknownNear(x,y,map,5);
        if(unknown_count < 5){
          continue;
        }
        double score = dist - 0.05 * unknown_count;

        if (score < best_score) {
          best_score = score;
          target_x = wx;
          target_y = wy;
          found = true;
        }
      }
    }

    return found;
  }

  //Exit conditions 
  int countFreeNearRobot(double robot_x,double robot_y, 
  const nav_msgs::msg::OccupancyGrid & map,
double radius_m)
{
  int width = map.info.width;
  int height = map.info.height;
  double res = map.info.resolution;

  int cx = static_cast<int>((robot_x - map.info.origin.position.x) / res);
  int cy = static_cast<int>((robot_y - map.info.origin.position.y) / res);

  int radius_cells = static_cast<int>(radius_m / res);
  int free_count = 0;

  for (int dy = -radius_cells; dy <= radius_cells; dy++) {
    for (int dx = -radius_cells; dx <= radius_cells; dx++) {
      int nx = cx + dx;
      int ny = cy + dy;

      if (!isInside(nx, ny, width, height)) {
        continue;
      }

      double dist = std::hypot(dx * res, dy * res);
      if (dist > radius_m) {
        continue;
      }

      int value = map.data[index(nx, ny, width)];

      if (value >= 0 && value <= 20) {
        free_count++;
      }
    }
  }
  RCLCPP_INFO(
  this->get_logger(),
  "Free cells around robot: %d",
  free_count
);

  return free_count;
}
bool isOutOfMaze()
{
  if (!latest_map_) {
    return false;
  }

  double robot_x, robot_y;
  if (!getRobotPose(robot_x, robot_y)) {
    return false;
  }

  int free_count = countFreeNearRobot(robot_x, robot_y, *latest_map_, 0.5);

  if (free_count > 310) {//1 cell = 0.0025, 0.5 m radius is 0.785
    return true;
  }

  return false;
}

  void explore()
  {
    if (isOutOfMaze()) {
      RCLCPP_INFO(this->get_logger(), "Robot is out of the maze. Exploration complete.");
      return;
    }
    if (goal_active_) {
      return;
    }

    if (!latest_map_) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Waiting for map..."
      );
      return;
    }

    if (!nav_client_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_WARN(
        this->get_logger(),
        "Nav2 action server not available."
      );
      return;
    }

    double target_x, target_y;
    if (!findNearestFrontier(target_x, target_y)) {
      RCLCPP_INFO(
        this->get_logger(),
        "No frontier found. Exploration may be complete."
      );
      return;
    }

    sendGoal(target_x, target_y);
  }
  void publishGoalMarker(double x, double y)
{
  visualization_msgs::msg::Marker marker;

  marker.header.frame_id = "map";
  marker.header.stamp = this->now();

  marker.ns = "exploration_goal";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0.15;

  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  marker.lifetime = rclcpp::Duration::from_seconds(0.0);

  goal_marker_pub_->publish(marker);
}

  void sendGoal(double x, double y)
  {
    NavigateToPose::Goal goal_msg;

    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->now();

    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    goal_msg.pose.pose.position.z = 0.0;

    // Face forward. Orientation is not very important for exploration.
    goal_msg.pose.pose.orientation.x = 0.0;
    goal_msg.pose.pose.orientation.y = 0.0;
    goal_msg.pose.pose.orientation.z = 0.0;
    goal_msg.pose.pose.orientation.w = 1.0;

    RCLCPP_INFO(
      this->get_logger(),
      "Sending exploration goal: x=%.2f y=%.2f",
      x,
      y
    );

    auto send_goal_options =
      rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      [this](const GoalHandleNavigate::SharedPtr & goal_handle)
      {
        if (!goal_handle) {
          RCLCPP_WARN(this->get_logger(), "Goal rejected.");
          goal_active_ = false;
        } else {
          RCLCPP_INFO(this->get_logger(), "Goal accepted.");
          goal_active_ = true;
        }
      };

      last_goal_x_= x;
      last_goal_y_ = y;

    send_goal_options.result_callback =
      [this](const GoalHandleNavigate::WrappedResult & result)
      {
        goal_active_ = false;

        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal reached. Marked as visited.");
            visited_goals_.push_back({last_goal_x_, last_goal_y_});
            break;

          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(this->get_logger(), "Goal aborted. Blacklisting this area.");
            blacklisted_goals_.push_back({last_goal_x_,last_goal_y_});
            break;

          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Goal canceled.");
            break;

          default:
            RCLCPP_WARN(this->get_logger(), "Unknown goal result.");
            break;
        }
      };
    publishGoalMarker(x,y);
    nav_client_->async_send_goal(goal_msg, send_goal_options);

  }
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrontierExplorer>());
  rclcpp::shutdown();
  return 0;
}
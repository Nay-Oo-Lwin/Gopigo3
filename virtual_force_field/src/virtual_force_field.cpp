#include <limits>
#include <algorithm>
#include<memory>
#include<cmath>
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
class virtual_force_field : public rclcpp::Node
{
    public:
    virtual_force_field():Node("virtual_force_field"){
    k_att_ = declare_parameter("k_att", 2.0);
    k_rep_ = declare_parameter("k_rep", 0.10);
    obstacle_range_ = declare_parameter("obstacle_range", 0.5);
    max_rep_force_ = declare_parameter("max_rep_force", 2.5);

    max_linear_speed_ = declare_parameter("max_linear_speed", 0.12);
    max_angular_speed_ = declare_parameter("max_angular_speed", 1.0);
    k_turn_ = declare_parameter("k_turn", 1.5);
    goal_tolerance_ = declare_parameter("goal_tolerance", 0.15);

    goal_sub_=this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose",10,std::bind(&virtual_force_field::goal_callback,this,std::placeholders::_1));
    scan_sub_=this->create_subscription<sensor_msgs::msg::LaserScan>("/scan",10,std::bind(&virtual_force_field::sensor_callback,this,std::placeholders::_1));
    cmd_pub_=this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);
    marker_pub_=this->create_publisher<visualization_msgs::msg::Marker>("/vff_markers",10);
    tf_buffer_=std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_=std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }
    private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    geometry_msgs::msg::PoseStamped latest_goal_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    bool goal_received_= false;
    std::shared_ptr<tf2_ros::Buffer>tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>tf_listener_;
    laser_geometry::LaserProjection projector_;

  double k_att_;
  double k_rep_;
  double obstacle_range_;
  double max_rep_force_;
  double max_linear_speed_;
  double max_angular_speed_;
  double k_turn_;
  double goal_tolerance_;


// attractive(goal): blue, repulsive: red, resultant : green
  void publishArrow(int id,double fx, double fy, float r, float g,float b){
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id="base_link";
        marker.header.stamp=rclcpp::Time(0);

        marker.ns="vff_forces";
        marker.id =id;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action=visualization_msgs::msg::Marker::ADD;

        geometry_msgs::msg::Point start,end;

        start.x = 0.0;
        start.y = 0.0;
        start.z = 0.1;

        double scale = 0.5;

        end.x = scale * fx;
        end.y = scale * fy;
        end.z = 0.1;

        marker.points.push_back(start);
        marker.points.push_back(end);

        //for arrow
        marker.scale.x = 0.03;
        marker.scale.y=0.06;
        marker.scale.z=0.1;

        marker.color.r =r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0; 

        marker.lifetime = rclcpp::Duration::from_seconds(0.2);

        marker_pub_->publish(marker);
    }

    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr goal){
        latest_goal_=*goal;
        goal_received_ = true;

    RCLCPP_INFO(
      get_logger(),
      "Goal received in frame '%s': x=%.2f y=%.2f",
      latest_goal_.header.frame_id.c_str(),
      latest_goal_.pose.position.x,
      latest_goal_.pose.position.y);
    }
    double clamp(double value,double min_value, double max_value){
        return std::max(min_value,std::min(value,max_value));
    }

    void sensor_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    geometry_msgs::msg::Twist cmd;
    geometry_msgs::msg::PoseStamped goal_in_base;
    if (!goal_received_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Waiting for goal pose on /goal_pose...");
      return;
    }
    try{
        geometry_msgs::msg::PoseStamped goal_for_tf= latest_goal_;
        goal_for_tf.header.stamp=rclcpp::Time(0);
        goal_in_base=tf_buffer_->transform(goal_for_tf,"base_link",tf2::durationFromSec(0.1));
    }catch(const tf2::TransformException &ex){
        RCLCPP_WARN_THROTTLE(this->get_logger(),*this->get_clock(),1000,"Could not transform goal to base link: %s",ex.what());
        return;
    }
    double goal_x=goal_in_base.pose.position.x;
    double goal_y=goal_in_base.pose.position.y;
    double goal_dist= std::sqrt(goal_x * goal_x + goal_y * goal_y);
    if(goal_dist < goal_tolerance_){
        cmd.linear.x= 0.0;
        cmd.angular.z=0.0;
        cmd_pub_->publish(cmd);
        RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(),1000," Goal reached.");
        return;
    }
    double f_att_x=0.0;
    double f_att_y=0.0;
    if(goal_dist>1e-6){
        f_att_x = k_att_ * goal_x / goal_dist;
        f_att_y = k_att_ * goal_y / goal_dist;
    }
    sensor_msgs::msg::PointCloud2 cloud;
try {
    projector_.transformLaserScanToPointCloud(
        "base_link",
        *msg,
        cloud,
        *tf_buffer_);
} catch (const tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Could not transform LaserScan to base_link: %s", ex.what());
    return;
}   
    double f_rep_x=0.0;
    double f_rep_y=0.0;
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud,"x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud,"y");
    for(;iter_x!=iter_x.end();++iter_x,++iter_y){
        double x=static_cast<double>(*iter_x);
        double y=static_cast<double>(*iter_y);

        if(!std::isfinite(x)||!std::isfinite(y)){
            continue;
        }
        double d = std::sqrt(x *x + y *y);
        if(d<1e-6 || d > obstacle_range_){
            continue;
        }
        double force_mag = k_rep_ / (d*d);
        force_mag = std::min(force_mag, max_rep_force_);

        f_rep_x += -force_mag * x / d;
        f_rep_y += -force_mag * y / d;
    }
    double f_total_x = f_att_x + f_rep_x;
    double f_total_y = f_att_y + f_rep_y;
    

    publishArrow(0,f_att_x,f_att_y,0.0,0.0,1.0);
    publishArrow(1,f_rep_x,f_rep_y,1.0,0.0,0.0);
    publishArrow(2,f_total_x,f_total_y,0.0,1.0,0.0);

    double desired_angle = std::atan2(f_total_y, f_total_x);
    double forward_factor = std::max(0.0, std::cos(desired_angle));

    cmd.linear.x = max_linear_speed_ * forward_factor;
    cmd.angular.z = clamp(
      k_turn_ * desired_angle,
      -max_angular_speed_,
      max_angular_speed_);

    cmd_pub_->publish(cmd);

    }

};
int main(int argc,char ** argv){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<virtual_force_field>());
    rclcpp::shutdown();
    return 0;
}
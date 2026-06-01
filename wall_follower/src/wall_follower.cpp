#include<cmath>
#include<limits>
#include<algorithm>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#define FRONT_ANGLE 180.0
#define RIGHT_SIDE_ANGLE 90.0



class WallFollower : public rclcpp::Node
{
    public:
    WallFollower():Node("wall_follower"){
        target_distance_=this->declare_parameter("target_distance",0.40);
        forward_speed_=this->declare_parameter("forward_speed",0.30);//0.20
        max_turn_rate_=this->declare_parameter("max_turn_rate",2.0);//1.5
        kp_dist_=this->declare_parameter("kp_dist",1.2);
        kp_angle_= this->declare_parameter("kp_angle",1.5);
        front_stop_distance_=this->declare_parameter("front_stop_distance",0.60);
        scan_sub_=this->create_subscription<sensor_msgs::msg::LaserScan>("/scan",10,std::bind(&WallFollower::sensor_callback,this,std::placeholders::_1));
        cmd_pub_=this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);
        RCLCPP_INFO(this->get_logger(),"Wall follower started.");
    }
    private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    double target_distance_;
    double forward_speed_;
    double max_turn_rate_;
    double kp_dist_;
    double kp_angle_;
    double front_stop_distance_;
    double degree2Rad(double degree){
        return ((degree * M_PI) / 180.0);
    }
    double getRangeAtAngle(double angle_rad,const sensor_msgs::msg::LaserScan::SharedPtr msg){
        if(msg->ranges.empty()){
            return std::numeric_limits<double>::infinity();
        }
        int index= static_cast<int>(std::round((angle_rad - msg->angle_min)/msg->angle_increment));
        //Clamping the index
        index=std::max(0,std::min(index,static_cast<int>(msg->ranges.size())-1));
        double r =msg->ranges[index];
        if(!std::isfinite(r)){
            return std::numeric_limits<double>::infinity();
        }
        if(r<msg->range_min || r>msg->range_max){
            return std::numeric_limits<double>::infinity();
        }
        return r;
    }
    double getMinRangeAtAngle(double angle_deg,const sensor_msgs::msg::LaserScan::SharedPtr msg){
        double max_angle= degree2Rad(angle_deg + 20);
        double a = degree2Rad(angle_deg - 20);
        double step = degree2Rad(1.0);
        double min_range = getRangeAtAngle(a,msg);
        for (; a<=max_angle;a+=step){
            double temp = getRangeAtAngle(a,msg);
            if(std::isfinite(temp)&& temp<min_range){
                min_range = temp;
            }
        }
        return min_range;
    }
    double getAverageRangeAtAngle(double angle_deg, const sensor_msgs::msg::LaserScan::SharedPtr msg){
        double max_angle = degree2Rad(angle_deg +2 );
        double a = degree2Rad(angle_deg - 2);
        double step = degree2Rad(1.0);
        double sum = 0.0;
        int count{0};
        for(;a<=max_angle;a+=step){
            double temp = getRangeAtAngle(a,msg);
            if(std::isfinite(temp)){
                sum+=temp;
                count++;
            }
        }
        if(count==0){
            return (std::numeric_limits<double>::infinity());
        }
        return (sum/count);
    }
    void sensor_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        geometry_msgs::msg::Twist cmd;
        double front= getMinRangeAtAngle(FRONT_ANGLE, msg);
        double right_side=getAverageRangeAtAngle(RIGHT_SIDE_ANGLE,msg);
        double right_front_side= getAverageRangeAtAngle(60.0,msg);
        //For debugging only
        double a90=getAverageRangeAtAngle(RIGHT_SIDE_ANGLE,msg);
        double a0=getRangeAtAngle(0.0,msg);
        double a180=getMinRangeAtAngle(FRONT_ANGLE,msg);
        double a270=getRangeAtAngle((3.0 * M_PI)/2.0 , msg);
        double a360=getRangeAtAngle(2.0 * M_PI, msg);
        //Checking the front and turning if needed
        if(front<front_stop_distance_){
            cmd.linear.x=0.3;
            cmd.angular.z=0.8;
            cmd_pub_->publish(cmd);
            return;
        }
        //Checking if the wall is still on the right side
        if(!std::isfinite(right_side)||!std::isfinite(right_front_side)){
            cmd.linear.x=0.06;
            cmd.angular.z=-0.4;
            cmd_pub_->publish(cmd);
            return;
        }
        double theta = degree2Rad(30.0);
        /*alpha is the angle between the wall and the robot it should be 0 when the robot is perfectly parallel to the wall and positive toward the wall and negative away from the wall*/
        double alpha = std::atan2((right_front_side * std::cos(theta))-right_side, right_front_side * std::sin(theta));
        //Checking if the wall is too far or too close
        double dist_to_wall = right_side * std::cos(alpha);
        double distance_error=target_distance_-dist_to_wall;
        double heading_error= alpha;
        double turn = kp_dist_*distance_error + kp_angle_ * heading_error;
        //clamping the error
        turn=std::max(-max_turn_rate_,std::min(turn ,max_turn_rate_));
        cmd.linear.x=forward_speed_;
        cmd.angular.z=turn;
        cmd_pub_->publish(cmd);

        //RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(),1000,
    //"front=%.2f side=%.2f error=%.2f v=%.2f w=%.2f",front,side,error,cmd.linear.x,cmd.angular.z);
    //Debugging printout
    RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(),500,"0=%.2f, 90=%.2f 180=%.2f 270=%.2f 360=%.2f",a0,a90,a180,a270,a360);
    }
};
int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<WallFollower>());
    rclcpp::shutdown();
    return 0;
}

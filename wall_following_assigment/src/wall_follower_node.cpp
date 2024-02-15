#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cstdlib>
#include <limits>
#include <cmath>
#include <wall_following_assigment/pid.h>
class WallFollowerNode : public rclcpp::Node {
public:
    WallFollowerNode()
        : Node("wall_follower_node"), desired_distance_from_wall_(0.0), forward_speed_(0.0) {
        
        // Getting params before setting up the topic subscribers
        this->declare_parameter("forward_speed", forward_speed_);
        this->declare_parameter("desired_distance_from_wall", desired_distance_from_wall_);
      
        // todo: set up the command publisher to publish at topic '/husky_1/cmd_vel'
        // using geometry_msgs::msg::Twist messages
       // cmd_pub = ??

      // todo: set up the laser scan subscriber
      // this will set up a callback function that gets executed
     // upon each spinOnce() call, as long as a laser scan
     // message has been published in the meantime by another node
    // ros::Subscriber laser_sub = ??
    
        
        
    }
       
       

private:
    double desired_distance_from_wall_;
    double forward_speed_;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

  

    void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = forward_speed_;

        // Populate this command based on the distance to the closest
        // object in laser scan. I.e. compute the cross-track error
        // as mentioned in the PID slides.

        // You can populate the command based on either of the following two methods:
       // (1) using only the distance to the closest wall
       // (2) using the distance to the closest wall and the orientation of the wall
       //
       // If you select option 2, you might want to use cascading PID control. 
  
       // cmd.angular.z = ???
        
        cmd_pub_->publish(cmd);

      
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WallFollowerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

// --- CONFIGURATION ---
// ADJUST THESE TO MATCH YOUR ROBOT EXACTLY
const double WHEEL_RADIUS = 0.03;   // Meters (e.g., 3cm radius)
const double WHEEL_BASE = 0.30;     // Meters (Distance between wheels)
const double TICKS_PER_REV = 1350;  // Ticks per full wheel rotation (Check your motor specs!)

class OdomNode : public rclcpp::Node {
public:
    OdomNode() : Node("odom_node") {
        // Subscribe to raw ticks from ESP32 (linear.x = Right, linear.y = Left)
        sub_ticks_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom_raw", 10, std::bind(&OdomNode::handle_ticks, this, std::placeholders::_1));

        // Publish standard Odometry
        pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        
        // Broadcaster for Coordinate Transforms (TF)
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        prev_ticks_R_ = 0; prev_ticks_L_ = 0;
        x_ = 0.0; y_ = 0.0; theta_ = 0.0;
        initialized_ = false;
    }

private:
    void handle_ticks(const nav_msgs::msg::Odometry::SharedPtr msg) {
        long curr_R = (long)msg->twist.twist.linear.x;
        long curr_L = (long)msg->twist.twist.linear.y;

        if (!initialized_) {
            prev_ticks_R_ = curr_R;
            prev_ticks_L_ = curr_L;
            initialized_ = true;
            return;
        }

        // 1. Calculate change in ticks
        long d_ticks_R = curr_R - prev_ticks_R_;
        long d_ticks_L = curr_L - prev_ticks_L_;
        prev_ticks_R_ = curr_R;
        prev_ticks_L_ = curr_L;

        // 2. Convert to meters
        double dist_R = (d_ticks_R / TICKS_PER_REV) * (2.0 * 3.14159 * WHEEL_RADIUS);
        double dist_L = (d_ticks_L / TICKS_PER_REV) * (2.0 * 3.14159 * WHEEL_RADIUS);
        
        double center_dist = (dist_R + dist_L) / 2.0;
        double d_theta = (dist_R - dist_L) / WHEEL_BASE;

        // 3. Update Position (x, y, theta)
        x_ += center_dist * cos(theta_);
        y_ += center_dist * sin(theta_);
        theta_ += d_theta;

        // 4. Publish Odometry Message
        auto odom = nav_msgs::msg::Odometry();
        odom.header.stamp = this->now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        
        // Position
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        pub_odom_->publish(odom);

        // 5. Publish Transform (Essential for SLAM!)
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->now();
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";
        t.transform.translation.x = x_;
        t.transform.translation.y = y_;
        t.transform.rotation = odom.pose.pose.orientation;
        
        tf_broadcaster_->sendTransform(t);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_ticks_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    long prev_ticks_R_, prev_ticks_L_;
    double x_, y_, theta_;
    bool initialized_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomNode>());
    rclcpp::shutdown();
    return 0;
}

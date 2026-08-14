#include <functional>
#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

class OdomTfBroadcaster : public rclcpp::Node {
public:
    OdomTfBroadcaster()
        : Node("odom_tf_broadcaster"), transform_broadcaster_(this) {
        odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            rclcpp::QoS(10),
            std::bind(
                &OdomTfBroadcaster::odometry_callback,
                this,
                std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Publishing odometry poses to TF");
    }

private:
    void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr message) {
        if (message->header.frame_id.empty() || message->child_frame_id.empty()) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Ignoring odometry message with an empty frame ID");
            return;
        }

        geometry_msgs::msg::TransformStamped transform;
        transform.header = message->header;
        transform.child_frame_id = message->child_frame_id;
        transform.transform.translation.x = message->pose.pose.position.x;
        transform.transform.translation.y = message->pose.pose.position.y;
        transform.transform.translation.z = message->pose.pose.position.z;
        transform.transform.rotation = message->pose.pose.orientation;

        transform_broadcaster_.sendTransform(transform);

        if (!reported_first_transform_) {
            RCLCPP_INFO(
                get_logger(),
                "Broadcasting TF %s -> %s",
                transform.header.frame_id.c_str(),
                transform.child_frame_id.c_str());
            reported_first_transform_ = true;
        }
    }

    tf2_ros::TransformBroadcaster transform_broadcaster_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        odometry_subscription_;
    bool reported_first_transform_{false};
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomTfBroadcaster>());
    rclcpp::shutdown();
    return 0;
}

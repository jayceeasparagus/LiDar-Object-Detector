#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>
#include "detector_core/obstacle_detection.hpp"
#include <detector_core/scan_processing.hpp>
#include <cstddef>
#include <string>
#include <algorithm>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cstdint>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cstdint>

class LidarDetectorNode : public rclcpp::Node {
    public:
        LidarDetectorNode() : Node("lidar_detector_node") {
            const std::string scan_topic = this->declare_parameter<std::string>("scan_topic", "scan");
            distance_tolerance_ = this->declare_parameter<double>("distance_tolerance", 0.2);
            min_cluster_size_ = this->declare_parameter<int>("min_cluster_size", 2);
            max_cluster_size_ = this->declare_parameter<int>("max_cluster_size", 1000);

            scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "scan", rclcpp::SensorDataQoS(), std::bind(&LidarDetectorNode::scan_callback, this, std::placeholders::_1));
            
            marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("obstacles_markers", 10);

            obstacle_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("obstacle_points", 10);

            RCLCPP_INFO(this->get_logger(), "LiDar detector node started");
        }

    private:
        double distance_tolerance_;
        std::size_t min_cluster_size_;
        std::size_t max_cluster_size_;

        void scan_callback(
            const sensor_msgs::msg::LaserScan::ConstSharedPtr message) {
                const std::vector<double> ranges(message->ranges.begin(), message->ranges.end());

                const std::vector<detector_core::Point2D> points = detector_core::scan_to_points(ranges, message->angle_min, message->angle_increment, message->range_min, message->range_max);

                const detector_core::DetectionResult result = detector_core::detect_obstacles_with_points(points, distance_tolerance_, min_cluster_size_, max_cluster_size_);

                auto output_header = message->header;
                if (output_header.stamp.sec == 0 &&
                    output_header.stamp.nanosec == 0) {
                    const auto now = this->get_clock()->now();
                    const auto nanoseconds = now.nanoseconds();
                    output_header.stamp.sec =
                        static_cast<std::int32_t>(
                            nanoseconds / 1000000000LL);
                    output_header.stamp.nanosec =
                        static_cast<std::uint32_t>(
                            nanoseconds % 1000000000LL);
                }

                sensor_msgs::msg::PointCloud2 cloud;

                cloud.header = output_header;
                cloud.height = 1;
                cloud.width = static_cast<std::uint32_t>(result.obstacle_points.size());
                cloud.is_bigendian = false;
                cloud.is_dense = true;

                sensor_msgs::PointCloud2Modifier modifier(cloud);
                modifier.setPointCloud2FieldsByString(1, "xyz");
                modifier.resize(result.obstacle_points.size());

                sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
                sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
                sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

                for (const detector_core::Point2D& point : result.obstacle_points) {
                    *iter_x = static_cast<float>(point.x);
                    *iter_y = static_cast<float>(point.y);
                    *iter_z = 0.0F;

                    ++iter_x;
                    ++iter_y;
                    ++iter_z;
                }

                obstacle_cloud_publisher_->publish(cloud);

                visualization_msgs::msg::MarkerArray marker_array;

                const double scan_period = static_cast<double>(message->scan_time);
                const double lifetime_seconds = std::max(0.2, 2.0 * scan_period);
                const int32_t lifetime_sec = static_cast<int32_t>(lifetime_seconds);
                const uint32_t lifetime_nanosec = static_cast<uint32_t>((lifetime_seconds - lifetime_sec) * 1e9);

                for (std::size_t i = 0; i < result.bounding_boxes.size(); ++i) {
                    const auto& box = result.bounding_boxes[i];

                    visualization_msgs::msg::Marker marker;

                    marker.header = output_header;
                    marker.ns = "obstacles";
                    marker.id = static_cast<int32_t>(i);
                    marker.type = visualization_msgs::msg::Marker::CUBE;
                    marker.action = visualization_msgs::msg::Marker::ADD;

                    marker.pose.position.x = (box.min_x + box.max_x) / 2.0;
                    marker.pose.position.y = (box.min_y + box.max_y) / 2.0;
                    marker.pose.position.z = 0.05;
                    marker.pose.orientation.w = 1.0;

                    marker.scale.x = std::max(0.02, box.max_x - box.min_x);
                    marker.scale.y = std::max(0.02, box.max_y - box.min_y);
                    marker.scale.z = 0.1;

                    marker.color.r = 1.0;
                    marker.color.g = 0.0;
                    marker.color.b = 0.0;
                    marker.color.a = 0.5;

                    marker.lifetime.sec = lifetime_sec;
                    marker.lifetime.nanosec = lifetime_nanosec;

                    marker_array.markers.push_back(marker);
                }

                marker_publisher_->publish(marker_array);

                RCLCPP_INFO(this->get_logger(), "Received scan with %zu ranges and detected %zu obstacles",
                message->ranges.size(), result.bounding_boxes.size());

            }

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_cloud_publisher_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<LidarDetectorNode>());

    rclcpp::shutdown();

    return 0;
}
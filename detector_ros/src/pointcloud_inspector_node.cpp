#include "detector_core/bounding_box3d_processing.hpp"
#include "detector_core/clustering.hpp"
#include "detector_core/point3d.hpp"
#include "detector_core/point3d_processing.hpp"

#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class PointCloudInspectorNode : public rclcpp::Node {
public:
    PointCloudInspectorNode()
        : Node("pointcloud_inspector_node") {
        const std::string topic = this->declare_parameter<std::string>(
            "pointcloud_topic", "/lidar/points");

        min_range_ = this->declare_parameter<double>("min_range", 0.1);
        max_range_ = this->declare_parameter<double>("max_range", 30.0);
        voxel_size_ = this->declare_parameter<double>("voxel_size", 0.05);
        clustering_distance_ = this->declare_parameter<double>(
            "clustering_distance", 0.25);
        min_cluster_size_ = this->declare_parameter<int>(
            "min_cluster_size", 5);
        max_cluster_size_ = this->declare_parameter<int>(
            "max_cluster_size", 100000);
        min_z_ = this->declare_parameter<double>("min_z", -0.2);
        max_z_ = this->declare_parameter<double>("max_z", 3.0);

        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic,
            rclcpp::SensorDataQoS(),
            std::bind(
                &PointCloudInspectorNode::pointcloud_callback,
                this,
                std::placeholders::_1));

        filtered_cloud_publisher_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/filtered_points", 10);
        downsampled_cloud_publisher_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/downsampled_points", 10);
        marker_publisher_ =
            this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "/obstacles_3d", 10);

        RCLCPP_INFO(
            this->get_logger(),
            "Listening to PointCloud2 topic: %s",
            topic.c_str());
    }

private:
    void pointcloud_callback(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        std::vector<detector_core::Point3D> points;
        points.reserve(
            static_cast<std::size_t>(message->width) *
            static_cast<std::size_t>(message->height));

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*message, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*message, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*message, "z");

        const std::size_t point_count =
            static_cast<std::size_t>(message->width) *
            static_cast<std::size_t>(message->height);

        for (std::size_t i = 0; i < point_count;
             ++i, ++iter_x, ++iter_y, ++iter_z) {
            points.push_back(detector_core::Point3D{
                static_cast<double>(*iter_x),
                static_cast<double>(*iter_y),
                static_cast<double>(*iter_z)});
        }

        const std::vector<detector_core::Point3D> filtered_points =
            detector_core::filter_points(points, min_range_, max_range_);
        const std::vector<detector_core::Point3D> downsampled_points =
            detector_core::voxel_downsample(filtered_points, voxel_size_);

        std::vector<detector_core::Point3D> obstacle_points;
        for (const detector_core::Point3D& point : downsampled_points) {
            if (point.z >= min_z_ && point.z <= max_z_) {
                obstacle_points.push_back(point);
            }
        }

        const std::vector<detector_core::Cluster> clusters =
            detector_core::spatial_grid_clusters_3d(
                obstacle_points,
                clustering_distance_,
                static_cast<std::size_t>(min_cluster_size_),
                static_cast<std::size_t>(max_cluster_size_));

        publish_cloud(filtered_points, message->header, filtered_cloud_publisher_);
        publish_cloud(
            downsampled_points,
            message->header,
            downsampled_cloud_publisher_);
        publish_markers(obstacle_points, clusters, message->header);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Raw: %zu, valid: %zu, downsampled: %zu, obstacle points: %zu, clusters: %zu",
            points.size(),
            filtered_points.size(),
            downsampled_points.size(),
            obstacle_points.size(),
            clusters.size());
    }

    void publish_cloud(
        const std::vector<detector_core::Point3D>& points,
        const std_msgs::msg::Header& header,
        const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher) {
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header = header;
        cloud.height = 1;
        cloud.width = static_cast<std::uint32_t>(points.size());
        cloud.is_bigendian = false;
        cloud.is_dense = true;

        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(points.size());

        sensor_msgs::PointCloud2Iterator<float> output_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> output_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> output_z(cloud, "z");

        for (const detector_core::Point3D& point : points) {
            *output_x = static_cast<float>(point.x);
            *output_y = static_cast<float>(point.y);
            *output_z = static_cast<float>(point.z);
            ++output_x;
            ++output_y;
            ++output_z;
        }

        publisher->publish(cloud);
    }

    void publish_markers(
        const std::vector<detector_core::Point3D>& points,
        const std::vector<detector_core::Cluster>& clusters,
        const std_msgs::msg::Header& header) {
        visualization_msgs::msg::MarkerArray marker_array;

        visualization_msgs::msg::Marker clear_marker;
        clear_marker.header = header;
        clear_marker.ns = "obstacles_3d";
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);

        for (std::size_t i = 0; i < clusters.size(); ++i) {
            const detector_core::BoundingBox3D box =
                detector_core::bounding_box3d_for_cluster(points, clusters[i]);

            visualization_msgs::msg::Marker marker;
            marker.header = header;
            marker.ns = "obstacles_3d";
            marker.id = static_cast<int32_t>(i);
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = box.center_x;
            marker.pose.position.y = box.center_y;
            marker.pose.position.z = box.center_z;
            marker.pose.orientation.z = std::sin(box.yaw / 2.0);
            marker.pose.orientation.w = std::cos(box.yaw / 2.0);
            marker.scale.x = box.width;
            marker.scale.y = box.depth;
            marker.scale.z = box.height;
            marker.color.r = 1.0F;
            marker.color.g = 0.2F;
            marker.color.b = 0.1F;
            marker.color.a = 0.65F;
            marker.lifetime.sec = 0;
            marker.lifetime.nanosec = 300000000;
            marker_array.markers.push_back(marker);
        }

        marker_publisher_->publish(marker_array);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr downsampled_cloud_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;

    double min_range_;
    double max_range_;
    double voxel_size_;
    double clustering_distance_;
    int min_cluster_size_;
    int max_cluster_size_;
    double min_z_;
    double max_z_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudInspectorNode>());
    rclcpp::shutdown();
    return 0;
}

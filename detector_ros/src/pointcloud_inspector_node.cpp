#include "detector_core/bounding_box3d_processing.hpp"
#include "detector_core/clustering.hpp"
#include "detector_core/multi_object_tracker_3d.hpp"
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
    PointCloudInspectorNode() : Node("pointcloud_inspector_node") {
        const std::string topic = declare_parameter<std::string>("pointcloud_topic", "/lidar/points");
        min_range_ = declare_parameter<double>("min_range", 0.1);
        max_range_ = declare_parameter<double>("max_range", 30.0);
        voxel_size_ = declare_parameter<double>("voxel_size", 0.05);
        clustering_distance_ = declare_parameter<double>("clustering_distance", 0.25);
        min_cluster_size_ = declare_parameter<int>("min_cluster_size", 5);
        max_cluster_size_ = declare_parameter<int>("max_cluster_size", 100000);
        min_z_ = declare_parameter<double>("min_z", -0.2);
        max_z_ = declare_parameter<double>("max_z", 3.0);
        const double association_distance = declare_parameter<double>("tracking_association_distance", 0.8);
        const int max_missed_frames = declare_parameter<int>("tracking_max_missed_frames", 3);
        const double process_noise = declare_parameter<double>("tracking_process_noise", 0.05);
        const double measurement_noise = declare_parameter<double>("tracking_measurement_noise", 0.1);
        tracker_ = std::make_unique<detector_core::MultiObjectTracker3D>(
            association_distance, static_cast<std::size_t>(max_missed_frames),
            process_noise, measurement_noise);

        subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            topic, rclcpp::SensorDataQoS(),
            std::bind(&PointCloudInspectorNode::pointcloud_callback, this, std::placeholders::_1));
        filtered_cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_points", 10);
        downsampled_cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/downsampled_points", 10);
        marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("/obstacles_3d", 10);
        RCLCPP_INFO(get_logger(), "Listening to PointCloud2 topic: %s", topic.c_str());
    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        std::vector<detector_core::Point3D> points;
        const std::size_t point_count = static_cast<std::size_t>(message->width) * message->height;
        points.reserve(point_count);
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*message, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*message, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*message, "z");
        for (std::size_t i = 0; i < point_count; ++i, ++iter_x, ++iter_y, ++iter_z) {
            points.push_back({static_cast<double>(*iter_x), static_cast<double>(*iter_y), static_cast<double>(*iter_z)});
        }

        const auto filtered_points = detector_core::filter_points(points, min_range_, max_range_);
        const auto downsampled_points = detector_core::voxel_downsample(filtered_points, voxel_size_);
        std::vector<detector_core::Point3D> obstacle_points;
        for (const auto& point : downsampled_points) {
            if (point.z >= min_z_ && point.z <= max_z_) obstacle_points.push_back(point);
        }
        const auto clusters = detector_core::spatial_grid_clusters_3d(
            obstacle_points, clustering_distance_, static_cast<std::size_t>(min_cluster_size_),
            static_cast<std::size_t>(max_cluster_size_));

        std::vector<detector_core::BoundingBox3D> detections;
        detections.reserve(clusters.size());
        for (const auto& cluster : clusters) {
            detections.push_back(detector_core::bounding_box3d_for_cluster(obstacle_points, cluster));
        }

        double delta_time = 0.1;
        const rclcpp::Time stamp(message->header.stamp);
        if (have_last_stamp_) {
            const double measured_delta = (stamp - last_stamp_).seconds();
            if (measured_delta > 0.0 && measured_delta < 2.0) delta_time = measured_delta;
        }
        last_stamp_ = stamp;
        have_last_stamp_ = true;
        tracker_->update(detections, delta_time);

        publish_cloud(filtered_points, message->header, filtered_cloud_publisher_);
        publish_cloud(downsampled_points, message->header, downsampled_cloud_publisher_);
        publish_markers(message->header);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "Raw: %zu, valid: %zu, downsampled: %zu, obstacle points: %zu, detections: %zu, active tracks: %zu",
            points.size(), filtered_points.size(), downsampled_points.size(), obstacle_points.size(),
            detections.size(), tracker_->tracks().size());
    }

    void publish_cloud(const std::vector<detector_core::Point3D>& points,
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
        for (const auto& point : points) {
            *output_x = static_cast<float>(point.x);
            *output_y = static_cast<float>(point.y);
            *output_z = static_cast<float>(point.z);
            ++output_x;
            ++output_y;
            ++output_z;
        }
        publisher->publish(cloud);
    }

    void publish_markers(const std_msgs::msg::Header& header) {
        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.header = header;
        clear_marker.ns = "obstacles_3d";
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);

        for (const auto& track : tracker_->tracks()) {
            // Do not draw a predicted-only track after its detection disappears.
            if (track.missed_frames() != 0) {
                continue;
            }

            const auto& box = track.bounding_box();
            const int32_t marker_id = static_cast<int32_t>(track.id() * 2);
            visualization_msgs::msg::Marker box_marker;
            box_marker.header = header;
            box_marker.ns = "obstacles_3d";
            box_marker.id = marker_id;
            box_marker.type = visualization_msgs::msg::Marker::CUBE;
            box_marker.action = visualization_msgs::msg::Marker::ADD;
            box_marker.pose.position.x = track.x();
            box_marker.pose.position.y = track.y();
            box_marker.pose.position.z = track.z();
            box_marker.pose.orientation.z = std::sin(box.yaw / 2.0);
            box_marker.pose.orientation.w = std::cos(box.yaw / 2.0);
            box_marker.scale.x = box.width;
            box_marker.scale.y = box.depth;
            box_marker.scale.z = box.height;
            box_marker.color.r = 1.0F;
            box_marker.color.g = 0.2F;
            box_marker.color.b = 0.1F;
            box_marker.color.a = 0.65F;
            marker_array.markers.push_back(box_marker);

            visualization_msgs::msg::Marker label;
            label.header = header;
            label.ns = "obstacles_3d_labels";
            label.id = marker_id + 1;
            label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            label.action = visualization_msgs::msg::Marker::ADD;
            label.pose.position.x = track.x();
            label.pose.position.y = track.y();
            label.pose.position.z = track.z() + box.height / 2.0 + 0.15;
            label.scale.z = 0.18;
            label.color.r = 1.0F;
            label.color.g = 1.0F;
            label.color.b = 0.0F;
            label.color.a = 1.0F;
            label.text = "ID " + std::to_string(track.id()) +
                "\nv " + std::to_string(track.velocity_x()).substr(0, 5) + "," +
                std::to_string(track.velocity_y()).substr(0, 5) + "," +
                std::to_string(track.velocity_z()).substr(0, 5);
            marker_array.markers.push_back(label);
        }
        marker_publisher_->publish(marker_array);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr downsampled_cloud_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    std::unique_ptr<detector_core::MultiObjectTracker3D> tracker_;
    rclcpp::Time last_stamp_;
    bool have_last_stamp_ = false;
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

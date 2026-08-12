#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>
#include <limits>

using namespace std::chrono_literals;

class SyntheticScanPublisher : public rclcpp::Node {
    public:
        SyntheticScanPublisher() : Node("synthetic_scan_publisher") {
            publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
            clearing_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_clear", 10);

            timer_ = this->create_wall_timer(200ms, std::bind(&SyntheticScanPublisher::publish_scan, this));
        }

    private:
        double elapsed_time_ = 0.0;

        void publish_scan() {
            sensor_msgs::msg::LaserScan scan;

            constexpr std::size_t beam_count = 1080;

            scan.header.stamp = this->get_clock()->now();
            scan.header.frame_id = "laser";

            scan.angle_min = -3.14159265F;
            scan.angle_max = 3.14159265F;
            scan.angle_increment =
                (scan.angle_max - scan.angle_min) /
                static_cast<float>(beam_count);

            scan.time_increment = 0.0F;
            scan.scan_time = 0.2F;

            scan.range_min = 0.1F;
            scan.range_max = 10.F;

            scan.ranges.assign(
                beam_count,
                std::numeric_limits<float>::infinity());

            scan.intensities.clear();

            const double obstacle_one_range = 2.0 + 0.75 * std::sin(elapsed_time_);

            const double obstacle_two_angle = 0.8 + 0.4 * std::sin(elapsed_time_ * 0.7);

            add_obstacle(scan, -0.4, obstacle_one_range);

            const int cycle = static_cast<int>(elapsed_time_) % 12;

        if (cycle < 9) {
            add_obstacle(scan, obstacle_two_angle,4.5);
        }

elapsed_time_ += 0.2;

            publisher_->publish(scan);

            sensor_msgs::msg::LaserScan clearing_scan = scan;

            for (float& range : clearing_scan.ranges) {
                if (!std::isfinite(range) && range > 0.0F) {
                    range = clearing_scan.range_max - 0.0001F;
                }
            }

            clearing_publisher_->publish(clearing_scan);
        }

        void add_obstacle(sensor_msgs::msg::LaserScan& scan, double angle, double range) {
            const int center_index = static_cast<int>((angle - scan.angle_min) / scan.angle_increment);

            for (int offset = -4; offset <= 4; ++offset) {
                const int index = center_index + offset;

                if (index >= 0 && index < static_cast<int>(scan.ranges.size())) {
                    scan.ranges[index] = static_cast<float>(range);
        }
    }
}

        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr clearing_publisher_;

        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<SyntheticScanPublisher>());

    rclcpp::shutdown();

    return 0;
}
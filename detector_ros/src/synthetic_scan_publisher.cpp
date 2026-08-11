#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;

class SyntheticScanPublisher : public rclcpp::Node {
    public:
        SyntheticScanPublisher() : Node("synthetic_scan_publisher") {
            publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

            timer_ = this->create_wall_timer(100ms, std::bind(&SyntheticScanPublisher::publish_scan, this));
        }

    private:
        void publish_scan() {
            sensor_msgs::msg::LaserScan scan;

            scan.header.stamp = this->get_clock()->now();
            scan.header.frame_id = "laser";

            scan.angle_min = 0.0F;
            scan.angle_max = 0.03F;
            scan.angle_increment = 0.01F;

            scan.time_increment = 0.0F;
            scan.scan_time = 0.1F;

            scan.range_min = 0.1F;
            scan.range_max = 10.F;

            scan.ranges = {1.0F, 1.05F, 5.0F, 5.05F};

            scan.intensities.clear();

            publisher_->publish(scan);
        }

        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;

        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<SyntheticScanPublisher>());

    rclcpp::shutdown();

    return 0;
}
#include <cstddef>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ExampleLaser : public rclcpp::Node
{
public:
  ExampleLaser()
  : Node("example_laser")
  {
    subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan_filtered", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (msg->ranges.empty()) {
          RCLCPP_WARN(get_logger(), "Received an empty laser scan");
          return;
        }

        const std::size_t center = msg->ranges.size() / 2;
        RCLCPP_INFO(get_logger(), "Distance front: %.3f", msg->ranges[center]);
      });
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExampleLaser>());
  rclcpp::shutdown();
  return 0;
}

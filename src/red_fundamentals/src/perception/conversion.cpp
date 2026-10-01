#ifndef RCLCPP
#define RCLCPP
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/utilities.hpp>
#endif

#include "sensor_msgs/point_cloud_conversion.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

using LaserScan = sensor_msgs::msg::LaserScan;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointField = sensor_msgs::msg::PointField;

const float LIDAR_MID_DIST = 0.14;

using std::placeholders::_1;

rclcpp::Subscription<LaserScan>::SharedPtr subscription_;
rclcpp::Publisher<PointCloud2>::SharedPtr publisher_;

void scan_callback(const LaserScan& msg) {
	int size = msg.ranges.size();
	float angle = msg.angle_min;

	int valid_count = 0;
	for (int i = 0; i < size; i++)
		if (std::isfinite(msg.ranges[i]) && msg.ranges[i] >= msg.range_min)
			valid_count++;

	std::vector<uint8_t> points;
	points.resize(valid_count * 12);
	float* points_ptr = reinterpret_cast<float*>(points.data());

	int valid_index = 0;
	for (int i=0; i < size; i++) {
		if (!std::isfinite(msg.ranges[i]) || msg.ranges[i] < msg.range_min) {
			angle += msg.angle_increment;
			continue;
		}
		float y = msg.ranges[i] * std::cos(angle);
		float x = msg.ranges[i] * -std::sin(angle);
		y += LIDAR_MID_DIST;

		points_ptr[valid_index * 3 + 0] = x;
		points_ptr[valid_index * 3 + 1] = y;
		points_ptr[valid_index * 3 + 2] = 0;
		valid_index++;

		angle += msg.angle_increment;
	}

	auto cloud = std::make_shared<PointCloud2>();

	const rclcpp::Time stamp = rclcpp::Clock(RCL_ROS_TIME).now();
	cloud->header = msg.header;
	cloud->header.frame_id = "base_link"; //TODO probably needed for correct values
	cloud->header.stamp = stamp;
	cloud->height = 1;
	cloud->width = valid_count;
	cloud->is_dense = true;

	auto make_field = [](const std::string& name, uint32_t offset) {
	    PointField f;
	    f.name = name;
	    f.offset = offset;
	    f.datatype = PointField::FLOAT32;
	    f.count = 1;
	    return f;
	};
	cloud->fields = { make_field("x", 0), make_field("y", 4), make_field("z", 8) };

	cloud->point_step = 12;
	cloud->row_step = 12 * valid_count;
	cloud->data = points;

	publisher_->publish(*cloud);
	printf("Published point cloud with %d points\n", valid_count);
}

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);

	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("conversion");

	subscription_ = node->create_subscription<LaserScan>("scan_filtered", 10, std::bind(&scan_callback, _1));
	publisher_ = node->create_publisher<PointCloud2>("point_cloud", 10);

	rclcpp::spin(node);

	rclcpp::shutdown();
	return 0;
}


#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "create_fundamentals/msg/sensor_packet.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using std::placeholders::_1;

static constexpr double WHEELBASE_M = 0.258;
static constexpr double ENCODER_TO_METERS = 0.032;

class OdometryNode : public rclcpp::Node
{
public:
    OdometryNode() : Node("odometry")
    {

        have_last_ = false;
        last_left_ = 0.0;
        last_right_ = 0.0;

        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        sub_ = this->create_subscription<create_fundamentals::msg::SensorPacket>(
            "sensor_packet", 10,
            std::bind(&OdometryNode::packet_callback, this, _1));

        RCLCPP_INFO(this->get_logger(),
                    "Odometry node started. wheelbase=%.3f m, encoder scale=%.4f m/unit",
                    WHEELBASE_M, ENCODER_TO_METERS);
    }

private:
    void packet_callback(const create_fundamentals::msg::SensorPacket::SharedPtr msg)
    {

        const double left_m = msg->encoder_left * ENCODER_TO_METERS;
        const double right_m = msg->encoder_right * ENCODER_TO_METERS;
        RCLCPP_INFO(this->get_logger(), "raw L=%.2f R=%.2f  x=%.3f y=%.3f",
                    msg->encoder_left, msg->encoder_right, x_, y_);

        if (!have_last_)
        {
            last_left_ = left_m;
            last_right_ = right_m;
            have_last_ = true;
            return;
        }

        const double d_left = left_m - last_left_;
        const double d_right = right_m - last_right_;
        last_left_ = left_m;
        last_right_ = right_m;

        const double d_center = (d_left + d_right) / 2.0;
        const double d_theta = (d_right - d_left) / WHEELBASE_M;

        const double mid_theta = theta_ + d_theta / 2.0;

        x_ += d_center * std::cos(mid_theta);
        y_ += d_center * std::sin(mid_theta);
        theta_ += d_theta;

        while (theta_ > M_PI)
            theta_ -= 2.0 * M_PI;
        while (theta_ < -M_PI)
            theta_ += 2.0 * M_PI;

        const rclcpp::Time stamp = this->now();
        publish(stamp);
    }

    void publish(const builtin_interfaces::msg::Time &stamp)
    {

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, theta_);

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        odom_pub_->publish(odom);

        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = stamp;
        tf.header.frame_id = "odom";
        tf.child_frame_id = "base_link";
        tf.transform.translation.x = x_;
        tf.transform.translation.y = y_;
        tf.transform.translation.z = 0.0;
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(tf);
    }

    bool have_last_;
    double last_left_, last_right_;
    double x_, y_, theta_;

    rclcpp::Subscription<create_fundamentals::msg::SensorPacket>::SharedPtr sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryNode>());
    rclcpp::shutdown();
    return 0;
}
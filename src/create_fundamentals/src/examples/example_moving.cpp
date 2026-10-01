#include <chrono>
#include <memory>
#include <thread>

#include "create_fundamentals/msg/sensor_packet.hpp"
#include "create_fundamentals/srv/diff_drive.hpp"
#include "create_fundamentals/srv/reset_encoders.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class ExampleMoving : public rclcpp::Node
{
public:
  ExampleMoving()
  : Node("example_moving")
  {
    subscription_ = create_subscription<create_fundamentals::msg::SensorPacket>(
      "sensor_packet", 10,
      [this](const create_fundamentals::msg::SensorPacket::SharedPtr msg) {
        encoder_left_ = msg->encoder_left;
        encoder_right_ = msg->encoder_right;
      });
    drive_client_ = create_client<create_fundamentals::srv::DiffDrive>("diff_drive");
    reset_client_ = create_client<create_fundamentals::srv::ResetEncoders>("reset_encoders");
  }

  bool wait_for_services()
  {
    RCLCPP_INFO(get_logger(), "Waiting for create driver ...");
    while (!drive_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        return false;
      }
      RCLCPP_INFO(get_logger(), "Waiting for diff_drive service...");
    }
    while (!reset_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        return false;
      }
      RCLCPP_INFO(get_logger(), "Waiting for reset_encoders service...");
    }
    RCLCPP_INFO(get_logger(), "Create driver is ready!");
    return true;
  }

  auto reset_encoders()
  {
    return reset_client_->async_send_request(
      std::make_shared<create_fundamentals::srv::ResetEncoders::Request>());
  }

  auto drive(float left, float right)
  {
    auto request = std::make_shared<create_fundamentals::srv::DiffDrive::Request>();
    request->left = left;
    request->right = right;
    return drive_client_->async_send_request(request);
  }

  void log_encoders() const
  {
    RCLCPP_INFO(
      get_logger(), "encoder_l: %.3f, encoder_r: %.3f", encoder_left_, encoder_right_);
  }

private:
  rclcpp::Subscription<create_fundamentals::msg::SensorPacket>::SharedPtr subscription_;
  rclcpp::Client<create_fundamentals::srv::DiffDrive>::SharedPtr drive_client_;
  rclcpp::Client<create_fundamentals::srv::ResetEncoders>::SharedPtr reset_client_;
  float encoder_left_{0.0F};
  float encoder_right_{0.0F};
};

template<typename FutureT>
bool wait_for_response(rclcpp::executors::SingleThreadedExecutor & executor, FutureT & future)
{
  if (executor.spin_until_future_complete(future) != rclcpp::FutureReturnCode::SUCCESS) {
    return false;
  }
  return future.get()->success;
}

void spin_for(
  rclcpp::executors::SingleThreadedExecutor & executor,
  std::chrono::steady_clock::duration duration)
{
  const auto deadline = std::chrono::steady_clock::now() + duration;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }
}

void best_effort_stop(
  const std::shared_ptr<ExampleMoving> & node,
  rclcpp::executors::SingleThreadedExecutor & executor)
{
  auto stop = node->drive(0.0F, 0.0F);
  executor.spin_until_future_complete(stop, 2s);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExampleMoving>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  if (!node->wait_for_services()) {
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Differential drive demo ...");
  spin_for(executor, 1s);

  auto reset = node->reset_encoders();
  if (!wait_for_response(executor, reset)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to reset encoders");
    rclcpp::shutdown();
    return 1;
  }
  spin_for(executor, 1s);
  node->log_encoders();

  auto drive_left = node->drive(10.0F, 0.0F);
  if (!wait_for_response(executor, drive_left)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to command left wheel");
    best_effort_stop(node, executor);
    rclcpp::shutdown();
    return 1;
  }
  spin_for(executor, 1s);
  auto stop = node->drive(0.0F, 0.0F);
  if (!wait_for_response(executor, stop)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to stop robot");
    best_effort_stop(node, executor);
    rclcpp::shutdown();
    return 1;
  }
  spin_for(executor, 1s);
  node->log_encoders();

  auto drive_right = node->drive(0.0F, 10.0F);
  if (!wait_for_response(executor, drive_right)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to command right wheel");
    best_effort_stop(node, executor);
    rclcpp::shutdown();
    return 1;
  }
  spin_for(executor, 1s);
  auto final_stop = node->drive(0.0F, 0.0F);
  if (!wait_for_response(executor, final_stop)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to stop robot");
    best_effort_stop(node, executor);
    rclcpp::shutdown();
    return 1;
  }
  spin_for(executor, 1s);
  node->log_encoders();

  rclcpp::shutdown();
  return 0;
}

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "create_fundamentals/srv/play_song.hpp"
#include "create_fundamentals/srv/store_song.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class ExampleSong : public rclcpp::Node
{
public:
  ExampleSong()
  : Node("example_song")
  {
    store_client_ = create_client<create_fundamentals::srv::StoreSong>("store_song");
    play_client_ = create_client<create_fundamentals::srv::PlaySong>("play_song");
  }

  bool wait_for_services()
  {
    RCLCPP_INFO(get_logger(), "Waiting for create driver ...");
    while (!store_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        return false;
      }
      RCLCPP_INFO(get_logger(), "Waiting for store_song service...");
    }
    while (!play_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        return false;
      }
      RCLCPP_INFO(get_logger(), "Waiting for play_song service...");
    }
    RCLCPP_INFO(get_logger(), "Create driver is ready!");
    return true;
  }

  auto store(std::uint8_t number, const std::vector<std::uint8_t> & song)
  {
    auto request = std::make_shared<create_fundamentals::srv::StoreSong::Request>();
    request->number = number;
    request->song = song;
    return store_client_->async_send_request(request);
  }

  auto play(std::uint8_t number)
  {
    auto request = std::make_shared<create_fundamentals::srv::PlaySong::Request>();
    request->number = number;
    return play_client_->async_send_request(request);
  }

private:
  rclcpp::Client<create_fundamentals::srv::StoreSong>::SharedPtr store_client_;
  rclcpp::Client<create_fundamentals::srv::PlaySong>::SharedPtr play_client_;
};

template<typename FutureT>
bool wait_for_response(rclcpp::executors::SingleThreadedExecutor & executor, FutureT & future)
{
  if (executor.spin_until_future_complete(future) != rclcpp::FutureReturnCode::SUCCESS) {
    return false;
  }
  return future.get()->success;
}

void sleep_seconds(double seconds)
{
  const auto duration = std::chrono::duration<double>(seconds);
  rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(duration));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExampleSong>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  if (!node->wait_for_services()) {
    rclcpp::shutdown();
    return 1;
  }

  constexpr std::uint8_t rest = 30;
  constexpr std::uint8_t f4 = 65;
  constexpr std::uint8_t a4 = 69;
  constexpr std::uint8_t bes4 = 70;
  constexpr std::uint8_t b4 = 71;
  constexpr std::uint8_t c5 = 72;
  constexpr std::uint8_t des5 = 73;
  constexpr std::uint8_t d5 = 74;
  constexpr std::uint8_t ees5 = 75;
  constexpr std::uint8_t e5 = 76;
  constexpr std::uint8_t f5 = 77;
  constexpr std::uint8_t ges5 = 78;
  constexpr std::uint8_t g5 = 79;
  constexpr std::uint8_t aes5 = 80;
  constexpr std::uint8_t a5 = 81;
  constexpr std::uint8_t aes4 = 68;

  constexpr std::uint8_t measure = 160;
  constexpr std::uint8_t half = measure / 2;
  constexpr std::uint8_t quarter = measure / 4;
  constexpr std::uint8_t eighth = measure / 8;
  constexpr std::uint8_t dotted_eighth = measure * 3 / 16;
  constexpr std::uint8_t sixteenth = measure / 16;
  constexpr double measure_time = measure / 64.0;

  const std::vector<std::vector<std::uint8_t>> songs = {
    {rest, quarter},
    {
      a4, quarter, a4, quarter, a4, quarter, f4, dotted_eighth, c5, sixteenth,
      a4, quarter, f4, dotted_eighth, c5, sixteenth, a4, half,
    },
    {
      e5, quarter, e5, quarter, e5, quarter, f5, dotted_eighth, c5, sixteenth,
      aes4, quarter, f4, dotted_eighth, c5, sixteenth, a4, half,
    },
    {
      a5, quarter, a4, dotted_eighth, a4, sixteenth, a5, quarter, aes5, eighth,
      g5, eighth, ges5, sixteenth, f5, sixteenth, ges5, sixteenth,
    },
    {
      rest, eighth, bes4, eighth, ees5, quarter, d5, eighth, des5, eighth,
      c5, sixteenth, b4, sixteenth, c5, eighth,
    },
    {
      rest, eighth, f4, eighth, aes4, quarter, f4, dotted_eighth, aes4, sixteenth,
      c5, quarter, a4, dotted_eighth, c5, sixteenth, e5, half,
    },
    {
      rest, eighth, f4, eighth, aes4, quarter, f4, dotted_eighth, c5, sixteenth,
      a4, quarter, f4, dotted_eighth, c5, sixteenth, a4, half,
    },
  };

  RCLCPP_INFO(node->get_logger(), "send songs...");
  for (std::size_t number = 0; number < songs.size(); ++number) {
    auto future = node->store(static_cast<std::uint8_t>(number), songs[number]);
    if (!wait_for_response(executor, future)) {
      RCLCPP_ERROR(node->get_logger(), "Failed to store song %zu", number);
      rclcpp::shutdown();
      return 1;
    }
  }

  sleep_seconds(1.0);
  RCLCPP_INFO(node->get_logger(), "play songs...");
  const std::vector<std::pair<std::uint8_t, double>> playback = {
    {1, 2.01}, {2, 2.01}, {3, 1.26}, {4, 1.01},
    {5, 1.76}, {3, 1.26}, {4, 1.01}, {6, 1.76},
  };
  for (const auto & entry : playback) {
    auto future = node->play(entry.first);
    if (!wait_for_response(executor, future)) {
      RCLCPP_ERROR(
        node->get_logger(), "Failed to play song %u",
        static_cast<unsigned int>(entry.first));
      rclcpp::shutdown();
      return 1;
    }
    sleep_seconds(measure_time * entry.second);
  }

  rclcpp::shutdown();
  return 0;
}

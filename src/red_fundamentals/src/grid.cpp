#include <rclcpp/rclcpp.hpp>
#include <red_fundamentals/srv/execute_plan.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <atomic>
#include <thread>
#include <chrono>
#include <cmath>

#define MOVEMENT_IMPLEMENTATION
#include "movement.hpp"

using namespace std::chrono_literals;

class GridMover : public rclcpp::Node {
public:
    GridMover() : Node("grid_mover"), current_angle_(0.0), obstacle_in_front_(false) {
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        
        execute_plan_srv_ = this->create_service<red_fundamentals::srv::ExecutePlan>(
            "execute_plan", 
            std::bind(&GridMover::execute_plan_cb, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            cb_group_
        );

        add_move_client_ = this->create_client<AddMove>("add_move");
        abort_move_client_ = this->create_client<AbortMove>("abort_move");
        clear_queue_client_ = this->create_client<ClearQueue>("clear_move_queue");

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan_filtered", 10, std::bind(&GridMover::scan_cb, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Grid mover node ready. Waiting for plans.");
    }

private:
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::Service<red_fundamentals::srv::ExecutePlan>::SharedPtr execute_plan_srv_;
    
    rclcpp::Client<AddMove>::SharedPtr add_move_client_;
    rclcpp::Client<AbortMove>::SharedPtr abort_move_client_;
    rclcpp::Client<ClearQueue>::SharedPtr clear_queue_client_;
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    std::atomic<bool> obstacle_in_front_;
    double current_angle_; 
    const float cell_size_cm_ = 80.0f; 

    void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        int mid_idx = msg->ranges.size() / 2;
        int arc_size = 15; 
        
        bool obstacle = false;
        int start_idx = std::max(0, mid_idx - arc_size);
        int end_idx = std::min(static_cast<int>(msg->ranges.size()), mid_idx + arc_size);
        
        for (int i = start_idx; i < end_idx; ++i) {
            float r = msg->ranges[i];
            if (std::isfinite(r) && r > 0.0f && r < 0.10f) {
                obstacle = true;
                break;
            }
        }
        obstacle_in_front_ = obstacle;
    }

    bool wait_and_check(double duration_sec) {
        int steps = static_cast<int>(duration_sec / 0.1);
        for (int i = 0; i < steps; ++i) {
            if (obstacle_in_front_) {
                return false;
            }
            std::this_thread::sleep_for(100ms);
        }
        return true;
    }

    void handle_abort() {
        RCLCPP_WARN(this->get_logger(), "Obstacle detected! Aborting.");
        abort_move_req(abort_move_client_);
        clear_queue_req(clear_queue_client_);
    }

    void execute_plan_cb(const std::shared_ptr<red_fundamentals::srv::ExecutePlan::Request> request,
                         std::shared_ptr<red_fundamentals::srv::ExecutePlan::Response> response) {
        RCLCPP_INFO(this->get_logger(), "Executing plan...");

        for (int action : request->plan) {
            double target_angle = 0.0;
            switch (action) {
                case red_fundamentals::srv::ExecutePlan::Request::RIGHT: target_angle = -90.0; break;
                case red_fundamentals::srv::ExecutePlan::Request::UP:    target_angle = 0.0;   break;
                case red_fundamentals::srv::ExecutePlan::Request::LEFT:  target_angle = 90.0;  break;
                case red_fundamentals::srv::ExecutePlan::Request::DOWN:  target_angle = 180.0; break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Unknown action code: %d", action);
                    response->success = false;
                    return;
            }

            double turn_deg = target_angle - current_angle_;
            
            turn_deg = std::fmod(turn_deg + 180.0, 360.0);
            if (turn_deg < 0) turn_deg += 360.0;
            turn_deg -= 180.0;

            if (std::abs(turn_deg) > 1.0) {
                send_async(turn(turn_deg), add_move_client_);
                
                if (!wait_and_check(3.5)) {
                    handle_abort();
                    response->success = false;
                    return;
                }
                current_angle_ = target_angle;
            }

            if (obstacle_in_front_) {
                handle_abort();
                response->success = false;
                return;
            }

            send_async(drive(cell_size_cm_), add_move_client_);

            if (!wait_and_check(5.5)) {
                handle_abort();
                response->success = false;
                return;
            }
            
            std::this_thread::sleep_for(500ms);
        }

        RCLCPP_INFO(this->get_logger(), "Plan executed successfully.");
        response->success = true;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GridMover>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}

#include <rclcpp/rclcpp.hpp>
#include <red_fundamentals/msg/line_segment.hpp>
#include <red_fundamentals/msg/line_segment_array.hpp>
#include <red_fundamentals/srv/add_move.hpp>
#include <red_fundamentals/srv/clear_move_queue.hpp>
#include <red_fundamentals/srv/abort_move.hpp>
#include <std_msgs/msg/empty.hpp>
#define MOVEMENT_IMPLEMENTATION
#include "movement.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <optional>
#include <atomic>

std::shared_ptr<rclcpp::Node> align;

using namespace std::chrono_literals;

visualization_msgs::msg::Marker marker_from_point(
    double x, double y, rclcpp::Node* node, const std::string& frame_id = "laser", 
    float r = 1.0, float g = 0.0, float b = 0.0) 
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = node->now();
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.1;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    return marker;
}

class Align : public rclcpp::Node {
public:
    Align(int mode = 3)
        : Node("align"), mode_(mode), state_(0), wheel_radius_(3.1), wheel_dist_mid_(25.47) {
        std::cout << "Start Aligning! (Mode: " << mode_ << ")\n";
        std::cout << "Subscribing to extracted_walls and cell points...\n";

        // Separate callback group for the routine timer
        routine_cb_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        subscription_ = this->create_subscription<red_fundamentals::msg::LineSegmentArray>(
            "extracted_walls", 10,
            std::bind(&Align::line_segment_callback, this, std::placeholders::_1));

        center_subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "cell_center", 10,
            std::bind(&Align::center_callback, this, std::placeholders::_1));

        corner_subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "cell_corner", 10,
            std::bind(&Align::corner_callback, this, std::placeholders::_1));

        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "align_markers", 10);

        marker_publisher2_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "O_markers", 10);

        std::cout << "Done subscribing.\n";
        std::cout << "Creating client for add_move...\n";

        add_move_client_ = this->create_client<red_fundamentals::srv::AddMove>("add_move");
        while (!add_move_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) return;
            std::cout << "Waiting for move_client service...\n";
        }

        abort_move_client_ = this->create_client<red_fundamentals::srv::AbortMove>("abort_move");
        while (!abort_move_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) return;
            std::cout << "Waiting for abort_move service...\n";
        }

        clear_move_client_ = this->create_client<red_fundamentals::srv::ClearMoveQueue>("clear_move_queue");
        while (!clear_move_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) return;
            std::cout << "Waiting for clear_move_queue service...\n";
        }

        std::cout << "Done creating clients\n";

        {
            rclcpp::PublisherOptions align_done_opts;
            align_done_opts.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
            align_done_pub_ = this->create_publisher<std_msgs::msg::Empty>("align_done", 10, align_done_opts);
        }

        {
            rclcpp::SubscriptionOptions trigger_align_opts;
            trigger_align_opts.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
            trigger_align_sub_ = this->create_subscription<std_msgs::msg::Empty>(
                "trigger_align", 10,
                std::bind(&Align::trigger_align_callback, this, std::placeholders::_1),
                trigger_align_opts);
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Align::routine_step, this),
            routine_cb_group_);
    }
private:
    std::atomic<bool> movement_in_progress_{false};
    bool has_travelled = false;
    bool has_turned = false;

    void line_segment_callback(const red_fundamentals::msg::LineSegmentArray::SharedPtr msg) {
        latest_lines_ = msg;
    }

    void center_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        if (std::isnan(msg->point.x)) {
            target_m_.reset();
        } else {
			// printf("CENTER RECEIVED: x: %f, y: %f\n", msg->point.x, msg->point.y);
            target_m_ = Eigen::Vector2d(msg->point.x, msg->point.y);
        }
    }

    void corner_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        if (std::isnan(msg->point.x)) {
            corner_.reset();
        } else {
            corner_ = Eigen::Vector2d(msg->point.x, msg->point.y);
        }
    }

    double line_deg_offset() {
        if (!latest_lines_) {
            return 0.0;
        }

        const auto& segments = latest_lines_->segments;
        double relative_deg = 0.0;

        for (size_t i = 0; i < segments.size(); ++i) {
            double x = segments[i].start_x - segments[i].end_x;
            double y = segments[i].start_y - segments[i].end_y;

            double deg = std::atan2(y, x) * 180.0 / M_PI;
            double deg_mod = deg - std::floor(deg / 90.0) * 90.0;

            double error_deg = 0.0;
            if (deg_mod > 45.0) {
                error_deg = deg_mod - 90.0;
            } else {
                error_deg = deg_mod;
            }

            double w1 = static_cast<double>(i) / (i + 1);
            double w2 = 1.0 / (i + 1);
            relative_deg = w1 * relative_deg + w2 * error_deg;
        }

        return relative_deg;
    }

    void clear_pending_queue() {
        auto req = std::make_shared<red_fundamentals::srv::ClearMoveQueue::Request>();
        auto fut = clear_move_client_->async_send_request(req);
        if (fut.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
            (void)fut.get();
        }
    }

    void send_moves(const std::vector<DriveInstruction>& moves) {
        for (const auto& m : moves) {
            auto request = std::make_shared<red_fundamentals::srv::AddMove::Request>();
            request->left  = m.left;
            request->right = m.right;
            printf("send_moves: (%f, %f)\n", request->left, request->right);
            futures_.push_back(add_move_client_->async_send_request(request).future.share());
        }
    }

    bool await_moves() {
        bool all_ok = true;
        for (auto& f : futures_) {
            f.wait();
            auto response = f.get();
            printf("one movement done with: %d\n", response->ret);
            if (response->ret != MOVE_RETURN_SUCCESS &&
                response->ret != MOVE_RETURN_REPLACED) {
                all_ok = false;
            }
        }
        futures_.clear();
        return all_ok;
    }

    void execute_moves_and_wait(const std::vector<DriveInstruction>& moves) {
        if (moves.empty()) return;
        movement_in_progress_ = true;
        send_moves(moves);
        if (!await_moves()) {
			printf("MOVE FAILED! restarting \n");
			state_ = 0;
		}
        move_until_time_ = this->now() + rclcpp::Duration::from_seconds(3);
        movement_in_progress_ = false;
    }

    void trigger_align_callback(const std_msgs::msg::Empty::SharedPtr /*msg*/) {
        std::cout << "trigger_align received: restarting alignment (state " << state_ << " -> 0)\n";
        clear_pending_queue();
        abort_move_req(abort_move_client_);
        state_ = 0;
        align_done_published_ = false;
        move_until_time_.reset();
        target_m_.reset();
        corner_.reset();
    }

    void routine_step() {
        if (state_ == 4 && !align_done_published_) {
            align_done_published_ = true;
            std::cout << "Alignment complete! Publishing align_done.\n";
            align_done_pub_->publish(std_msgs::msg::Empty());
        }

        if (movement_in_progress_) {
            return;
        }

        if (!latest_lines_) {
            std::cout << "Don't see any lines. Aborting step.\n";
            return;
        }

        if (move_until_time_.has_value()) {
            if (this->now() < move_until_time_.value()) {
                return;
            } else {
                move_until_time_.reset();
            }
        }

        const auto& lines = latest_lines_->segments;

        if (state_ <= 1) {
            if (!target_m_ || !corner_) {
                std::cout << "No valid cell center found from line detection. Turning...\n";
                execute_moves_and_wait(::turn(90.0));
                return;
            }
            locked_target_m_ = target_m_.value();
            locked_corner_ = corner_.value();

            std::cout << "O: [" << corner_->x() << ", " << corner_->y()
                      << "], M: [" << target_m_->x() << ", " << target_m_->y() << "]\n";

            auto marker_O = marker_from_point(corner_->x(), corner_->y(), this, "base_link", 0.0, 1.0, 0.0);
            auto marker_M = marker_from_point(target_m_->x(), target_m_->y(), this, "base_link", 1.0, 0.0, 0.0);

            marker_publisher_->publish(marker_M);
            marker_publisher2_->publish(marker_O);

            target_frame_ = latest_lines_->header.frame_id;
            state_ = 2;
            std::cout << "Transition to state " << state_ << "\n";
            return;

        } else if (state_ == 2) {
            double x = locked_target_m_.x();
            double y = locked_target_m_.y();
            double deg = std::atan2(y, x) * 180.0 / M_PI;
            double dist = std::sqrt(x * x + y * y);

            // Discard any leftover moves from previous step
            clear_pending_queue();

            std::vector<DriveInstruction> combined_moves;

            if (dist > 0.1 && !has_travelled) {
                printf("Deg to target: %f, dist to target: %f\n", deg, dist);
                combined_moves = ::drive_to_point(x*100, y*100);
                state_ = 1;
            }
            else {
                if (mode_ == 1) {
                    double corner_x = locked_corner_.x();
                    double corner_y = locked_corner_.y();
                    double corner_deg = std::atan2(corner_y, corner_x) * 180.0 / M_PI;

                    auto turn_moves = ::turn(corner_deg - 45.0 - deg);
                    combined_moves.insert(combined_moves.end(), turn_moves.begin(), turn_moves.end());

                    state_ = 4;
                    std::cout << "Mode 1: Corner alignment executed. Transition to state 4\n";
                } else if (mode_ == 2) {
                    state_ = 8;
                    std::cout << "Mode 2: Transition to state 8\n";
                } else if (mode_ == 3) {
                    state_ = 3;
                    std::cout << "Mode 3: Transition to state 3\n";
                } else {
                    state_ = 4;
                    std::cout << "Invalid mode. Aborting (Transition to state 4)\n";
                }
            }

            execute_moves_and_wait(combined_moves);                                                                                                   
            has_travelled = true;
            return;
        } else if (state_ == 8) {
            double deg2 = line_deg_offset();
            std::cout << "turn: " << deg2 << "\n";
            if (std::abs(deg2) < 5.0) {
                state_ = 4;
                std::cout << "We stand correct! Transition to state 4\n";
                clear_pending_queue();
                abort_move_req(abort_move_client_);
                return;
            }
            std::cout << "Turning!\n";
            execute_moves_and_wait(::turn(deg2));
        }
        else if (state_ == 3) {
            //double min_deg = std::numeric_limits<double>::infinity();
            double min_deg = 0;
			int count_degs = 0;
            double alt_min_deg = std::numeric_limits<double>::infinity();
            for (const auto& line : lines) {
                Eigen::Vector2d p1(line.start_x, line.start_y);
                Eigen::Vector2d p2(line.end_x, line.end_y);

                Eigen::Vector2d v = p2 - p1;
                double n_v = v.norm();

                if (n_v == 0) {
                    continue;
                }

                // double t_c = std::max(0.0, std::min(1.0, (-p1).dot(v) / (n_v * n_v)));
                double t_c = (-p1).dot(v) / (n_v * n_v);
                Eigen::Vector2d cp_c = p1 + t_c * v;

                if (cp_c.norm() >= 0.38) {
                    double deg = std::atan2(cp_c.y(), cp_c.x()) * 180.0 / M_PI;
					while (deg >= 45) deg -= 90;
					while (deg <= -45) deg += 90;
					printf("trying wall (min_deg: %f): start: %f, %f;end: %f, %f, deg_norm: %f, deg: %f: New best? \n", min_deg, line.start_x, line.start_y, line.end_x, line.end_y, cp_c.norm(), deg);
					min_deg += deg;
					count_degs++;

                    //if (std::abs(deg) < std::abs(min_deg)) {
					//	printf("YES!\n");
                    //    min_deg = deg;
                    //} else {
					//	printf("NO!\n");
					//}
                }
            }
			printf("min_deg: (before) %f", -min_deg);
			min_deg /= count_degs;
			printf(", (after) %f\n", -min_deg);

            if (std::abs(min_deg) < 3 || has_turned) {
                state_ = 4;
                std::cout << "Transition to state 4 (finished)\n";

                clear_pending_queue();
                abort_move_req(abort_move_client_);
                return;
            }

            if (min_deg == std::numeric_limits<double>::infinity()) {
                min_deg = 45.0;
            }
            else {
                has_turned = true;
            }

            std::cout << "Moving along min deg " << -min_deg << "\n";
            execute_moves_and_wait(::turn(-min_deg));
        }
    }

    int mode_;
    int state_;
    bool align_done_published_ = false;
    double wheel_radius_;
    double wheel_dist_mid_;
    rclcpp::CallbackGroup::SharedPtr routine_cb_group_;
    Eigen::Vector2d locked_target_m_;
    Eigen::Vector2d locked_corner_;
    std::optional<Eigen::Vector2d> target_m_;
    std::optional<Eigen::Vector2d> corner_;
    std::optional<rclcpp::Time> move_until_time_;
    std::string target_frame_;
    
    red_fundamentals::msg::LineSegmentArray::SharedPtr latest_lines_;

    rclcpp::Subscription<red_fundamentals::msg::LineSegmentArray>::SharedPtr subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr center_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr corner_subscription_;
    
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher2_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr align_done_pub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr trigger_align_sub_;
    
    rclcpp::Client<red_fundamentals::srv::AddMove>::SharedPtr add_move_client_;
    rclcpp::Client<red_fundamentals::srv::AbortMove>::SharedPtr abort_move_client_;
    rclcpp::Client<red_fundamentals::srv::ClearMoveQueue>::SharedPtr clear_move_client_;

    std::vector<rclcpp::Client<red_fundamentals::srv::AddMove>::SharedFuture> futures_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    int mode = 3;
    if (argc > 1) {
        try {
            mode = std::stoi(argv[1]);
        } catch (const std::exception& e) {
            std::cout << "Invalid argument provided (1, 2 or 3). Defaulting to 3.\n";
        }
    }

    align = std::make_shared<Align>(mode);
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(align);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}

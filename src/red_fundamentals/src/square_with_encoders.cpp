#ifndef RCLCPP
#define RCLCPP

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/utilities.hpp>

#endif

#define MOVEMENT_IMPLEMENTATION
#include "movement.hpp"

using namespace std::chrono_literals;

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	
	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("square_with_encoders");
	rclcpp::Client<AddMove>::SharedPtr move = node->create_client<AddMove>("add_move");
	rclcpp::Client<ClearQueue>::SharedPtr clear_queue_client = node->create_client<ClearQueue>("clear_move_queue");
	rclcpp::Client<AbortMove>::SharedPtr abort_move_client = node->create_client<AbortMove>("abort_move");

	while (!move->wait_for_service(1s)) {
		if (!rclcpp::ok()) {
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
			return 0;
		}
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
	}

	send_async(drive(100), move);
	send_async(turn(90), move);
	send_async(drive(100), move);
	send_async(turn(90), move);
	send_async(drive(100), move);
	send_async(turn(90), move);
	send_async(drive(100), move);
	send_async(turn(90), move);

	rclcpp::spin(node);

	clear_queue_req(clear_queue_client);
	abort_move_req(abort_move_client);
	rclcpp::shutdown();
	return 0;
}

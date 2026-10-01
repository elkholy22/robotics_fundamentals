#ifndef RCLCPP
#define RCLCPP

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/utilities.hpp>

#endif

#define MOVEMENT_IMPLEMENTATION
#include "movement.hpp"

using namespace std::chrono_literals;

std::vector<rclcpp::Client<AddMove>::SharedFuture> futures;
void send(std::vector<DriveInstruction> moves, rclcpp::Client<AddMove>::SharedPtr client) {
	// Send all requests
	printf("sending move: \n");
	for (const auto& m : moves) {
		auto request = std::make_shared<AddMove::Request>();
		request->left = m.left;
		request->right = m.right;
		printf("(%f, %f), ", m.left, m.right);

		futures.push_back(client->async_send_request(request).future.share());
	}
	printf("\n");
}

bool await_moves(std::shared_ptr<rclcpp::Node> node) {
	printf("now awaiting moves: %ld\n", futures.size());
	for (auto& future : futures) {
		rclcpp::spin_until_future_complete(node, future);
		auto response = future.get();
		printf("move done with: %d\n", response->ret);
		if (response->ret != MOVE_RETURN_SUCCESS) {
			futures.clear();
			return false;
		}
	}
	futures.clear();
	printf("done awaiting moves\n");
	return true;
}

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);

	float X[argc/3], Y[argc/3], deg[argc/3];
	for (int i=1; i < argc; i+=3) {
		X[i/3] = std::stof(argv[i]);
		Y[i/3] = std::stof(argv[i+1]);
		deg[i/3] = std::stof(argv[i+2]);
	}
	
	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("movement_test");
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

	for(int i=0; i < argc/3; i++)
		send(drive_to_pose(X[i], Y[i], deg[i]), move);

	if (await_moves(node) == false)
		RCLCPP_INFO(node->get_logger(), "Plan failed.");
	else
		RCLCPP_INFO(node->get_logger(), "Plan executed successfully.");
	return 0;

	rclcpp::spin(node);

	clear_queue_req(clear_queue_client);
	abort_move_req(abort_move_client);
	rclcpp::shutdown();
	return 0;
}

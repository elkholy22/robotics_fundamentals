// This follows the header-only principle with MOVEMENT_IMPLENTATION

#ifndef RCLCPP
#define RCLCPP

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/utilities.hpp>

#endif

#include "red_fundamentals/srv/add_move.hpp"
#include "red_fundamentals/srv/clear_move_queue.hpp"
#include "red_fundamentals/srv/abort_move.hpp"

#define _USE_MATH_DEFINES 
#include <math.h>

using AddMove = red_fundamentals::srv::AddMove;
using ClearQueue = red_fundamentals::srv::ClearMoveQueue;
using AbortMove = red_fundamentals::srv::AbortMove;

constexpr const static float WHEEL_RADIUS = 3.2;
constexpr const static float WHEEL_DIST_MID = 25.8;

struct DriveInstruction {
	float left = 0;
	float right = 0;
};

enum MoveReturn {
	MOVE_RETURN_SUCCESS = 0,
	MOVE_RETURN_REPLACED = 1,
	MOVE_RETURN_ABORTED_MOVING = 2,
	MOVE_RETURN_ABORTED_QUEUING = 3,
	MOVE_RETURN_NOT_FINISHED = 4 // This should not be returned
};

// void send(DriveInstruction d, rclcpp::Client<AddMove>::SharedPtr client);
std::vector<MoveReturn> send_sync(std::vector<DriveInstruction> moves, rclcpp::Client<AddMove>::SharedPtr client, rclcpp::Node::SharedPtr node);
void send_async(std::vector<DriveInstruction> moves, rclcpp::Client<AddMove>::SharedPtr client);
void clear_queue_req(rclcpp::Client<ClearQueue>::SharedPtr client);
void abort_move_req(rclcpp::Client<AbortMove>::SharedPtr client);
std::vector<DriveInstruction> turn(float deg);
std::vector<DriveInstruction> drive(float dist);
std::vector<DriveInstruction> drive_to_point(float x, float y);
std::vector<DriveInstruction> drive_to_point_direct(float x, float y);
std::vector<DriveInstruction> drive_to_pose(float x, float y, float deg);
std::vector<DriveInstruction> _bi_arc(float x, float y, float deg);

#ifndef MOVEMENT_IMPLEMENTATOIN
#define MOVEMENT_IMPLEMENTATION


//rclcpp::detail::FutureAndRequestId<AddMove::Request> 
std::vector<MoveReturn> send_sync(std::vector<DriveInstruction> moves, rclcpp::Client<AddMove>::SharedPtr client, rclcpp::Node::SharedPtr node) {
    std::vector<MoveReturn> return_codes;
    std::vector<rclcpp::Client<AddMove>::SharedFuture> futures;

    for (const auto& m : moves) {
        auto request = std::make_shared<AddMove::Request>();
        request->left = m.left;
        request->right = m.right;

        futures.push_back(client->async_send_request(request).future.share());
    }

    for (auto& future : futures) {
        future.wait();
        auto response = future.get();
        return_codes.push_back(static_cast<MoveReturn>(response->ret));
    }

    return return_codes;
}

std::vector<MoveReturn> send_sync_executor(std::vector<DriveInstruction> moves, rclcpp::Client<AddMove>::SharedPtr client, std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor) {
	std::vector<MoveReturn> return_codes;
	std::vector<rclcpp::Client<AddMove>::SharedFuture> futures;

	// Send all requests
	for (const auto& m : moves) {
		auto request = std::make_shared<AddMove::Request>();
		request->left = m.left;
		request->right = m.right;


		futures.push_back(client->async_send_request(request).future.share());
	}

	// Wait for all responses
	for (auto& future : futures) {
		if (executor->spin_until_future_complete(future) == rclcpp::FutureReturnCode::SUCCESS) {
			auto response = future.get();
			return_codes.push_back(static_cast<MoveReturn>(response->ret));
		} else {
			return_codes.push_back(MOVE_RETURN_NOT_FINISHED); // Unknown error awaiting the future
		}
	}

	return return_codes;
}


void send_async(std::vector<DriveInstruction> moves, rclcpp::Client<AddMove>::SharedPtr client) {
	for (DriveInstruction m : moves) {
		auto request = std::make_shared<AddMove::Request>();

		request->left = m.left;
		request->right = m.right;

		client->async_send_request(request);

	}
}

void clear_queue_req(rclcpp::Client<ClearQueue>::SharedPtr client) {
	auto request = std::make_shared<ClearQueue::Request>();
	client->async_send_request(request);
}

void abort_move_req(rclcpp::Client<AbortMove>::SharedPtr client) {
	auto request = std::make_shared<AbortMove::Request>();
	client->async_send_request(request);
}

std::vector<DriveInstruction> turn(float deg) {
	const float dist = (deg / 360) * M_PI * WHEEL_DIST_MID;
	return {{dist, -dist}};
}

std::vector<DriveInstruction> drive(float dist) {
		return {{dist, dist}};
}

std::vector<DriveInstruction> drive_to_point(float x, float y) {
	// atan returns rad, so we convert to deg
	float radius = std::atan2(x, y) * 180.0 / M_PI;
	float dist = sqrt(x*x + y*y);
	return {turn(radius)[0], drive(dist)[0]};
}

std::vector<DriveInstruction> drive_to_point_direct(float x, float y) {
	// radius of the circle which the robot center will drive
	float radius = (x*x + y*y) / (2.0*x);
	float turn_angle = 2.0 * std::atan2(std::fabs(x), y);
	float center_arc_dist = std::abs(radius) * turn_angle;

	float left = center_arc_dist * (1.0 + WHEEL_DIST_MID / (2.0 * radius));
	float right = center_arc_dist * (1.0 - WHEEL_DIST_MID / (2.0 * radius));
	return {{left, right}};
}

// We drive to the relative position x and y and
// at the end we have turned ourself deg degrees
std::vector<DriveInstruction> drive_to_pose(float x, float y, float deg) {
	// if x and y are 0 we just want to turn
	if (x == 0 && y == 0) return turn(deg);
	// for small x we just want to drive forward
	if (abs(x) < 1) return drive(y);

	// We either drive only one one curve or use an bi-arc algo to turn in 2 curves
	// Check if single curve works
	float turn_angle = 2.0 * std::atan2(std::fabs(x), y);
	// heading at the end of the single curve
	turn_angle *= x > 0 ? 1 : -1;
	// TODO test without fabs(x)

	float rad = deg * M_PI / 180.0;
	float diff = atan2(sin(rad - turn_angle), cos(rad - turn_angle));
	printf("driving_to_pose: diff: %f\n", diff);

	if (fabs(diff) < 0.01) return drive_to_point_direct(x, y);

	return _bi_arc(x, y, deg);

}

std::vector<DriveInstruction> _bi_arc(float x, float y, float deg) {
	float rad = deg * M_PI / 180.0;
	float dist = sqrt(x*x + y*y);
	float n2x = cos(rad);
	float n2y = -sin(rad);
	
	std::vector<float> candidates;
	for (int i=-200; i <= 200; i++)
		candidates.push_back(i * dist / 50.0);
	candidates.push_back(dist * 10.0);
	candidates.push_back(dist * -10.0);
	candidates.push_back(dist * 50.0);
	candidates.push_back(dist * -50.0);

	float best_len = INFINITY;
	std::vector<DriveInstruction> best_path;

	for (float R1 : candidates) {
		if (fabs(R1) < 0.001) continue;

		float C1x = R1, C1y = 0;
		float Dx = x - R1;
		float Dy = y;
		float denom = 2.0 * (Dx * n2x + Dy * n2y + R1);
		if (fabs(denom) < 1e-6) {
			continue;
		}

		float R2 = (R1*R1 - (Dx*Dx + Dy*Dy)) / denom;
		if (fabs(R2) < 1e-3) {
			continue;
		}

		float C2x = x + R2 * n2x;
		float C2y = y + R2 * n2y;

		float Rdiff = R1 - R2;
		if (fabs(Rdiff) < 1e-6) {
			continue;
		}

		float t = R1 / Rdiff;
		float P1x = C1x + t * (C2x - C1x);
		float P1y = C1y + t * (C2y - C1y);
		if (P1y < -0.1f) {
			continue;
		}


		// turn 1
		float beta = atan2(P1x, P1y);
		float DeltaTheta1 = 2.0 * beta;
		if (R1 * DeltaTheta1 < 0) {
			if (R1 > 0) DeltaTheta1 += 2.0 * M_PI;
			else DeltaTheta1 -= 2.0 * M_PI;
		}
		float L1 = R1 * DeltaTheta1;
		if (L1 <= 0) {
			continue;
		}
		DriveInstruction di1 = {
			L1 * (1.0f + WHEEL_DIST_MID / (2.0f * R1)),
			L1 * (1.0f - WHEEL_DIST_MID / (2.0f * R1)),
		};
		if (di1.left < 0 || di1.right < 0) continue;

		// turn 2
		float phi1 = beta * 2.0;
		float dphi = rad - phi1;
		dphi = atan2(sin(dphi), cos(dphi));
		float DeltaTheta2 = dphi;
		if (R2 * DeltaTheta2 < 0) {
			if (R2 > 0) DeltaTheta2 += 2.0 * M_PI;
			else DeltaTheta2 -= 2.0 * M_PI;
		}
		float L2 = R2 * DeltaTheta2;
		if (L2 <= 0) {
			continue;
		}
		DriveInstruction di2 = {
			L2 * (1.0f + WHEEL_DIST_MID / (2.0f * R2)),
			L2 * (1.0f - WHEEL_DIST_MID / (2.0f * R2)),
		};
		if (di2.left < 0 || di2.right < 0) continue;

		float drive_dist_ratio = (di1.left + di1.right) / (di2.left + di2.right);
		if (drive_dist_ratio < 0.5 || drive_dist_ratio > 2.0) continue;

		float total = L1 + L2;
		if (total < best_len) {
			best_len = total;
			best_path.clear();
			best_path.push_back(di1);
			best_path.push_back(di2);
		}
	}
	
	if (best_path.empty())
		printf("WARN: _bi_arc could not find a single drivable instruction! This should NOT happen! \n");
	return best_path;
}

#endif

#ifndef RCLCPP
#define RCLCPP
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/utilities.hpp>
#endif

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "create_fundamentals/srv/diff_drive.hpp"
#include "create_fundamentals/msg/sensor_packet.hpp"
#include "red_fundamentals/srv/add_move.hpp"
#include "red_fundamentals/srv/clear_move_queue.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
// #include "visualization_msgs/visualization_msgs/msg/marker_array.hpp"
#define MOVEMENT_IMPLEMENTATION
#include "movement.hpp"

#include <chrono>
#include <queue>
#include <condition_variable>
#include <thread>

#define INFO(...) RCLCPP_INFO(rclcpp::get_logger("rclcpp"), __VA_ARGS__);

using std::placeholders::_1;
using namespace std::chrono_literals;

using SensorPacket = create_fundamentals::msg::SensorPacket;
using DiffDrive = create_fundamentals::srv::DiffDrive;
using PointCloud2 = sensor_msgs::msg::PointCloud2;

struct QueueType {
	DriveInstruction val;
	int id;
};

void move();
void diff_drive(const float left, const float right);
void add_move(const std::shared_ptr<AddMove::Request> req, std::shared_ptr<AddMove::Response> resp);
void replace_move(const std::shared_ptr<AddMove::Request> req, std::shared_ptr<AddMove::Response> resp);
void clear_queue();
void abort_move(MoveReturn ret);

float NEG_MULTIPLIER = 0.95;
float SPEED_MIN = 3;
float SPEED_MIN_HARD = 1;
float SPEED_MAX = 50; // This was reverse engineered by looking at the src code ;)
float DRIVE_STRAIGHT_OFFSET = 2.8;
float STRAIGHT_MULTIPLIER = 2;
float CHASSIE_RADIUS = 15;
float X_PUFFER = -1;
float Y_PUFFER = 3;
float DIST_PUFFER = 0;

std::deque<QueueType> queue;
MoveReturn move_return_code = MOVE_RETURN_NOT_FINISHED;
int move_return_id = -1;
int global_id_counter = 0;
std::mutex mtx;
std::condition_variable move_finished_cv;
std::condition_variable code_read_cv;
std::condition_variable moving_cv;

rclcpp::CallbackGroup::SharedPtr move_callback_group, cancel_callback_group, cloud_callback_group;
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
rclcpp::Client<DiffDrive>::SharedPtr diff_drive_client_;

void print_queue() {
	printf("Queue: ");
	for (int i=0 ; i < queue.size(); i++) {
		printf("%f, %f", queue[i].val.left, queue[i].val.right);
	}
	printf("\n");
}

void move() {
	while(true) {
		DriveInstruction instr;

		{
			std::unique_lock<std::mutex> lock(mtx);
			moving_cv.wait(lock, []{ 
				return queue.size() > 0; 
			});

			move_return_code = MOVE_RETURN_NOT_FINISHED;

			instr = queue.front().val;

			static const float eps = 0.01;
			if (abs(instr.left) < eps && abs(instr.right) < eps) {
				printf("skipping move (%f, %f)\n", instr.left, instr.right);


				move_return_code = MOVE_RETURN_SUCCESS;
				move_return_id = queue.front().id;
				queue.pop_front();
				// wake up the add_move and replace_move threads
				move_finished_cv.notify_all();
				// sleep until someone read
				code_read_cv.wait(lock, []{ return move_return_code == MOVE_RETURN_NOT_FINISHED; });
				
				continue;
			}

			printf("New Move (%f, %f)!\n", instr.left, instr.right);
			print_queue();
		}

		const float diff_mult = instr.left / instr.right;

		// if (abs(diff_mult) == 1) {
		// 	// for straight passages we want to drive further than we think,
		// 	// this is not needed in corners (idk why either ok)
		// 	instr.left += DRIVE_STRAIGHT_OFFSET;
		// 	instr.right += DRIVE_STRAIGHT_OFFSET;
		// }

		if (instr.left < 0) instr.left *= NEG_MULTIPLIER;
		if (instr.right < 0) instr.right *= NEG_MULTIPLIER;

		float speed_left = SPEED_MIN * (instr.left < 0 ? -1 : 1) * sqrt(abs(diff_mult));
		float speed_right = SPEED_MIN * (instr.right < 0 ? -1 : 1) * 1/(sqrt(abs(diff_mult)));


		// we drive faster in the straights
		
		if (abs(diff_mult) == 1) {
			speed_left *= STRAIGHT_MULTIPLIER;
			speed_right *= STRAIGHT_MULTIPLIER;
		}

		// floor max limit
		if (abs(speed_left) > SPEED_MAX && abs(speed_left) > abs(speed_right)) {
			float max_multiplier = SPEED_MAX/abs(speed_left);
			speed_left *= max_multiplier;
			speed_right *= max_multiplier;
		} else if (abs(speed_right) > SPEED_MAX && abs(speed_right) > abs(speed_left)) {
			float max_multiplier = SPEED_MAX/abs(speed_right);
			speed_left *= max_multiplier;
			speed_right *= max_multiplier;
		}

		// floor min limit
		if (abs(speed_left) < SPEED_MIN_HARD) speed_left = SPEED_MIN_HARD * (speed_left < 0 ? -1 : 1);
		if (abs(speed_right) < SPEED_MIN_HARD) speed_right = SPEED_MIN_HARD * (speed_right < 0 ? -1 : 1);
		
		const float time_needed = instr.left / speed_left;
		// We only want delay on straights (dont ask me why)
        const std::chrono::duration<float> sleep_duration(time_needed);
		printf("instr: (%f, %f), diff_mult: %f, speed: (%f, %f), time_needed: %f, sleep duration: %f\n", instr.left, instr.right, diff_mult, speed_left, speed_right, time_needed, sleep_duration.count());
		diff_drive(speed_left, speed_right);

		{
			printf("starting move with (orig): %d\n", move_return_code);
			std::unique_lock<std::mutex> lock(mtx);
			auto start = std::chrono::steady_clock::now();
			auto result = moving_cv.wait_for(lock, sleep_duration, []{ 
				return move_return_code != MOVE_RETURN_NOT_FINISHED; 
			});
			auto end = std::chrono::steady_clock::now();
			auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
			printf("wait_for returned after %ld ms (expected %f ms), result: %s, move_return_code: %d\n", elapsed.count(), sleep_duration.count() * 1000, result ? "predicate true" : "timeout", move_return_code);

			if (move_return_code != MOVE_RETURN_REPLACED) {
            	diff_drive(0, 0);
            	printf("stopped move with (orig): %d\n", move_return_code);
        	} else {
            	printf("replaced move smoothly, skipping stop.\n");
        	}

			if (move_return_code == MOVE_RETURN_NOT_FINISHED)
				move_return_code = MOVE_RETURN_SUCCESS;
			printf("stopped move with: %d\n", move_return_code);


			move_return_id = queue.front().id;
			queue.pop_front();

			// wake up the add_move and replace_move threads
			move_finished_cv.notify_all();
			// sleep until someone read
			code_read_cv.wait(lock, []{ return move_return_id == -1; });
		}
	}
}

// left and right are in cm/s TODO
void diff_drive(const float left, const float right) {
	auto request = std::make_shared<DiffDrive::Request>();
	request->left = left/3.15;
	request->right = right/3.15;
	INFO("diff_drive: %f (%f), %f (%f)\n", request->left, left, request->right, right);
	diff_drive_client_->async_send_request(request);
}

void add_move(const std::shared_ptr<AddMove::Request> req, std::shared_ptr<AddMove::Response> resp) {
	printf("add_move called: (%f, %f)\n", req->left, req->right);
	std::unique_lock<std::mutex> lock(mtx);

	int id = global_id_counter++;
	queue.push_back({{(float) req->left, (float) req->right}, id});

	moving_cv.notify_all();

	// sleep until the move is finished
	move_finished_cv.wait(lock, [id]{ 
		return move_return_code != MOVE_RETURN_NOT_FINISHED && move_return_id == id; 
	});

	resp->ret = (int) move_return_code;
	move_return_code = MOVE_RETURN_NOT_FINISHED;
	move_return_id = -1;

	// wake up return_move_code
	code_read_cv.notify_all();
	printf("add_move id=%d returns %d\n", id, resp->ret);
	return;
}

void replace_move(const std::shared_ptr<AddMove::Request> req, std::shared_ptr<AddMove::Response> resp) {
	printf("Replacement move! (%f, %f)\n", req->left, req->right);

	std::unique_lock<std::mutex> lock(mtx);

	if (!queue.empty()) {
        abort_move(MOVE_RETURN_REPLACED);
    }
	
	int id = global_id_counter++;
	
	if (!queue.empty()) {
        queue.insert(queue.begin() + 1, {{(float) req->left, (float) req->right}, id});
    } else {
        queue.push_back({{(float) req->left, (float) req->right}, id});
    }

	moving_cv.notify_all();
	printf("Awaiting movement finish \n");
	// sleep until the move is finished
	move_finished_cv.wait(lock, [id]{ 
		return move_return_code != MOVE_RETURN_NOT_FINISHED && move_return_id == id; 
	});

	resp->ret = (int) move_return_code;
	move_return_code = MOVE_RETURN_NOT_FINISHED;
	move_return_id = -1;

	//wake up return_move_code
	code_read_cv.notify_all();
	printf("add_move (replaced) id=%d returns %d\n", id, resp->ret);
	return;
}

void clear_queue_(const std::shared_ptr<ClearQueue::Request> req, std::shared_ptr<ClearQueue::Response> resp) {
	clear_queue();
}

void clear_queue() {
	printf("Clearing Queue... ");
	while(queue.size() > 1) {
		std::unique_lock<std::mutex> lock(mtx);

		move_return_code = MOVE_RETURN_ABORTED_QUEUING;
		move_return_id = queue.back().id;

		// wake up the add_move and replace_move threads
		move_finished_cv.notify_all();
		// sleep until someone read
		code_read_cv.wait(lock, []{ return move_return_id == -1; });

		queue.pop_back();
	}
	print_queue();
	printf("done.\n");
	return;
}

void abort_move_(const std::shared_ptr<AbortMove::Request> req, std::shared_ptr<AbortMove::Response> resp) {
	abort_move(MOVE_RETURN_ABORTED_MOVING);
}

void abort_move(MoveReturn ret) {
    printf("Aborting Move... ");
    
    // Only overwrite the return code if we are not currently in a handoff
    if (move_return_id == -1) {
        move_return_code = ret;
        moving_cv.notify_all();
    }
    
    if (ret != MOVE_RETURN_REPLACED) {
        diff_drive(0, 0); 
    }
    
    printf("done.\n");
    return;
}

void cloud_callback(const PointCloud2::SharedPtr msg) {
	// We only scan for obstacles when we are moving and not turning
	if (queue.size() == 0) return;
	const float diff_mult = queue.front().val.left / queue.front().val.right;
	if (diff_mult == -1) return;

	sensor_msgs::PointCloud2ConstIterator<float> x_iter(*msg, "x");
	sensor_msgs::PointCloud2ConstIterator<float> y_iter(*msg, "y");

	for(; x_iter != x_iter.end(); ++x_iter, ++y_iter) {
		const float x = *x_iter*100;
		const float y = *y_iter*100;
		const float dist = abs(sqrt(x*x+y*y));
		if (abs(x) > CHASSIE_RADIUS + X_PUFFER) continue;
		if (y > CHASSIE_RADIUS + Y_PUFFER || y < 0) continue;
		if (dist > CHASSIE_RADIUS + DIST_PUFFER) continue;

		INFO("OBSTACLE IN FRONT! At: (%f, %f) => dist: %f (<= %f)", x, y, dist, CHASSIE_RADIUS+DIST_PUFFER);
		clear_queue();
		abort_move(MOVE_RETURN_ABORTED_MOVING);
		print_queue();
		return;
	}
}

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);

	NEG_MULTIPLIER = std::stof(argv[1]);
	SPEED_MIN_HARD = std::stof(argv[2]);
	SPEED_MIN = std::stof(argv[3]);
	SPEED_MAX = std::stof(argv[4]);
	DRIVE_STRAIGHT_OFFSET = std::stof(argv[5]);
	STRAIGHT_MULTIPLIER = std::stof(argv[6]);
	CHASSIE_RADIUS = std::stof(argv[7]);
	X_PUFFER = std::stof(argv[8]);
	Y_PUFFER = std::stof(argv[9]);
	DIST_PUFFER = std::stof(argv[10]);

	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_move");

	diff_drive_client_ = node->create_client<DiffDrive>("diff_drive");

	while (!diff_drive_client_->wait_for_service(1s)) {
		if (!rclcpp::ok()) {
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for diff_drive");
			exit(1);
		}
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "waiting for diff_drive...");
	}

	cancel_callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	move_callback_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
	cloud_callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	
	rclcpp::SubscriptionOptions cloud_options;
	cloud_options.callback_group = cloud_callback_group;
	cloud_sub_ = node->create_subscription<PointCloud2>("point_cloud", 10, std::bind(&cloud_callback, _1), cloud_options);

	rclcpp::Service<AddMove>::SharedPtr add_move_srv = node->create_service<AddMove>("add_move", add_move, rmw_qos_profile_services_default, move_callback_group);
	rclcpp::Service<AddMove>::SharedPtr replace_move_srv = node->create_service<AddMove>("replace_move", replace_move, rmw_qos_profile_services_default, cancel_callback_group);
	rclcpp::Service<ClearQueue>::SharedPtr clear_queue_srv = node->create_service<ClearQueue>("clear_move_queue", clear_queue_, rmw_qos_profile_services_default, cancel_callback_group);
	rclcpp::Service<AbortMove>::SharedPtr abort_move_srv = node->create_service<AbortMove>("abort_move", abort_move_, rmw_qos_profile_services_default, cancel_callback_group);

	rclcpp::ExecutorOptions options;
	rclcpp::executors::MultiThreadedExecutor executor(options, 50); // spawn 50 threads
	executor.add_node(node);

	diff_drive(0, 0);

	std::thread move_thread(move);

	executor.spin();

	// TODO we dont join because we dont have to, its not trivial to check when to shutdown
	// move_thread.join();
	rclcpp::shutdown();
	return 0;
}

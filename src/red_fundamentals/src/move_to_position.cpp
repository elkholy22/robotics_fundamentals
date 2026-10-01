#ifndef RCLCPP
#define RCLCPP
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/utilities.hpp>
#endif

#include <queue>
#include <map>
#include <algorithm>
#include "map_and_messages/msg/grid.hpp"
#include "map_and_messages/msg/cell.hpp"
#include "map_and_messages/msg/pose.hpp"
#include "red_fundamentals/srv/execute_plan.hpp"
#include "red_fundamentals/srv/move_to_position.hpp"

using Grid = map_and_messages::msg::Grid;
using Cell = map_and_messages::msg::Cell;
using Pose = map_and_messages::msg::Pose;
using ExecutePlan = red_fundamentals::srv::ExecutePlan;
using MoveToPosition = red_fundamentals::srv::MoveToPosition;

struct Pos {
	int x = 0, y = 0;
};

enum DIRECTION {
	RIGHT,
	TOP,
	LEFT,
	BOTTOM
};

using std::placeholders::_1;
using namespace std::chrono_literals;

void print_path(const std::vector<Pos> path);
void map_callback(const Grid& msg);
void pose_callback(const Pose& msg);
void move_to_position(const std::shared_ptr<MoveToPosition::Request> req, std::shared_ptr<MoveToPosition::Response> resp);
std::optional<std::vector<Pos>> get_best_path(Pos dest);
std::vector<int> path_to_plan(std::vector<Pos> path, Pos cur);
bool pos_in_path(std::vector<Pos> path, Pos pos);
std::optional<std::vector<Pos>> best_path(std::optional<std::vector<Pos>> best, std::vector<Pos> cur, Pos curpos, Pos target);

rclcpp::Subscription<Grid>::SharedPtr map_sub_;
rclcpp::Client<ExecutePlan>::SharedPtr execute_plan_;
rclcpp::Service<MoveToPosition>::SharedPtr move_to_position_;
rclcpp::Subscription<Pose>::SharedPtr pose_sub_;

std::optional<Grid> grid;
std::optional<Pose> pose;

// we have to store the response globally so it doesnt go out of scope instantly
std::shared_ptr<MoveToPosition::Response> pending_response_;

void print_path(const std::vector<Pos> path) {
	printf("[");
	for(int i=0; i < path.size(); i++) {
		printf("{%d, %d}, ", path[i].x, path[i].y);
	}
	printf("]");
}

void print_plan(const std::vector<int> plan) {
	printf("[");
	for(int i=0; i < plan.size(); i++) {
		printf("%d, ", plan[i]);
	}
	printf("]\n");
}

void map_callback(const Grid& msg) {
	if (grid.has_value()) return;
	printf("Received a new Map!\n");
	grid = msg;
}

void pose_callback(const Pose& msg) {
	// if (pose.has_value()) return;
	printf("Received a new Pose! {%d, %d, %d}\n", msg.column, msg.row, msg.orientation);
	pose = msg;
}

void move_to_position(const std::shared_ptr<MoveToPosition::Request> req, std::shared_ptr<MoveToPosition::Response> resp) {
	if (!grid.has_value()) {
		printf("cant drive, no map\n");
		resp->success = false;
		return;
	}
	if (!pose.has_value()) {
		printf("cant drive, no pose\n");
		resp->success = false;
		return;
	}

	// get path
	const Pos cur = {pose.value().column, pose.value().row};
	const Pos dest = {req->column, req->row};
	
	const auto path = get_best_path(dest);
	if (!path.has_value()) { resp->success = false; return; }

	//convert path to plan
	const auto plan = path_to_plan(*path, cur);
	print_plan(plan);

	printf("Waiting for execute_plan to finish...\n");
	auto request = std::make_shared<ExecutePlan::Request>();
	request->plan = plan;
    auto future = execute_plan_->async_send_request(request).future;
	future.wait();
	
	resp->success = future.get()->success;
	printf("Executed plan successfully? %b!\n", resp->success);
}

std::vector<int> path_to_plan(std::vector<Pos> path, Pos cur) {
	std::vector<int> plan;
	for(int i=0; i < path.size(); i++) {
		int x_diff, y_diff;
		if (i == 0) {
			x_diff = path[0].x - cur.x;
			y_diff = path[0].y - cur.y;
		} else {
			x_diff = path[i].x - path[i-1].x;
			y_diff = path[i].y - path[i-1].y;
		}
		if (x_diff == -1 && y_diff == 0) plan.push_back(LEFT);
		else if (x_diff == 1 && y_diff == 0) plan.push_back(RIGHT);
		else if (x_diff == 0 && y_diff == -1) plan.push_back(TOP);
		else if (x_diff == 0 && y_diff == 1) plan.push_back(BOTTOM);
	}
	return plan;
}

bool pos_in_path(std::vector<Pos> path, Pos pos) {
	auto ret = std::find_if(path.begin(), path.end(), [pos](const Pos& p) {return pos.x == p.x && pos.y == p.y;});
	return ret != path.end();
}

std::optional<std::vector<Pos>> get_best_path(Pos dest) {
    const Pos start = {pose.value().column, pose.value().row};
    printf("Getting the path (BFS)...\n");

    if (!grid.has_value()) return std::nullopt;
    const int rows = (int)grid.value().rows.size();
    const int cols = (int)grid.value().rows[0].cells.size();

    auto in_bounds = [&](Pos p){ return p.x >= 0 && p.x < cols && p.y >= 0 && p.y < rows; };
    auto key = [&](Pos p){ return p.y * cols + p.x; };

    std::queue<Pos> q;
    std::map<int, Pos> came_from;   // child key to parent
    std::map<int, bool> visited;

    q.push(start);
    visited[key(start)] = true;

    bool found = (start.x == dest.x && start.y == dest.y);

    while (!q.empty() && !found) {
        Pos c = q.front(); q.pop();

        const auto walls = grid.value().rows[c.y].cells[c.x].walls;
        bool top=false, bottom=false, right=false, left=false;
        for (int w : walls) {
            if (w == RIGHT) right = true;
            else if (w == TOP) top = true;
            else if (w == LEFT) left = true;
            else if (w == BOTTOM) bottom = true;
        }

        std::vector<Pos> neighbors;
        if (!top)    neighbors.push_back({c.x, c.y-1});
        if (!bottom) neighbors.push_back({c.x, c.y+1});
        if (!right)  neighbors.push_back({c.x+1, c.y});
        if (!left)   neighbors.push_back({c.x-1, c.y});

        for (Pos n : neighbors) {
            if (!in_bounds(n)) continue;
            if (visited[key(n)]) continue;
            visited[key(n)] = true;
            came_from[key(n)] = c;
            if (n.x == dest.x && n.y == dest.y) { found = true; break; }
            q.push(n);
        }
    }

    if (!found) {
        printf("there is no path to the destination {%d, %d}. Sorry!\n", dest.x, dest.y);
        return std::nullopt;
    }

    // reconstruct path (excluding start, like the original)
    std::vector<Pos> path;
    Pos cur = dest;
    while (!(cur.x == start.x && cur.y == start.y)) {
        path.push_back(cur);
        cur = came_from[key(cur)];
    }
    std::reverse(path.begin(), path.end());

    printf("Found a Path (%ld): ", path.size()); print_path(path); printf("\n");
    return path;
}
int main(int argc, char **argv) {
	rclcpp::init(argc, argv);

	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("move_to_position");
	auto cb_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

	map_sub_ = node->create_subscription<Grid>("map", 10, std::bind(&map_callback, _1));
	execute_plan_ = node->create_client<ExecutePlan>("execute_plan");
	move_to_position_ = node->create_service<MoveToPosition>("move_to_position", move_to_position, rmw_qos_profile_services_default, cb_group);
	pose_sub_ = node->create_subscription<Pose>("pose", 10, std::bind(&pose_callback, _1));

	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(node);
	executor.spin();

	rclcpp::shutdown();
	return 0;
}


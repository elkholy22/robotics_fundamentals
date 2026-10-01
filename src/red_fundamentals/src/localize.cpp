// #ifndef RCLCPP
// #define RCLCPP
// #include "rclcpp/rclcpp.hpp"
// #include <rclcpp/utilities.hpp>
// #endif

#include <cmath>
#include <mutex>
#include "maze.hpp"

#include <map>
#include <set>
#include <climits>

#include <algorithm>

#include "red_fundamentals/msg/line_segment_array.hpp"
#include "red_fundamentals/msg/line_segment.hpp"
#include "movement.hpp"
#include "map_and_messages/msg/pose.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "red_fundamentals/srv/execute_plan.hpp"
#include "create_fundamentals/srv/store_song.hpp"
#include "create_fundamentals/srv/play_song.hpp"
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/empty.hpp>

using red_fundamentals::msg::LineSegment;
using red_fundamentals::msg::LineSegmentArray;

using std::placeholders::_1;
using std::placeholders::_2;
using ExecutePlan = red_fundamentals::srv::ExecutePlan;

using namespace std::chrono_literals;

struct CellPos;
std::optional<CellPos> localize();
std::vector<CellPos> grid_in_grid(Grid global_grid, Grid local_grid, CellPos local_pos);
Grid turn_grid(Grid g, DIRECTION turn_by);
CellPos turn_pos(CellPos c, DIRECTION direction, int x_size, int y_size);
void print_grid(Grid g);

Grid new_empty_grid(int x, int y);

struct CellPos
{
	DIRECTION direction;
	int x, y; // This is in cells NOT in cm or m
};

rclcpp::CallbackGroup::SharedPtr map_callback_group, walls_callback_group, localization_callback_group;

rclcpp::Subscription<Grid>::SharedPtr map_sub_;
rclcpp::Subscription<LineSegmentArray>::SharedPtr walls_sub_;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wall_publisher_;
rclcpp::Client<AddMove>::SharedPtr add_move_client_;
rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr center_sub_;
rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr subcell_publisher_;
rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr subcell_global_publisher_;
rclcpp::CallbackGroup::SharedPtr plan_callback_group;
rclcpp::Publisher<map_and_messages::msg::Pose>::SharedPtr pose_publisher_;
rclcpp::Client<create_fundamentals::srv::StoreSong>::SharedPtr store_song_client_;
rclcpp::Client<create_fundamentals::srv::PlaySong>::SharedPtr play_song_client_;
rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr plan_sub_;
rclcpp::Client<AbortMove>::SharedPtr abort_move_client_;

rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr align_done_sub_;
rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr trigger_align_pub_;

std::vector<int> current_plan;
int plan_index = 0;
bool executing_plan = false;

std::atomic<bool> align_received_{false};
std::atomic<bool> waiting_for_realign_{false};

// Forward declare so align_done_callback can call these
void on_localized(CellPos pos);

// These must be declared before align_done_callback which uses them
int cur_row = 0, cur_col = 0, cur_orientation = 0;
bool tracking_active = false;
bool robot_is_lost = false;
bool localization_done = false;

void align_done_callback(const std_msgs::msg::Empty::SharedPtr /*msg*/)
{
	printf("align_done received!\n");
	align_received_ = true;
	waiting_for_realign_ = false;

	/* if (waiting_for_realign_.load()) {
	   printf("Re-align complete, now re-localizing...\n");
	   waiting_for_realign_ = false;

	//auto result = localize();
	if (result.has_value()) {
	robot_is_lost = false;
	localization_done = true;
	on_localized(result.value());
	printf("Re-localization successful after kidnapping!\n");
	} else {
	printf("Re-localization FAILED after kidnapping. Will retry on next align_done.\n");
	}
	}*/
}

bool centered_checked = false;
const double CENTER_BAND = 0.12;
double prev_fwd = 0.0;
bool have_prev = false;
const double CROSSING_THRESHOLD = 0.2;

std::optional<Grid> global_grid_;
std::optional<LineSegmentArray> lines;
std::mutex lines_mutex;

std::vector<rclcpp::Client<AddMove>::SharedFuture> futures;
void send_moves(std::vector<DriveInstruction> moves, rclcpp::Client<AddMove>::SharedPtr client)
{
	// Send all requests
	printf("sending move\n");
	for (const auto &m : moves)
	{
		auto request = std::make_shared<AddMove::Request>();
		request->left = m.left;
		request->right = m.right;

		futures.push_back(client->async_send_request(request).future.share());
	}
}
bool await_moves()
{
	std::vector<MoveReturn> return_codes;
	for (auto &future : futures)
	{
		future.wait();
		auto response = future.get();
		if (response->ret != MOVE_RETURN_SUCCESS)
		{
			futures.clear();
			return false;
		}
	}
	futures.clear();
	return true;
}

void printf_debug_cell(Cell c)
{
	for (int i = 0; i < c.walls.size(); i++)
	{
		printf("%d,", c.walls[i]);
	}
}

bool same_cell(Cell a, Cell b)
{
	std::set<int> as(a.walls.begin(), a.walls.end());
	std::set<int> bs(b.walls.begin(), b.walls.end());
	return as == bs;
}

void publish_pose()
{
	map_and_messages::msg::Pose pose;
	pose.row = cur_row;
	pose.column = cur_col;
	pose.orientation = cur_orientation;
	pose_publisher_->publish(pose);
}

void play_song(int n)
{
	auto req = std::make_shared<create_fundamentals::srv::PlaySong::Request>();
	req->number = n;
	play_song_client_->async_send_request(req);
}

void store_song(int n, std::vector<uint32_t> song)
{
	auto req = std::make_shared<create_fundamentals::srv::StoreSong::Request>();
	req->number = n;
	req->song = song;
	store_song_client_->async_send_request(req);
}

DIRECTION plan_to_direction(int plan_code)
{
	switch (plan_code)
	{
		case ExecutePlan::Request::RIGHT:
			return RIGHT;
		case ExecutePlan::Request::UP:
			return TOP;
		case ExecutePlan::Request::LEFT:
			return LEFT;
		case ExecutePlan::Request::DOWN:
			return BOTTOM;
	}
	return TOP; // unreachable if the plan codes are valid
}
// ggrid to global_grid
// lgrid to local_grid
std::vector<CellPos> grid_in_grid(Grid ggrid, Grid lgrid, CellPos local_pos)
{
	printf("grid_in_grid 1\n");
	printf("GLOBAL Grid: \n");
	print_grid(ggrid);
	printf("LOCAL Grid (local_pos: %d; %d, %d): \n", local_pos.direction, local_pos.x, local_pos.y);
	print_grid(lgrid);
	std::vector<CellPos> possible_pos;
	std::vector<DIRECTION> possible_pos_local_offset;
	// get first anker
	for (int i = 0; i < 4; i++)
	{
		auto turned_lgrid = turn_grid(lgrid, (DIRECTION)i);
		auto turned_local_pos = turn_pos(local_pos, (DIRECTION)i, lgrid.rows[0].cells.size(), lgrid.rows.size());
		printf("1. Step: local Grid (pos: %d; %d, %d): \n", turned_local_pos.direction, turned_local_pos.x, turned_local_pos.y);
		print_grid(turned_lgrid);
		for (int y = 0; y < ggrid.rows.size(); y++)
		{
			auto row = ggrid.rows[y];
			for (int x = 0; x < row.cells.size(); x++)
			{
				auto cell = row.cells[x];
				printf("cell (%d):", i);
				printf_debug_cell(cell);
				printf(", ");
				printf_debug_cell(turned_lgrid.rows[turned_local_pos.y].cells[turned_local_pos.x]);
				printf("... ");

				if (same_cell(cell, turned_lgrid.rows[turned_local_pos.y].cells[turned_local_pos.x]))
				{
					printf("yes\n");
					possible_pos.push_back({.direction = turned_local_pos.direction, .x = x, .y = y}); // TODO is this direction correct? Or flipped?
					possible_pos_local_offset.push_back((DIRECTION)i);
				}
				else
				{
					printf("no\n");
				}
			}
		}
	}
	printf("grid_in_grid 2\n");
	printf("possible_pos: %ld:\n\n", possible_pos.size());
	for (int i = 0; i < possible_pos.size(); i++)
	{
		printf("DIR: %d; x: %d; y: %d\n", possible_pos[i].direction, possible_pos[i].x, possible_pos[i].y);
	}
	printf("\n");
	printf("2. Local Grid: \n");
	print_grid(lgrid);
	// spread from local_grid begin to see which possibilities stick
	for (int i = possible_pos.size() - 1; i >= 0; i--)
	{ // We do backwards because we cant safely erase otherwise
		auto turned_lgrid = turn_grid(lgrid, possible_pos_local_offset[i]);
		auto turned_local_pos = turn_pos(local_pos, possible_pos_local_offset[i], lgrid.rows[0].cells.size(), lgrid.rows.size());
		printf("Turned lgrid (%d) (turned by: %d, localpos dir: %d): \n", i, possible_pos_local_offset[i], possible_pos[i].direction);
		print_grid(turned_lgrid);
		for (int y = 0; y < ggrid.rows.size(); y++)
		{
			auto row = ggrid.rows[y];
			for (int x = 0; x < row.cells.size(); x++)
			{
				auto cell = row.cells[x];
				int local_x = turned_local_pos.x - (possible_pos[i].x - x);
				int local_y = turned_local_pos.y - (possible_pos[i].y - y);
				printf("(%d) %d;%d :: local: %d:%d: turned_local_pos:%d:%d, possible_pos: %d:%d\n", i, x, y, local_x, local_y, turned_local_pos.x, turned_local_pos.y, possible_pos[i].x, possible_pos[i].y);

				if (local_y < 0 || local_y >= (int)turned_lgrid.rows.size() ||
						local_x < 0 || local_x >= (int)turned_lgrid.rows[local_y].cells.size())
				{
					possible_pos.erase(possible_pos.begin() + i);
					goto next_iteration;
				}

				// TODO we cant handle no walls cells in lgrid, as its the same as "we dont know".
				// This is possibly bad, so we should probably find a solution at some point
				if (turned_lgrid.rows[local_y].cells[local_x].walls.empty())
					continue;

				printf("not discarding cells: ");
				printf_debug_cell(cell);
				printf(", ");
				printf_debug_cell(turned_lgrid.rows[local_y].cells[local_x]);
				printf("\n");

				if (!same_cell(cell, turned_lgrid.rows[local_y].cells[local_x]))
				{
					possible_pos.erase(possible_pos.begin() + i);
					goto next_iteration;
				}
			}
		}
next_iteration:;
	}
	printf("grid_in_grid 3\n");
	printf("possible positions (grid_in_grid end): %ld\n", possible_pos.size());
	return possible_pos;
}

// TODO do we really need to copy?
Grid turn_grid(Grid g, DIRECTION direction)
{
	const int x_size = g.rows[0].cells.size();
	const int y_size = g.rows.size();
	Grid new_g = new_empty_grid(x_size, y_size);

	for (int y = 0; y < y_size; y++)
	{
		for (int x = 0; x < x_size; x++)
		{
			for (auto wall : g.rows[y].cells[x].walls)
			{
				// printf("WALL IN TURN_GRID: x:%d, y:%d, val: %d\n", x, y, wall);

				int new_x, new_y;
				if (direction == RIGHT)
				{
					new_x = x_size - 1 - y;
					new_y = x;
				}
				else if (direction == BOTTOM)
				{
					new_x = x_size - 1 - x;
					new_y = y_size - 1 - y;
				}
				else if (direction == LEFT)
				{
					new_x = y;
					new_y = x_size - 1 - x;
				}
				else
				{ // TOP or unknown
					new_x = x;
					new_y = y;
				}

				// Safety: skip if coordinates are out of bounds
				if (new_y < 0 || new_y >= y_size || new_x < 0 || new_x >= x_size)
					continue;

				new_g.rows[new_y].cells[new_x].walls.push_back((wall + ((direction - 1) & 3)) & 3);
			}
		}
	}

	return new_g;
}

void print_grid(Grid g)
{
	printf("Printing grid (%ldx%ld)", g.rows[0].cells.size(), g.rows.size());
	for (int y = 0; y < g.rows.size(); y++)
	{
		for (int layer = 0; layer < 3; layer++)
		{
			for (int x = 0; x < g.rows[0].cells.size(); x++)
			{
				bool top = false, bottom = false, left = false, right = false;
				for (auto wall : g.rows[y].cells[x].walls)
				{
					if (wall == TOP)
						top = true;
					if (wall == BOTTOM)
						bottom = true;
					if (wall == LEFT)
						left = true;
					if (wall == RIGHT)
						right = true;
				}
				if (layer == 0)
					printf(".%c.", top ? '_' : ' ');
				if (layer == 1)
					printf("%c %c", left ? '|' : '.', right ? '|' : '.');
				if (layer == 2)
					printf(".%c.", bottom ? '_' : ' ');
			}
			printf("\n");
		}
	}
}

// CellPos turn_pos(CellPos c, int direction_change, int x_size, int y_size)
// {
//      c.direction = (DIRECTION) ((c.direction + (int)direction_change) & 3);
//      if (direction_change == 3) {
//              int tmp = c.x;
//              c.x = x_size-1 - c.y;
//              c.y = tmp;
//      }
//      if (direction_change == 2) {
//              c.y = y_size-1 - c.y;
//      }
//      if (direction_change == 1) {
//              int tmp = c.x;
//              c.x = c.y;
//              c.y = x_size-1 - tmp;
//      }
//      return c;
// }
CellPos turn_pos(CellPos c, DIRECTION direction, int x_size, int y_size)
{
	c.direction = (DIRECTION)((c.direction + (int)direction - 1) & 3);
	if (direction == RIGHT)
	{
		int tmp = c.x;
		c.x = x_size - 1 - c.y;
		c.y = tmp;
	}
	else if (direction == BOTTOM)
	{
		c.x = x_size - 1 - c.x;
		c.y = y_size - 1 - c.y;
	}
	else if (direction == LEFT)
	{
		int tmp = c.x;
		c.x = c.y;
		c.y = x_size - 1 - tmp;
	}
	// TOP: no coordinate change needed

	// Clamp to valid range to prevent out of bounds access later
	if (c.x < 0)
		c.x = 0;
	if (c.x >= x_size)
		c.x = x_size - 1;
	if (c.y < 0)
		c.y = 0;
	if (c.y >= y_size)
		c.y = y_size - 1;

	return c;
}

// die direction ist, welche richtung am ende TOP sein soll. Wenn direction=LEFT, dann wird also alles um 90* nach rechts gedreht
Cell cur_cell(DIRECTION direction = TOP)
{
	// We think that we stand in the middle of a cell
	Cell c;
	// TODO do this the proper and robust way
	while (!lines.has_value())
	{
		printf("We dont have lines yet, waiting...");
		std::this_thread::sleep_for(2s);
		printf("trying again.\n");
	}
	static const float max_dist = sqrt(0.6 * 0.6 + 0.6 * 0.6);
	static const float axis_align_threshold = 0.707f; // lines within 45° of an axis considered aligned with that axis

	LineSegmentArray lines_snapshot;
	{
		std::lock_guard<std::mutex> lk(lines_mutex);
		lines_snapshot = lines.value();
	}

	bool top = false, bottom = false, left = false, right = false;
	for (auto line : lines_snapshot.segments)
	{
		printf("LINE: x: %f-%f; y: %f-%f\n", line.start_x, line.end_x, line.start_y, line.end_y);
		printf("LINE: start: %f\n", sqrt(line.start_x * line.start_x + line.start_y * line.start_y));
		printf("LINE: end: %f\n", sqrt(line.end_x * line.end_x + line.end_y * line.end_y));

		// direction vector of the line segment
		float dx = line.end_x - line.start_x;
		float dy = line.end_y - line.start_y;
		float len_sq = dx*dx + dy*dy;
		if (len_sq < 0.0001f) continue;          // degenerate segment
		float len = sqrt(len_sq);
		float ux = dx / len;
		float uy = dy / len;	
		// perpendicular distance from origin to the infinite line (2 D cross-product magnitude)
		float perp_dist = fabs(line.start_x * uy - line.start_y * ux);

		// determine whether the line is axis-aligned.
		// horizontal: direction is close to (+-1, 0)
		// bertical:   direction is close to (0, +-1)
		bool is_horizontal = fabs(ux) > axis_align_threshold;
		bool is_vertical = fabs(uy) > axis_align_threshold;
		printf("LINE: (hor: %d, ver: %d)\n", is_horizontal, is_vertical);

		if (!is_horizontal && !is_vertical) continue;   // not a cell 

		// Calculate projection point onto wall (not line, i.e. clamp t) for distance and orientation checking
		float t = - (ux * line.start_x + uy * line.start_y);
		printf("t: %f, len: %f\n", t, len);
		if (t < 0.0f || t > len+0.2) continue;

		float mx = line.start_x + t * ux;
		float my = line.start_y + t * uy;

		if (sqrt(mx * mx + my * my) > max_dist) {
			printf("gsssss\n");
			continue;
		}
		if (is_horizontal && my > 0.0f)      top = true;
		else if (is_horizontal && my < 0.0f) bottom = true;
		else if (is_vertical   && mx > 0.0f) right = true;
		else if (is_vertical   && mx < 0.0f) left = true;

		printf("TOP: %d, BOTTOM: %d, LEFT: %d, RIGHT: %d\n", top, bottom, left, right);
	}

	int turn_by = (direction - 1) & 3;
	if (top)
		c.walls.push_back((TOP + turn_by) & 3);
	if (bottom)
		c.walls.push_back((BOTTOM + turn_by) & 3);
	if (left)
		c.walls.push_back((LEFT + turn_by) & 3);
	if (right)
		c.walls.push_back((RIGHT + turn_by) & 3);
	printf("cell: ");
	printf_debug_cell(c);
	printf("\n");

	return c;
}

// Scan all 4 walls by physically turning the robot 90° three times.
// This gives a complete picture of the current cell, reducing ambiguity
// during the initial localization step.
Cell cur_cell_full_scan()
{
	printf("=== cur_cell_full_scan: starting ===\n");
	Cell result;

	DIRECTION dir = (DIRECTION)1;

	// Invalidate cached lines so cur_cell() waits for fresh sensor data
	lines.reset();
	std::this_thread::sleep_for(300ms);

	Cell partial = cur_cell(dir);
	printf("Full scan dir %d: ", 1);
	printf_debug_cell(partial);
	printf("\n");

	printf("Turning 90 degrees\n");
	send_moves(turn(90), add_move_client_);
	await_moves();
	std::this_thread::sleep_for(3000ms);

	result = cur_cell(dir);

	for(int i=0; i < partial.walls.size(); i++) {
		if (partial.walls[i] == 2) result.walls.push_back(3);
	}

	printf("=== cur_cell_full_scan: stop ===\n");
	printf("result: ");
	printf_debug_cell(result);
	printf("\n");
	return result;
}

/////
static void dir_offset(DIRECTION d, int &dx, int &dy)
{
	dx = 0;
	dy = 0;
	switch (d)
	{
		case RIGHT:
			dx = 1;
			break;
		case LEFT:
			dx = -1;
			break;
		case TOP:
			dy = -1;
			break;
		case BOTTOM:
			dy = 1;
			break;
	}
}
static bool has_wall(const Cell &c, DIRECTION d)
{
	return std::find(c.walls.begin(), c.walls.end(), (int)d) != c.walls.end();
}
static int rotate_to_global(int d, int r) { return (d + (r - 1)) & 3; }
static int rotate_to_robot(int w, int r) { return (w - (r - 1)) & 3; }
static std::vector<int> predicted_obs(const Cell &cell, int r)
{
	std::vector<int> obs;
	for (int w : cell.walls)
		obs.push_back(rotate_to_robot(w, r));
	std::sort(obs.begin(), obs.end());
	return obs;
}
// choose best new cell to go to

// First: Just random
// Second: Greedy algorithm: Which move eliminates the most possibilities
// Third: Full tree algorithm, choose smallest way to an end

// Sanity check: new Cell should be max 1 away from currunt CellPos
// if (abs(cpos.x + cpos.y - (tpos.x + tpos.y)) > 1) {
//      printf("Bad output from choose_new_cell: (%d, %d) -> (%d, %d) dist is > 1\n", cpos.x, cpos.y, tpos.x, tpos.y);
//      return std::optional<CellPos>();
// }

// TODO maybe remove cpos?

CellPos choose_new_cell(Grid global_grid, std::vector<CellPos> potential_pos, const Grid &local_grid, CellPos cpos, std::set<std::pair<int, int>> &visited)
{
	bool top = true, bottom = true, left = true, right = true;
	for (int i = 0; i < local_grid.rows[cpos.y].cells[cpos.x].walls.size(); i++)
	{
		const auto wall = (local_grid.rows[cpos.y].cells[cpos.x].walls[i] + ((4 - (cpos.direction - 1) & 3) & 3)) & 3;
		printf("converting wall (cpos dir: %d): %d => %d\n", cpos.direction, local_grid.rows[cpos.y].cells[cpos.x].walls[i], (local_grid.rows[cpos.y].cells[cpos.x].walls[i] + ((cpos.direction - 1) & 3)) & 3);
		if (wall == TOP)
			top = false;
		if (wall == BOTTOM)
			bottom = false;
		if (wall == LEFT)
			left = false;
		if (wall == RIGHT)
			right = false;
	}

	std::vector<int> rotations;
	if (top)
		rotations.push_back(0);
	else if (left)
		rotations.push_back(1);
	else if (bottom)
		rotations.push_back(2);
	else if (right)
		rotations.push_back(3);
	if (rotations.empty())
	{
		printf("STUUUUUCK AAAAAAAAAAAAAAH\n");
		return cpos;
	}

	std::srand(std::time(nullptr));
	const int r = std::rand() % rotations.size();
	const int rotation = rotations[r];

	DIRECTION next_direction = (DIRECTION)((cpos.direction + rotation) & 3);
	printf("choose_new_cell: top: %b, bottom: %b, left: %b, right: %b => (%d-%d&3=%d)\n", top, bottom, left, right, cpos.direction, rotation, next_direction);
	if (next_direction == TOP)
		return (CellPos){.direction = next_direction, .x = cpos.x, .y = cpos.y - 1};
	if (next_direction == BOTTOM)
		return (CellPos){.direction = next_direction, .x = cpos.x, .y = cpos.y + 1};
	if (next_direction == LEFT)
		return (CellPos){.direction = next_direction, .x = cpos.x - 1, .y = cpos.y};
	if (next_direction == RIGHT)
		return (CellPos){.direction = next_direction, .x = cpos.x + 1, .y = cpos.y};
	// if (rotation == 0) return (CellPos) {.direction = next_direction, .x = cpos.x, .y = cpos.y-1};
	// if (rotation == 2) return (CellPos) {.direction = next_direction, .x = cpos.x, .y = cpos.y+1};
	// if (rotation == 3) return (CellPos) {.direction = next_direction, .x = cpos.x-1, .y = cpos.y};
	// if (rotation == 1) return (CellPos) {.direction = next_direction, .x = cpos.x+1, .y = cpos.y};
	else
	{
		printf("AAAAAAAAAAAAAAAH\n");
		return cpos;
	}

	// const Cell &here = local_grid.rows[cpos.y].cells[cpos.x];

	// visited.insert({cpos.x, cpos.y});

	// int rows = global_grid.rows.size();
	// int cols = global_grid.rows[0].cells.size();
	// int best_worst = INT_MAX;
	// bool found = false;
	// CellPos best{};

	// for (int d = 0; d < 4; d++) {
	//      if (has_wall(here, (DIRECTION)d)) continue;

	//      std::map<std::vector<int>, int> groups;

	//      for (const CellPos &h : potential_pos)
	//      {
	//              int dg = rotate_to_global(d, h.direction);
	//              int dx, dy;
	//              dir_offset((DIRECTION)dg, dx, dy);
	//              int nx = h.x + dx, ny = h.y + dy;
	//              if (nx < 0 || nx >= cols || ny < 0 || ny >= rows)
	//                      continue;
	//              groups[predicted_obs(global_grid.rows[ny].cells[nx], h.direction)]++;
	//      }

	//      int worst = 0;
	//      for (auto &g : groups)
	//              worst = std::max(worst, g.second);
	//      if (!found || worst < best_worst)
	//      {
	//              best_worst = worst;
	//              int ldx, ldy;
	//              dir_offset((DIRECTION)d, ldx, ldy);
	//              best = CellPos{.direction = (DIRECTION)d, .x = cpos.x + ldx, .y = cpos.y + ldy};
	//              found = true;
	//      }
	// }
	// return found ? best : cpos;
}

Grid new_empty_grid(int x, int y)
{
	Grid g;
	for (int i = 0; i < y; i++)
	{
		Row r;
		for (int j = 0; j < x; j++)
		{
			Cell c;
			r.cells.push_back(c);
		}
		g.rows.push_back(r);
	}
	return g;
}

bool move_to_pos(CellPos cur, CellPos target)
{
	// TODO this can largely be copied from the grid_fast movement logic
	int x, y;
	int change_cclock;

	// if (cur.direction == RIGHT) {
	// 	x = cur.y - target.y;
	// 	y = target.x - cur.x;
	// }
	// if (cur.direction == LEFT) {
	// 	x = target.y - cur.y;
	// 	y = target.x - cur.x;
	// }
	// if (cur.direction == BOTTOM) {
	// 	x = cur.x - target.x;
	// 	y = cur.y - target.y;
	// }
	// if (cur.direction == TOP) {
	x = target.x - cur.x;
	y = target.y - cur.y;
	// }
	if (abs(x) + abs(y) > 1)
	{
		printf("Moving more than one cell in move_to_pos. Aborting...\n");
		return false;
	}
	printf("move_to_pos: x: %d->%d, y: %d->%d\n", target.x - cur.x, x, target.y - cur.y, y);
	printf("move_to_pos: cur.direction: %d\n", cur.direction);

	// if (x > 0) dir = RIGHT;
	// else if (x < 0) dir = LEFT;
	// else if (y > 0) dir = BOTTOM;
	// else if (y < 0) dir = TOP;
	change_cclock = (DIRECTION)((target.direction - cur.direction) & 3);
	if (change_cclock == 3)
		change_cclock = -1;

	printf("move_to_pos: change_cclock: %d\n", change_cclock);

	send_moves(turn(change_cclock * -90), add_move_client_);
	if (!await_moves()) return false;
	std::this_thread::sleep_for(100ms);
	send_moves(drive(80), add_move_client_);

	// switch (change_cclock) {
	// 	case TOP: send_moves(drive_to_pose(0, 80, 0), add_move_client_); break;
	// 	case RIGHT: send_moves({turn(90)[0], drive(80)[0], turn((90 * direction_change - 90) % 360)[0]}, add_move_client_); break;
	// 	case LEFT: send_moves({turn(-90)[0], drive(80)[0], turn((90 * direction_change + 90) % 360)[0]}, add_move_client_); break;
	// 	case BOTTOM: send_moves({turn(180)[0], drive(80)[0], turn((90 * direction_change - 180) % 360)[0]}, add_move_client_); break; // TODO do we have to drive till the middle?
	// }

	return await_moves();
}

bool observation_matches()
{
    if (!global_grid_.has_value()) return true;
    Cell observed = cur_cell((DIRECTION)cur_orientation);
    if (observed.walls.empty()) return true;
    Cell expected = global_grid_.value().rows[cur_row].cells[cur_col];
    return same_cell(expected, observed);
}

void on_localized(CellPos pos)
{
	cur_row = pos.y;
	cur_col = pos.x;
	cur_orientation = pos.direction;
	tracking_active = true;
	publish_pose();
	play_song(0);
}

void on_kidnapped()
{
	play_song(1); // warning sound
	tracking_active = false;
	robot_is_lost = true;
	localization_done = false;

	align_received_ = false;

	// Trigger re-alignment.  Once align finishes it will publish
	// align_done, our callback will see waiting_for_realign_==true
	// and run localize() automatically.
	waiting_for_realign_ = true;
	printf("KIDNAPPED! Publishing trigger_align to restart alignment...\n");
	trigger_align_pub_->publish(std_msgs::msg::Empty());
}

bool check_for_lost()
{
	int mismatches = 0;
	const int READS = 3;
	for (int i = 0; i < READS; i++)
	{
		std::this_thread::sleep_for(100ms);
		if (!observation_matches())
			mismatches++;
	}
	return mismatches == READS;
}

// localization can fail if there is a deadlock
// or some symmetry mazes where not every field is accessable
std::optional<CellPos> localize()
{
	printf("start localize\n");
	const auto global_grid = global_grid_.value();

	const unsigned long x = global_grid.rows[0].cells.size();
	const unsigned long y = global_grid.rows.size();
	printf("global_grid x: %ld, y: %ld\n", x, y);
	Grid local_grid = new_empty_grid(x * 2 - 1, y * 2 - 1);

	printf("localize 1\n");
	/////////
	std::set<std::pair<int, int>> visited;
	Cell c = cur_cell_full_scan();
	printf("SCANNED CELL c: ");
	printf_debug_cell(c);
	printf("\n");
	printf("localize 2\n");
	CellPos cpos = {.direction = TOP, .x = (int)x - 1, .y = (int)y - 1};
	printf("cposx: %d, cposy: %d\n", cpos.x, cpos.y);
	local_grid.rows[cpos.y].cells[cpos.x] = c;
	printf("localize 3\n");
	auto global_possibilities = grid_in_grid(global_grid, local_grid, cpos);
	printf("global_possibilities: %ld\n", global_possibilities.size());
	printf("localize 4\n");

	if (global_possibilities.size() == 0)
		return std::optional<CellPos>();
	if (global_possibilities.size() == 1)
		return global_possibilities[0];
	printf("localize 5\n");

	int turn_by = 0;
	while (global_possibilities.size() > 1)
	{
		printf("localize while 1\n");
		/////////////
		CellPos tpos = choose_new_cell(global_grid, global_possibilities, local_grid, cpos, visited);
		printf("going to new cell (local): %d; %d, %d ==> %d; %d, %d\n", cpos.direction, cpos.x, cpos.y, tpos.direction, tpos.x, tpos.y);
		printf("localize while 2\n");
		if (!move_to_pos(cpos, tpos))
		{
			printf("Error while moving. Cant localize any further. Aborting...\n");
			return std::optional<CellPos>();
		}
		printf("sleeping to be sure we have fresh walls\n");
		std::this_thread::sleep_for(1000ms);
		printf("localize while 3\n");

		// TODO this should be done by choose_new_cell
		// if (tpos.y < cpos.y) turn_by += 2;
		// else if (tpos.x < cpos.x) turn_by -= 1;
		// else if (tpos.x > cpos.x) turn_by += 1;
		// turn_by = turn_by & 3; // TODO bitmask by 4. We could also just remove it
		cpos = tpos;

		c = cur_cell(cpos.direction);
		printf("localize while 4\n");
		local_grid.rows[cpos.y].cells[cpos.x] = c;
		global_possibilities = grid_in_grid(global_grid, local_grid, cpos);
		printf("localize while 5\n");
	}
	printf("localize 6\n");

	if (global_possibilities.size() == 0)
		return std::optional<CellPos>();
	else
		return global_possibilities[0];
}

void localize_callback()
{

	if (localization_done)
		return;
	if (!align_received_.load())
	{
		printf("Waiting for alignment to finish before localizing...\n");
		return;
	}
	if (global_grid_.has_value() == false)
	{
		printf("Did not get a Grid yet, waiting 1s\n");
		return;
	}

	auto ret = localize();
	if (ret.has_value() == false)
	{
		printf("Failed to localize\n");
		return;
	}
	localization_done = true;
	auto val = ret.value();
	printf("Localized!: x: %d, y: %d, direction: %d\n", val.x, val.y, val.direction);
	on_localized(val);
}

void map_callback(const Grid &grid)
{
	// printf("Received a new Map!\n");
	global_grid_ = grid;

	// THIS IS DEBUG STUFF ONLY. This publishes the map to rviz2
	wall_publisher_->publish(grid_to_lines(global_grid_.value()));
}

void walls_callback(const LineSegmentArray &new_lines)
{
	std::lock_guard<std::mutex> lk(lines_mutex);
	lines = new_lines;
}

void plan_callback(const std_msgs::msg::Int32MultiArray &msg)
{
	if (!localization_done)
		return; // localise first, then accept plans

	current_plan.assign(msg.data.begin(), msg.data.end());
	plan_index = 0;
	have_prev = false;
	executing_plan = true;
	centered_checked = false;
	printf("Received plan (len %zu): ", current_plan.size());
	for (int d : current_plan)
		printf("%d ", d);
	printf("\n");
}

void abort_plan()
{
	abort_move_req(abort_move_client_);
	executing_plan = false;
}

void cell_center_callback(const geometry_msgs::msg::PointStamped &msg)
{
    double fwd = msg.point.y;
    if (std::isnan(fwd)) return;

    // preserve existing subcell_offset republish
    geometry_msgs::msg::PointStamped sub;
    sub.point.x = -msg.point.x;
    sub.point.y = -msg.point.y;
    subcell_publisher_->publish(sub);

    // global frame offset: 
    double lx = -msg.point.x;
    double ly = -msg.point.y;
    double gx = 0.0, gy = 0.0;
    switch (cur_orientation) {
        case 0: gx =  ly; gy =  lx; break;  // RIGHT 
        case 1: gx =  lx; gy = -ly; break;  // UP 
        case 2: gx = -ly; gy = -lx; break;  // LEFT 
        case 3: gx = -lx; gy =  ly; break;  // DOWN 
    }
    geometry_msgs::msg::PointStamped sub_global;
    sub_global.point.x = gx;  
    sub_global.point.y = gy;   
    subcell_global_publisher_->publish(sub_global);
	if (localization_done) publish_pose();
    if (!executing_plan) {
        prev_fwd  = fwd;
        have_prev = true;
        return;
    }

    // boundary crossing: advance the discrete pose
    if (have_prev && prev_fwd < -CROSSING_THRESHOLD && fwd > CROSSING_THRESHOLD
        && plan_index < (int)current_plan.size())
    {
        DIRECTION dir = plan_to_direction(current_plan[plan_index]);
        int dx, dy;
        dir_offset(dir, dx, dy);
        cur_col += dx;
        cur_row += dy;
        cur_orientation = (int)dir;
        plan_index++;
        if (plan_index >= (int)current_plan.size())
            executing_plan = false;
        publish_pose();
        printf("CROSSING -> row %d col %d orient %d (step %d/%d)\n",
               cur_row, cur_col, cur_orientation, plan_index, (int)current_plan.size());

        centered_checked = false;           // new cell allow one lost check
    }

    // lost detection: only when settled near a cell center, once per cell
    if (executing_plan && !centered_checked
        && std::abs(msg.point.x) < CENTER_BAND
        && std::abs(fwd)         < CENTER_BAND)
    {
        centered_checked = true;
        // if (check_for_lost()) {
        //     printf("LOST mid-plan -> abort + relocalise\n");
        //     abort_plan();
        //     on_kidnapped();
        // }
    }

    prev_fwd  = fwd;
    have_prev = true;
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	printf("initialising...");

	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("localize");

	add_move_client_ = node->create_client<AddMove>("add_move");
	abort_move_client_ = node->create_client<AbortMove>("abort_move");

	walls_callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	map_callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	localization_callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

	rclcpp::SubscriptionOptions map_options, walls_options;
	map_options.callback_group = map_callback_group;
	walls_options.callback_group = walls_callback_group;
	map_sub_ = node->create_subscription<Grid>("map", 10, std::bind(&map_callback, _1), map_options);
	walls_sub_ = node->create_subscription<LineSegmentArray>("extracted_walls", 10, std::bind(&walls_callback, _1), walls_options);

	{
		rclcpp::SubscriptionOptions align_done_opts;
		align_done_opts.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
		align_done_sub_ = node->create_subscription<std_msgs::msg::Empty>(
				"align_done", 10, std::bind(&align_done_callback, _1), align_done_opts);
	}
	{
		rclcpp::PublisherOptions trigger_align_opts;
		trigger_align_opts.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
		trigger_align_pub_ = node->create_publisher<std_msgs::msg::Empty>("trigger_align", 10, trigger_align_opts);
	}

	auto localization_callback_ = node->create_wall_timer(1s, std::bind(&localize_callback), localization_callback_group);

	pose_publisher_ = node->create_publisher<map_and_messages::msg::Pose>("pose", 10);

	store_song_client_ = node->create_client<create_fundamentals::srv::StoreSong>("store_song");
	play_song_client_ = node->create_client<create_fundamentals::srv::PlaySong>("play_song");
	store_song_client_->wait_for_service();
	store_song(0, {72, 12, 76, 12, 79, 12, 84, 24}); // success
	store_song(1, {60, 12, 57, 12, 53, 24});         // warning

	rclcpp::ExecutorOptions options;
	rclcpp::executors::MultiThreadedExecutor executor(options, 50); // spawn 50 threads

	wall_publisher_ = node->create_publisher<MarkerArray>("global_grid", 10);

	plan_callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	rclcpp::SubscriptionOptions plan_options;
	plan_options.callback_group = plan_callback_group;

	center_sub_ = node->create_subscription<geometry_msgs::msg::PointStamped>(
			"cell_center", 10, std::bind(&cell_center_callback, _1), plan_options);
	plan_sub_ = node->create_subscription<std_msgs::msg::Int32MultiArray>(
			"current_plan", 10, std::bind(&plan_callback, _1), plan_options);

	subcell_publisher_ = node->create_publisher<geometry_msgs::msg::PointStamped>("subcell_offset", 10);
	subcell_global_publisher_ = node->create_publisher<geometry_msgs::msg::PointStamped>("subcell_offset_global", 10);
	printf(" lets go!\n");
	executor.add_node(node);
	executor.spin();
	rclcpp::shutdown();
	return 0;
}

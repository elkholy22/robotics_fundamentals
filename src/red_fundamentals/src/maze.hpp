// #ifndef RCLCPP
// #define RCLCPP
// #include "rclcpp/rclcpp.hpp"
// #include <rclcpp/utilities.hpp>
// #endif

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "map_and_messages/msg/grid.hpp"
#include "map_and_messages/msg/cell.hpp"

using MarkerArray = visualization_msgs::msg::MarkerArray;
using Marker = visualization_msgs::msg::Marker;
using Grid = map_and_messages::msg::Grid;
using Row = map_and_messages::msg::Row;
using Cell = map_and_messages::msg::Cell;
using Point = geometry_msgs::msg::Point;

enum DIRECTION {
	RIGHT,
	TOP,
	LEFT,
	BOTTOM
};

#define MAZE_IMPLEMENTATION

#ifdef MAZE_IMPLEMENTATION
#ifndef MAZE_IMPLEMENTED
#define MAZE_IMPLEMENTED

MarkerArray grid_to_lines (Grid g) {
	const unsigned long x = g.rows[0].cells.size();
	const unsigned long y = g.rows.size();

	MarkerArray lines;

	Marker marker;
	marker.header.frame_id = std::string("base_link");
	marker.header.stamp = rclcpp::Clock().now();
	marker.ns = "grid";
	marker.id = 0;
	marker.type = visualization_msgs::msg::Marker::LINE_LIST;
	marker.action = visualization_msgs::msg::Marker::ADD;
	marker.scale.x = 0.02;
	marker.color.a = 1.0;
	marker.color.r = 0.0;
	marker.color.g = 0.0;
	marker.color.b = 1.0;

	for(int row=0; row < y; row++) {
		for(int col=0; col < g.rows[row].cells.size(); col++) {
			for(int wall_i=0; wall_i < g.rows[row].cells[col].walls.size(); wall_i++) {
				int wall = g.rows[row].cells[col].walls[wall_i];

				Point p_start, p_end;
				switch (wall) {
				case TOP: {
					p_start.x = .80 * col;
					p_end.x = p_start.x + .80;
					p_start.y = -.80 * row;
					p_end.y = p_start.y;
					p_start.z = 0.0;
					p_end.z = 0.0;
					break;
				}
				case RIGHT: {
					p_start.x = .80 * (col+1);
					p_end.x = p_start.x;
					p_start.y = -.80 * row;
					p_end.y = p_start.y - .80;
					p_start.z = 0.0;
					p_end.z = 0.0;
					break;
				}
				case BOTTOM: {
					p_start.x = .80 * col;
					p_end.x = p_start.x + .80;
					p_start.y = -.80 * (row+1);
					p_end.y = p_start.y;
					p_start.z = 0.0;
					p_end.z = 0.0;
					break;
				}
				case LEFT: {
					p_start.x = .80 * col;
					p_end.x = p_start.x;
					p_start.y = -.80 * row;
					p_end.y = p_start.y - .80;
					p_start.z = 0.0;
					p_end.z = 0.0;
					break;
				}
				};
				marker.points.push_back(p_start);
				marker.points.push_back(p_end);
			}
		}
	}
	lines.markers.push_back(marker);
	return lines;
}

#endif
#endif

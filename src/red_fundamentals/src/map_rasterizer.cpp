#include <vector>
#include <memory>
#include <cmath>
#include <cstdint>
 
#include "rclcpp/rclcpp.hpp"
#include "map_and_messages/msg/grid.hpp"
#include "map_and_messages/msg/cell.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
 
using std::placeholders::_1;
 

static constexpr double CELL_SIZE_M = 0.80;   
static constexpr int WALL_RIGHT  = 0;
static constexpr int WALL_TOP    = 1;
static constexpr int WALL_LEFT   = 2;
static constexpr int WALL_BOTTOM = 3;
 
class MapRasterizer : public rclcpp::Node {
public:
    MapRasterizer() : Node("map_rasterizer") {
        resolution_   = this->declare_parameter<double>("resolution", 0.02);   
        wall_thick_m_ = this->declare_parameter<double>("wall_thickness", 0.04); 
 
        published_ = false;
 

        pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "map_occupancy", rclcpp::QoS(1).transient_local());
 
        sub_ = this->create_subscription<map_and_messages::msg::Grid>(
            "map", 10,
            std::bind(&MapRasterizer::grid_callback, this, _1));
 
        RCLCPP_INFO(this->get_logger(),
                    "Map rasterizer up. res=%.3f m, wall=%.3f m. Waiting for Grid on 'map'...",
                    resolution_, wall_thick_m_);
    }
 
private:
    void grid_callback(const map_and_messages::msg::Grid::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received map callback");
        if (published_) return;      
        if (msg->rows.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty Grid, ignoring.");
            return;
        }
 
        const int nrows = (int)msg->rows.size();
        const int ncols = (int)msg->rows[0].cells.size();
        if (ncols == 0) {
            RCLCPP_WARN(this->get_logger(), "Grid row has no cells, ignoring.");
            return;
        }
 

        const double maze_w = ncols * CELL_SIZE_M;   
        const double maze_h = nrows * CELL_SIZE_M;  
        const double EPS = 1e-9;
        const int gw = (int)std::ceil(maze_w / resolution_ - EPS);
        const int gh = (int)std::ceil(maze_h / resolution_ - EPS);
 
        nav_msgs::msg::OccupancyGrid og;
        og.header.stamp = this->now();
        og.header.frame_id = "map";
        og.info.resolution = resolution_;
        og.info.width  = gw;
        og.info.height = gh;

        og.info.origin.position.x = 0.0;
        og.info.origin.position.y = -CELL_SIZE_M * nrows;
        og.info.origin.position.z = 0.0;
        og.info.origin.orientation.w = 1.0;
 

        og.data.assign((size_t)gw * gh, 0);
 

        const int half = std::max(0, (int)std::round((wall_thick_m_ / resolution_) / 2.0));
 

        for (int row = 0; row < nrows; row++) {
            const int rcols = (int)msg->rows[row].cells.size();
            for (int col = 0; col < rcols; col++) {
                for (int w : msg->rows[row].cells[col].walls) {
                    draw_wall(og, gw, gh, row, col, w, half);
                }
            }
        }
 
        pub_->publish(og);
        published_ = true;
        RCLCPP_INFO(this->get_logger(),
                    "Published OccupancyGrid: maze %dx%d cells -> grid %dx%d @ %.3f m "
                    "(extent %.2f x %.2f m, origin (%.2f, %.2f)).",
                    nrows, ncols, gw, gh, resolution_,
                    maze_w, maze_h, og.info.origin.position.x, og.info.origin.position.y);
    }
 

    void world_to_grid(const nav_msgs::msg::OccupancyGrid &og,
                       double wx, double wy, int &gx, int &gy) {
        gx = (int)std::floor((wx - og.info.origin.position.x) / og.info.resolution);
        gy = (int)std::floor((wy - og.info.origin.position.y) / og.info.resolution);
    }
 
    void set_occupied(nav_msgs::msg::OccupancyGrid &og, int gw, int gh, int gx, int gy) {
        if (gx < 0 || gy < 0 || gx >= gw || gy >= gh) return;
        og.data[(size_t)gx + (size_t)gy * gw] = 100;
    }
 

    void stamp(nav_msgs::msg::OccupancyGrid &og, int gw, int gh, int gx, int gy, int half) {
        for (int dy = -half; dy <= half; dy++)
            for (int dx = -half; dx <= half; dx++)
                set_occupied(og, gw, gh, gx + dx, gy + dy);
    }
 

    void draw_wall(nav_msgs::msg::OccupancyGrid &og, int gw, int gh,
                   int row, int col, int wall, int half) {
     
        double x0, y0, x1, y1;
        const double xL = CELL_SIZE_M * col;
        const double xR = CELL_SIZE_M * (col + 1);
        const double yT = -CELL_SIZE_M * row;          // top edge (higher y)
        const double yB = -CELL_SIZE_M * (row + 1);   
 
        switch (wall) {
            case WALL_TOP:    x0 = xL; y0 = yT; x1 = xR; y1 = yT; break;
            case WALL_BOTTOM: x0 = xL; y0 = yB; x1 = xR; y1 = yB; break;
            case WALL_LEFT:   x0 = xL; y0 = yB; x1 = xL; y1 = yT; break;
            case WALL_RIGHT:  x0 = xR; y0 = yB; x1 = xR; y1 = yT; break;
            default: return;
        }
 

        const double seg_len = std::hypot(x1 - x0, y1 - y0);
        const int steps = std::max(1, (int)std::ceil(seg_len / (og.info.resolution * 0.5)));
        for (int s = 0; s <= steps; s++) {
            double t = (double)s / steps;
            double wx = x0 + t * (x1 - x0);
            double wy = y0 + t * (y1 - y0);
            int gx, gy;
            world_to_grid(og, wx, wy, gx, gy);
            stamp(og, gw, gh, gx, gy, half);
        }
    }
 
    double resolution_;
    double wall_thick_m_;
    bool published_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
    rclcpp::Subscription<map_and_messages::msg::Grid>::SharedPtr sub_;
};
 
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapRasterizer>());
    rclcpp::shutdown();
    return 0;
}
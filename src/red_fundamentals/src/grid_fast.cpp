

#include <rclcpp/rclcpp.hpp>
#include <red_fundamentals/srv/execute_plan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

#include "movement.hpp"   // AddMove, AbortMove, DriveInstruction, turn(), drive(),
                          // abort_move_req(), MOVE_RETURN_SUCCESS

#include <cmath>
#include <mutex>
#include <vector>
#include <thread>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;
using ExecutePlan    = red_fundamentals::srv::ExecutePlan;
using PoseStamped    = geometry_msgs::msg::PoseStamped;
using Int32MultiArray = std_msgs::msg::Int32MultiArray;

static constexpr int RIGHT = 0;
static constexpr int UP    = 1;
static constexpr int LEFT  = 2;
static constexpr int DOWN  = 3;

static constexpr double CELL          = 0.8;
static constexpr double PROX_M        = 0.05;
static constexpr double ANG_TOL_DEG   = 6.0;
static constexpr double STEP_M        = 0.80;
static constexpr double STALE_S       = 0.2;
static constexpr bool   SNAP_FINAL    = false;

static inline double yaw_from_quat(double x, double y, double z, double w) {
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}
static inline double normalize(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}
static inline double deg(double rad) { return rad * 180.0 / M_PI; }

class GridNav : public rclcpp::Node {
public:
    GridNav() : Node("grid_nav") {

        pose_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        plan_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions so;
        so.callback_group = pose_group_;
        pose_sub_ = create_subscription<PoseStamped>(
            "estimated_pose", rclcpp::QoS(10),
            std::bind(&GridNav::pose_cb, this, _1), so);

        add_move_client_   = create_client<AddMove>("add_move");
        abort_move_client_ = create_client<AbortMove>("abort_move");
        plan_pub_          = create_publisher<Int32MultiArray>("current_plan", 10);

        execute_plan_srv_ = create_service<ExecutePlan>(
            "execute_plan",
            std::bind(&GridNav::execute_plan_cb, this, _1, _2),
            rmw_qos_profile_services_default, plan_group_);

        RCLCPP_INFO(get_logger(), "grid_nav ready (closed-loop on /estimated_pose).");
    }

private:
    void pose_cb(const PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(pose_mtx_);
        px_ = msg->pose.position.x;
        py_ = msg->pose.position.y;
        pth_ = yaw_from_quat(msg->pose.orientation.x, msg->pose.orientation.y,
                             msg->pose.orientation.z, msg->pose.orientation.w);
        pose_stamp_ = now();
        have_pose_ = true;
    }

    bool snapshot(double &x, double &y, double &th) {
        std::lock_guard<std::mutex> lk(pose_mtx_);
        if (!have_pose_) return false;
        if ((now() - pose_stamp_).seconds() > STALE_S) return false;
        x = px_; y = py_; th = pth_;
        return true;
    }

    static void worldToCell(double x, double y, int &row, int &col) {
        col = (int)std::floor(x / CELL);
        row = (int)std::floor(-y / CELL);
    }
    static void cellCenter(int row, int col, double &x, double &y) {
        x =  CELL * col + 0.5 * CELL;
        y = -CELL * row - 0.5 * CELL;
    }

    bool send_and_wait(const std::vector<DriveInstruction> &moves) {
        std::vector<rclcpp::Client<AddMove>::SharedFuture> futs;
        for (const auto &m : moves) {
            auto req = std::make_shared<AddMove::Request>();
            req->left = m.left; req->right = m.right;
            futs.push_back(add_move_client_->async_send_request(req).future.share());
            std::this_thread::sleep_for(60ms);
        }
        for (auto &f : futs) {
            f.wait();
            if (f.get()->ret != MOVE_RETURN_SUCCESS) return false;
        }
        return true;
    }
    void abort() { abort_move_req(abort_move_client_); }

    bool driveToGoal(double gx, double gy) {
        const double ang_tol = ANG_TOL_DEG * M_PI / 180.0;

        // Wait until the /estimated_pose has actually changed (prevents
        // sending duplicate commands on the same stale reading).
        auto wait_for_pose_update = [&](double ref_x, double ref_y, double ref_th,
                                        double timeout_s) {
            rclcpp::Time start = now();
            while (rclcpp::ok() && (now() - start).seconds() < timeout_s) {
                double nx, ny, nth;
                if (snapshot(nx, ny, nth)) {
                    if (std::hypot(nx - ref_x, ny - ref_y) > 0.02 ||
                        std::abs(normalize(nth - ref_th)) > ang_tol) {
                        return true;
                    }
                }
                std::this_thread::sleep_for(60ms);
            }
            return false;
        };

        while (rclcpp::ok()) {
            double x, y, th;
            if (!snapshot(x, y, th)) {
                RCLCPP_WARN(get_logger(), "no fresh /estimated_pose -> stopping.");
                abort();
                return false;
            }
            double dx = gx - x, dy = gy - y;
            double dist = std::hypot(dx, dy);
            if (dist < PROX_M) return true;

            double err = normalize(std::atan2(dy, dx) - th);

            RCLCPP_INFO(get_logger(),
                "driveToGoal: pos(%.2f,%.2f) th=%.1f° err=%.1f° dist=%.2f",
                x, y, deg(th), deg(err), dist);

            if (std::fabs(err) > ang_tol) {
                if (!send_and_wait(turn(deg(err)))) { abort(); return false; }
                if (!wait_for_pose_update(x, y, th, 5.0)) {
                    RCLCPP_WARN(get_logger(), "pose did not update after turn");
                }
                std::this_thread::sleep_for(500ms);
            } else {
                double step = std::min(dist, STEP_M);
                if (!send_and_wait(drive(step * 100.0))) { abort(); return false; }
                if (!wait_for_pose_update(x, y, th, 5.0)) {
                    RCLCPP_WARN(get_logger(), "pose did not update after drive");
                }
                std::this_thread::sleep_for(100ms);
            }
        }
        return true;
    }

    bool snapHeading(int last_dir) {
        double target = 0.0;
        switch (last_dir) {
            case RIGHT: target = 0.0;      break;
            case UP:    target = M_PI/2;   break;
            case LEFT:  target = M_PI;     break;
            case DOWN:  target = -M_PI/2;  break;
        }
        double x, y, th;
        if (!snapshot(x, y, th)) return false;
        double err = normalize(target - th);
        if (std::fabs(err) < (ANG_TOL_DEG * M_PI / 180.0)) return true;
        if (!send_and_wait(turn(deg(err)))) return false;

        // Wait for heading to actually change
        rclcpp::Time start = now();
        while (rclcpp::ok() && (now() - start).seconds() < 5.0) {
            double nx, ny, nth;
            if (snapshot(nx, ny, nth)) {
                if (std::abs(normalize(nth - th)) > (ANG_TOL_DEG * M_PI / 180.0))
                    break;
            }
            std::this_thread::sleep_for(60ms);
        }
        return true;
    }

    void execute_plan_cb(const std::shared_ptr<ExecutePlan::Request> req,
                         std::shared_ptr<ExecutePlan::Response> resp) {
        Int32MultiArray pm; pm.data = req->plan; plan_pub_->publish(pm);

        double x, y, th;
        if (!snapshot(x, y, th)) {
            RCLCPP_ERROR(get_logger(), "execute_plan: no /estimated_pose yet.");
            resp->success = false; return;
        }
        int row, col; worldToCell(x, y, row, col);
        RCLCPP_INFO(get_logger(), "start cell (row=%d,col=%d) from pose (%.2f,%.2f)", row, col, x, y);

        std::vector<std::pair<double,double>> goals;
        int last_dir = -1;
        size_t i = 0;
        const auto &plan = req->plan;
        while (i < plan.size()) {
            int d = plan[i];
            while (i < plan.size() && plan[i] == d) {
                switch (d) {
                    case RIGHT: col += 1; break;
                    case LEFT:  col -= 1; break;
                    case UP:    row -= 1; break;
                    case DOWN:  row += 1; break;
                    default:
                        RCLCPP_ERROR(get_logger(), "bad direction code %d", d);
                        resp->success = false; return;
                }
                last_dir = d;
                ++i;
            }
            double gx, gy; cellCenter(row, col, gx, gy);     // far midpoint of this run
            goals.emplace_back(gx, gy);
            RCLCPP_INFO(get_logger(), "  waypoint -> cell(row=%d,col=%d) world(%.2f,%.2f)", row, col, gx, gy);
        }

        for (auto &g : goals) {
            if (!driveToGoal(g.first, g.second)) { resp->success = false; return; }
        }
        if (SNAP_FINAL && last_dir >= 0) snapHeading(last_dir);

        RCLCPP_INFO(get_logger(), "plan complete.");
        resp->success = true;
    }

    rclcpp::CallbackGroup::SharedPtr pose_group_, plan_group_;
    rclcpp::Subscription<PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Client<AddMove>::SharedPtr   add_move_client_;
    rclcpp::Client<AbortMove>::SharedPtr abort_move_client_;
    rclcpp::Publisher<Int32MultiArray>::SharedPtr plan_pub_;
    rclcpp::Service<ExecutePlan>::SharedPtr execute_plan_srv_;

    std::mutex pose_mtx_;
    double px_ = 0, py_ = 0, pth_ = 0;
    rclcpp::Time pose_stamp_;
    bool have_pose_ = false;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GridNav>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
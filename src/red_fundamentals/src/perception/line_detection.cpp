#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <red_fundamentals/msg/line_segment.hpp>
#include <red_fundamentals/msg/line_segment_array.hpp>
#include <Eigen/Dense>
#include <vector>
#include <random>
#include <cmath>
#include <algorithm>
#include <limits>

int MIN_INLIER = 20;
double CONTINUITY_THRESHOLD = 0.25;
double DISTANCE_THRESHOLD = 0.07;
int ITERATIONS = 100;

void ransac(const std::vector<Eigen::Vector2d>& points,
            std::vector<Eigen::Vector2d>& best_inliers, std::vector<Eigen::Vector2d>& best_outliers) {
    int n = points.size();
    
    if (n < 2) {
        best_inliers.clear();
        best_outliers = points;
        return;
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, n - 1);

    std::vector<Eigen::Vector2d> current_best_inliers;
    std::vector<Eigen::Vector2d> current_best_outliers;

    for (int i = 0; i < ITERATIONS; ++i) {
        int idx1 = dis(gen);
        int idx2 = dis(gen);
        while (idx1 == idx2) {
            idx2 = dis(gen);
        }

        Eigen::Vector2d p1 = points[idx1];
        Eigen::Vector2d p2 = points[idx2];

        double A = p2.y() - p1.y();
        double B = p1.x() - p2.x();
        double C = p2.x() * p1.y() - p1.x() * p2.y();

        double norm = std::hypot(A, B);
        if (norm < 0.2) {
            continue;
        }

        A /= norm; 
        B /= norm; 
        C /= norm;

        std::vector<std::pair<double, Eigen::Vector2d>> segment_inliers;
        std::vector<bool> inlier_mask(n, false);
        int inlier_count = 0;

        Eigen::Vector2d dp = p2 - p1;
        double dp_dot = dp.dot(dp);

        for (int j = 0; j < n; ++j) {
            const auto& pt = points[j];
            double dist = std::abs(A * pt.x() + B * pt.y() + C);
            double t = (pt - p1).dot(dp) / dp_dot;
            if (dist < DISTANCE_THRESHOLD && t >= 0.0 && t <= 1.0) {
                inlier_mask[j] = true;
                inlier_count++;
                segment_inliers.push_back({t, pt});
            }
        }

        std::sort(segment_inliers.begin(), segment_inliers.end(),
                  [](const auto& a, const auto& b) { return a.first < b.first; });

        bool continuity_broken = false;
        for (size_t j = 1; j < segment_inliers.size(); ++j) {
            if ((segment_inliers[j].second - segment_inliers[j - 1].second).norm() > CONTINUITY_THRESHOLD) {
                continuity_broken = true;
                break;
            }
        }

        if (continuity_broken) {
            continue;
        }

        if (inlier_count > static_cast<int>(current_best_inliers.size())) {
            current_best_inliers.clear();
            current_best_outliers.clear();
            for (int j = 0; j < n; ++j) {
                if (inlier_mask[j]) {
                    current_best_inliers.push_back(points[j]);
                } else {
                    current_best_outliers.push_back(points[j]);
                }
            }
        }
    }

    best_inliers = current_best_inliers;
    best_outliers = current_best_outliers;
}

std::pair<Eigen::Vector2d, Eigen::Vector2d> linear_regression(const std::vector<Eigen::Vector2d>& points) {
    int n = points.size();
    Eigen::Vector2d mean(0.0, 0.0);
    for (const auto& p : points) {
        mean += p;
    }
    mean /= n;

    Eigen::MatrixXd centered(n, 2);
    for (int i = 0; i < n; ++i) {
        centered(i, 0) = points[i].x() - mean.x();
        centered(i, 1) = points[i].y() - mean.y();
    }

    Eigen::Matrix2d cov = centered.transpose() * centered;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(cov);
    Eigen::Vector2d v = eigensolver.eigenvectors().col(1);

    Eigen::VectorXd projections = centered * v;
    double min_p = projections.minCoeff();
    double max_p = projections.maxCoeff();

    return {mean + min_p * v, mean + max_p * v};
}

std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> line_detection(
    const std::vector<Eigen::Vector2d>& points) {
    
    std::vector<Eigen::Vector2d> pts_2d = points;
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> segments;

    while (pts_2d.size() > static_cast<size_t>(MIN_INLIER)) {
        std::vector<Eigen::Vector2d> inliers, outliers;
        ransac(pts_2d, inliers, outliers);

        if (inliers.size() < static_cast<size_t>(MIN_INLIER)) {
            break;
        }
		// printf("ransac delivered: %zu inliers (min: %d)\n", inliers.size(), MIN_INLIER);

        segments.push_back(linear_regression(inliers));
        pts_2d = outliers;
		// pts_2d = std::vector<Eigen::Vector2d>();
    }

    return segments;
}

class LineDetection : public rclcpp::Node {
public:
    LineDetection() : Node("line_detection") {
        // this->declare_parameter<int>("min_inliers", 20);
        // this->declare_parameter<double>("continuity_threshold", 0.25);
        // this->declare_parameter<double>("distance_threshold", 0.07);
        // this->declare_parameter<int>("iterations", 100);

        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "point_cloud", 10,
            std::bind(&LineDetection::point_cloud_callback, this, std::placeholders::_1));

        wall_publisher_ = this->create_publisher<red_fundamentals::msg::LineSegmentArray>(
            "extracted_walls", 10);

        marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "wall_markers", 10);

        cell_center_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "cell_center", 10);
            
        cell_corner_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "cell_corner", 10);

        RCLCPP_INFO(this->get_logger(), "Line Detection node has been started.");
    }

private:
    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        std::vector<Eigen::Vector2d> points_list;
        
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            if (!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z)) {
                points_list.emplace_back(*iter_x, *iter_y);
            }
        }

        auto segments = line_detection(points_list);

        red_fundamentals::msg::LineSegmentArray line_array;
        line_array.header = msg->header;

        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker marker;
        marker.header = msg->header;
        marker.ns = "walls";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.05;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        for (const auto& segment : segments) {
            red_fundamentals::msg::LineSegment seg;
            seg.start_x = segment.first.x();
            seg.start_y = segment.first.y();
            seg.end_x = segment.second.x();
            seg.end_y = segment.second.y();
            line_array.segments.push_back(seg);

            geometry_msgs::msg::Point p_start;
            p_start.x = segment.first.x();
            p_start.y = segment.first.y();
            p_start.z = 0.0;

            geometry_msgs::msg::Point p_end;
            p_end.x = segment.second.x();
            p_end.y = segment.second.y();
            p_end.z = 0.0;

            marker.points.push_back(p_start);
            marker.points.push_back(p_end);
        }

        if (!marker.points.empty()) {
            marker_array.markers.push_back(marker);
        }

        // --- Calculate Current Cell Center (M) and Corner (C) ---
        double global_min_dist = std::numeric_limits<double>::max();
        Eigen::Vector2d best_m;
        Eigen::Vector2d best_corner;
        bool found_valid = false;

        for (size_t i = 0; i < segments.size(); ++i) {
            for (size_t j = i + 1; j < segments.size(); ++j) {
                Eigen::Vector2d p1 = segments[i].first;
                Eigen::Vector2d p2 = segments[i].second;
                Eigen::Vector2d p3 = segments[j].first;
                Eigen::Vector2d p4 = segments[j].second;

                Eigen::Vector2d v1 = p2 - p1;
                Eigen::Vector2d v2 = p4 - p3;

                double n1 = v1.norm();
                double n2 = v2.norm();

                if (n1 < 0.10 || n2 < 0.10) continue;

                Eigen::Vector2d v1_n = v1 / n1;
                Eigen::Vector2d v2_n = v2 / n2;

                if (std::abs(v1_n.dot(v2_n)) < 0.04) {
                    Eigen::Matrix2d A;
                    A.col(0) = v1;
                    A.col(1) = -v2;
                    Eigen::Vector2d b = p3 - p1;

                    if (A.determinant() != 0) {
                        Eigen::Vector2d lambdas = A.partialPivLu().solve(b);
                        Eigen::Vector2d C = p1 + lambdas[0] * v1; 
                        
                        Eigen::Vector2d n_n(-v1_n.y(), v1_n.x());

                        double x_local = (-C).dot(v1_n);
                        double y_local = (-C).dot(n_n);

                        double m_x_local = std::round((x_local - 0.4) / 0.8) * 0.8 + 0.4;
                        double m_y_local = std::round((y_local - 0.4) / 0.8) * 0.8 + 0.4;

                        Eigen::Vector2d M = C + m_x_local * v1_n + m_y_local * n_n;

                        double dist = C.norm(); 
                        if (dist < global_min_dist) {
                            global_min_dist = dist;
                            best_m = M;
                            best_corner = C;
                            found_valid = true;
                        }
                    }
                }
            }
        }

        geometry_msgs::msg::PointStamped msg_center;
        geometry_msgs::msg::PointStamped msg_corner;
        msg_center.header = msg->header;
        msg_corner.header = msg->header;

        if (found_valid && best_m.norm() < 0.4) {
            msg_center.point.x = best_m.x();
            msg_center.point.y = best_m.y();
            msg_center.point.z = 0.0;
            
            msg_corner.point.x = best_corner.x();
            msg_corner.point.y = best_corner.y();
            msg_corner.point.z = 0.0;
        } else {
            // Signal to align node that center is unavailable
            msg_center.point.x = std::nan("");
            msg_center.point.y = std::nan("");
            msg_corner.point.x = std::nan("");
            msg_corner.point.y = std::nan("");
        }

        cell_center_publisher_->publish(msg_center);
        cell_corner_publisher_->publish(msg_corner);
        wall_publisher_->publish(line_array);
        marker_publisher_->publish(marker_array);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<red_fundamentals::msg::LineSegmentArray>::SharedPtr wall_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr cell_center_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr cell_corner_publisher_;
};

int main(int argc, char **argv) {
	if (argc >= 5) {
		MIN_INLIER = std::stof(argv[1]);
		CONTINUITY_THRESHOLD = std::stof(argv[2]);
		DISTANCE_THRESHOLD = std::stof(argv[3]);
		ITERATIONS = std::stof(argv[4]);
	}

    rclcpp::init(argc, argv);
    auto node = std::make_shared<LineDetection>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

#include <vector>
#include <memory>
#include <mutex>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "map_and_messages/msg/pose.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "movement.hpp"

using std::placeholders::_1;

static inline double yaw_from_quat(double qx, double qy, double qz, double qw)
{
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    return std::atan2(siny_cosp, cosy_cosp);
}

class Util
{
public:
    static double gaussian(double x, double std, double mean = 0)
    {
        return exp(-(x - mean) * (x - mean) / (2.0 * (std * std)));
    }
    static double gaussianRandom(double mean, double std)
    {
        const double norm = 1.0 / (RAND_MAX + 1.0);
        double u = 1.0 - rand() * norm; // can't let u == 0
        double v = rand() * norm;
        double z = sqrt(-2.0 * log(u)) * cos(2.0 * M_PI * v);
        return mean + std * z;
    }
    static double uniformRandom(double min, double max)
    {
        return min + (rand() / (double)RAND_MAX) * (max - min);
    }
    static double diffAngle(double a, double b)
    {
        return normalizeTheta(b - a);
    }
    static double normalizeTheta(double theta)
    {
        if (theta >= M_PI)
            theta -= 2 * M_PI;
        if (theta < -M_PI)
            theta += 2 * M_PI;
        return theta;
    }
};

class Particle
{
public:
    Particle(double x = 0, double y = 0, double theta = 0, double weight = 0)
        : x(x), y(y), theta(theta), weight(weight) {}
    Particle(Particle *p)
        : x(p->x), y(p->y), theta(p->theta), weight(p->weight) {}
    void setZero() { x = y = theta = weight = 0.0; }
    double x, y, theta;
    double weight;
};

static double g_map_origin_x = 0.0;
static double g_map_origin_y = 0.0;
static double g_lf_min_log = -20.0;

class ParticleFilter
{
private:
    std::vector<Particle *> particleSet;
    int numberOfParticles;
    double sumOfParticleWeights;
    Particle *bestHypothesis;

    double odomAlpha1, odomAlpha2, odomAlpha3, odomAlpha4;

    double *likelihoodField;
    int likelihoodFieldWidth, likelihoodFieldHeight;
    double likelihoodFieldResolution;
    int laserSkip;
    double *distMap;
    double laserOffsetX_, laserOffsetY_;

    void sampleMotionModelOdometry(double oldX, double oldY, double oldTheta,
                                   double newX, double newY, double newTheta);
    void likelihoodFieldRangeFinderModel(
        const sensor_msgs::msg::LaserScan::ConstSharedPtr &laserScan);
    void calculateDistanceMap(const nav_msgs::msg::OccupancyGrid &map);
    int computeMapIndex(int width, int height, int x, int y);

public:
    ParticleFilter(int numberOfParticles);
    ~ParticleFilter();

    int getNumberOfParticles();
    std::vector<Particle *> *getParticleSet();

    void initParticlesUniform();
    void initParticlesGaussian(double mean_x, double mean_y, double mean_theta,
                               double std_xx, double std_yy, double std_tt);

    void sampleMotionModel(double oldX, double oldY, double oldTheta,
                           double newX, double newY, double newTheta);
    void setMotionModelOdometry(double a1, double a2, double a3, double a4);

    void measurementModel(
        const sensor_msgs::msg::LaserScan::ConstSharedPtr &laserScan);

    void setMeasurementModelLikelihoodField(
        const nav_msgs::msg::OccupancyGrid &map, double zRand, double sigmaHit);
    void setLaserOffset(double dx, double dy);
    double *getLikelihoodField(int &width, int &height, double &resolution);

    void resample();
    Particle *getBestHypothesis();
};

ParticleFilter::ParticleFilter(int numberOfParticles)
{
    this->numberOfParticles = numberOfParticles;
    for (int i = 0; i < numberOfParticles; i++)
        this->particleSet.push_back(new Particle());
    this->bestHypothesis = new Particle();
    this->laserSkip = 5;
    this->distMap = NULL;
    this->likelihoodField = NULL;
    this->laserOffsetX_ = 0.0;
    this->laserOffsetY_ = 0.0;
    this->sumOfParticleWeights = 0.0;
    srand((unsigned int)time(NULL));
}

ParticleFilter::~ParticleFilter()
{
    for (int i = 0; i < numberOfParticles; i++)
        delete this->particleSet[i];
    this->particleSet.clear();
    if (this->likelihoodField)
        delete[] this->likelihoodField;
    delete this->bestHypothesis;
    if (this->distMap)
        delete[] this->distMap;
}

int ParticleFilter::getNumberOfParticles() { return this->numberOfParticles; }
std::vector<Particle *> *ParticleFilter::getParticleSet() { return &(this->particleSet); }

void ParticleFilter::initParticlesUniform()
{
    int mapWidth, mapHeight;
    double mapResolution;
    this->getLikelihoodField(mapWidth, mapHeight, mapResolution);
    for (int i = 0; i < numberOfParticles; i++)
    {
        Particle *p = this->particleSet[i];
        const double eps = 1e-6;
        p->x = Util::uniformRandom(g_map_origin_x, g_map_origin_x + mapWidth * mapResolution - eps);
        p->y = Util::uniformRandom(g_map_origin_y, g_map_origin_y + mapHeight * mapResolution - eps);
        p->theta = Util::uniformRandom(-M_PI, M_PI);
        p->weight = 1.0 / (double)numberOfParticles;
    }
    *this->bestHypothesis = *this->particleSet[0];
}

void ParticleFilter::initParticlesGaussian(double mean_x, double mean_y,
                                           double mean_theta, double std_xx, double std_yy, double std_tt)
{
    for (int i = 0; i < numberOfParticles; i++)
    {
        Particle *p = this->particleSet[i];
        p->x = Util::gaussianRandom(mean_x, std_xx);
        p->y = Util::gaussianRandom(mean_y, std_yy);
        double th = Util::gaussianRandom(mean_theta, std_tt);
        p->theta = Util::normalizeTheta(th);
        p->weight = 1.0 / (double)numberOfParticles;
    }
    *this->bestHypothesis = *this->particleSet[0];
}

void ParticleFilter::setMeasurementModelLikelihoodField(
    const nav_msgs::msg::OccupancyGrid &map, double zRand, double sigmaHit)
{
    RCLCPP_INFO(rclcpp::get_logger("particle_filter"),
                "Creating likelihood field for laser range finder...");
    this->likelihoodField = new double[map.info.height * map.info.width];
    this->likelihoodFieldWidth = map.info.width;
    this->likelihoodFieldHeight = map.info.height;
    this->likelihoodFieldResolution = map.info.resolution;
    g_map_origin_x = map.info.origin.position.x;
    g_map_origin_y = map.info.origin.position.y;

    calculateDistanceMap(map);

    double sigmaCells = sigmaHit / this->likelihoodFieldResolution;
    for (int x = 0; x < this->likelihoodFieldWidth; x++)
    {
        for (int y = 0; y < this->likelihoodFieldHeight; y++)
        {
            int idx = x + y * this->likelihoodFieldWidth;
            double d = this->distMap[idx];
            double pHit = exp(-(d * d) / (2.0 * sigmaCells * sigmaCells));
            double p = zRand + (1.0 - zRand) * pHit;
            if (p < 1e-12)
                p = 1e-12;
            this->likelihoodField[idx] = log(p);
        }
    }

    g_lf_min_log = this->likelihoodField[0];
    int N = this->likelihoodFieldWidth * this->likelihoodFieldHeight;
    for (int i = 1; i < N; ++i)
        if (this->likelihoodField[i] < g_lf_min_log)
            g_lf_min_log = this->likelihoodField[i];

    RCLCPP_INFO(rclcpp::get_logger("particle_filter"),
                "...DONE creating likelihood field!");
}

void ParticleFilter::calculateDistanceMap(const nav_msgs::msg::OccupancyGrid &map)
{
    distMap = new double[likelihoodFieldWidth * likelihoodFieldHeight];
    int occupiedCellProbability = 90;

    for (int x = 0; x < likelihoodFieldWidth; x++)
        for (int y = 0; y < likelihoodFieldHeight; y++)
            distMap[x + y * likelihoodFieldWidth] = 32000.0;

    for (int x = 0; x < (int)map.info.width; x++)
    {
        for (int y = 0; y < (int)map.info.height; y++)
        {
            if (map.data[x + y * map.info.width] >= occupiedCellProbability)
            {
                bool border = false;
                for (int i = -1; i <= 1; i++)
                {
                    for (int j = -1; j <= 1; j++)
                    {
                        // bounds check for neighbour (x+i, y+j) — fixes original OOB write
                        if (x + i >= 0 && y + j >= 0 && x + i < likelihoodFieldWidth && y + j < likelihoodFieldHeight && (i != 0 || j != 0))
                        {
                            if (!border)
                            {
                                if (map.data[x + i + (y + j) * likelihoodFieldWidth] < occupiedCellProbability && map.data[x + i + (y + j) * likelihoodFieldWidth] >= 0)
                                    border = true;
                            }
                            if (border)
                                distMap[x + i + (y + j) * likelihoodFieldWidth] = 0.0;
                        }
                    }
                }
            }
        }
    }

    // first pass -> SOUTHEAST
    for (int x = 0; x < likelihoodFieldWidth; x++)
        for (int y = 0; y < likelihoodFieldHeight; y++)
            for (int i = -1; i <= 1; i++)
                for (int j = -1; j <= 1; j++)
                    if (x + i >= 0 && y + j >= 0 && x + i < likelihoodFieldWidth && y + j < likelihoodFieldHeight && (i != 0 || j != 0))
                    {
                        double v = distMap[x + i + (y + j) * likelihoodFieldWidth] + ((i * j != 0) ? 1.414 : 1);
                        if (v < distMap[x + y * likelihoodFieldWidth])
                            distMap[x + y * likelihoodFieldWidth] = v;
                    }

    // second pass -> NORTHWEST
    for (int x = likelihoodFieldWidth - 1; x >= 0; x--)
        for (int y = likelihoodFieldHeight - 1; y >= 0; y--)
            for (int i = -1; i <= 1; i++)
                for (int j = -1; j <= 1; j++)
                    if (x + i >= 0 && y + j >= 0 && x + i < likelihoodFieldWidth && y + j < likelihoodFieldHeight && (i != 0 || j != 0))
                    {
                        double v = distMap[x + i + (y + j) * likelihoodFieldWidth] + ((i * j != 0) ? 1.414 : 1);
                        if (v < distMap[x + y * likelihoodFieldWidth])
                            distMap[x + y * likelihoodFieldWidth] = v;
                    }
}

double *ParticleFilter::getLikelihoodField(int &width, int &height, double &resolution)
{
    width = this->likelihoodFieldWidth;
    height = this->likelihoodFieldHeight;
    resolution = this->likelihoodFieldResolution;
    return this->likelihoodField;
}

void ParticleFilter::setLaserOffset(double dx, double dy)
{
    this->laserOffsetX_ = dx;
    this->laserOffsetY_ = dy;
}

void ParticleFilter::measurementModel(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr &laserScan)
{
    likelihoodFieldRangeFinderModel(laserScan);
}

void ParticleFilter::likelihoodFieldRangeFinderModel(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr &laserScan)
{
    int mapW = this->likelihoodFieldWidth;
    int mapH = this->likelihoodFieldHeight;
    double res = this->likelihoodFieldResolution;

    std::vector<double> logW(this->numberOfParticles, 0.0);
    double maxLogW = -1e300;

    const double ldx = this->laserOffsetX_;
    const double ldy = this->laserOffsetY_;

    for (int i = 0; i < this->numberOfParticles; i++)
    {
        Particle *p = this->particleSet[i];
        // laser origin in map frame (base_link pose + rotated laser offset)
        const double lx = p->x + ldx * cos(p->theta) - ldy * sin(p->theta);
        const double ly = p->y + ldx * sin(p->theta) + ldy * cos(p->theta);
        double lw = 0.0;
        for (size_t k = 0; k < laserScan->ranges.size(); k += this->laserSkip)
        {
            double r = laserScan->ranges[k];
            if (r < laserScan->range_min || r > laserScan->range_max)
                continue;
            double angle = laserScan->angle_min + k * laserScan->angle_increment;
            double endX = lx + r * cos(p->theta + angle);
            double endY = ly + r * sin(p->theta + angle);
            int mx = (int)floor((endX - g_map_origin_x) / res);
            int my = (int)floor((endY - g_map_origin_y) / res);
            if (mx < 0 || my < 0 || mx >= mapW || my >= mapH)
            {
                lw += g_lf_min_log;
                continue;
            }
            lw += this->likelihoodField[mx + my * mapW];
        }
        logW[i] = lw;
        if (lw > maxLogW)
            maxLogW = lw;
    }

    this->sumOfParticleWeights = 0.0;
    for (int i = 0; i < this->numberOfParticles; i++)
    {
        double w = exp(logW[i] - maxLogW);
        this->particleSet[i]->weight = w;
        this->sumOfParticleWeights += w;
    }
    for (int i = 0; i < this->numberOfParticles; i++)
        this->particleSet[i]->weight /= this->sumOfParticleWeights;
}

void ParticleFilter::setMotionModelOdometry(double a1, double a2, double a3, double a4)
{
    this->odomAlpha1 = a1;
    this->odomAlpha2 = a2;
    this->odomAlpha3 = a3;
    this->odomAlpha4 = a4;
}

void ParticleFilter::sampleMotionModel(double oldX, double oldY, double oldTheta,
                                       double newX, double newY, double newTheta)
{
    sampleMotionModelOdometry(oldX, oldY, oldTheta, newX, newY, newTheta);
}

void ParticleFilter::sampleMotionModelOdometry(double oldX, double oldY, double oldTheta,
                                               double newX, double newY, double newTheta)
{
    double deltaX = newX - oldX;
    double deltaY = newY - oldY;
    double deltaTrans = sqrt(deltaX * deltaX + deltaY * deltaY);
    double deltaRot1 = Util::diffAngle(oldTheta, atan2(deltaY, deltaX));
    double deltaTheta = Util::diffAngle(oldTheta, newTheta);
    double deltaRot2 = Util::normalizeTheta(deltaTheta - deltaRot1);

    double eps_rot1 = this->odomAlpha1 * fabs(deltaRot1) + this->odomAlpha2 * fabs(deltaTrans);
    double eps_trans = this->odomAlpha3 * fabs(deltaTrans) + this->odomAlpha4 * (fabs(deltaRot1) + fabs(deltaRot2));
    double eps_rot2 = this->odomAlpha1 * fabs(deltaRot2) + this->odomAlpha2 * fabs(deltaTrans);

    for (int i = 0; i < this->numberOfParticles; i++)
    {
        Particle *p = this->particleSet[i];
        double deltaRot1Hat = deltaRot1 + Util::gaussianRandom(0.0, eps_rot1);
        double deltaTransHat = deltaTrans + Util::gaussianRandom(0.0, eps_trans);
        double deltaRot2Hat = deltaRot2 + Util::gaussianRandom(0.0, eps_rot2);
        p->x += deltaTransHat * cos(p->theta + deltaRot1Hat);
        p->y += deltaTransHat * sin(p->theta + deltaRot1Hat);
        p->theta = Util::normalizeTheta(p->theta + deltaRot1Hat + deltaRot2Hat);

        const double eps = 1e-6;
        const double minX = g_map_origin_x;
        const double minY = g_map_origin_y;
        const double maxX = g_map_origin_x + (this->likelihoodFieldWidth * this->likelihoodFieldResolution) - eps;
        const double maxY = g_map_origin_y + (this->likelihoodFieldHeight * this->likelihoodFieldResolution) - eps;
        if (p->x < minX)
            p->x = minX;
        if (p->y < minY)
            p->y = minY;
        if (p->x > maxX)
            p->x = maxX;
        if (p->y > maxY)
            p->y = maxY;
    }
}

void ParticleFilter::resample()
{
    int bestIdx = 0;
    double bestW = this->particleSet[0]->weight;
    for (int i = 1; i < this->numberOfParticles; i++)
        if (this->particleSet[i]->weight > bestW)
        {
            bestW = this->particleSet[i]->weight;
            bestIdx = i;
        }
    *this->bestHypothesis = *this->particleSet[bestIdx];

    std::vector<Particle *> S_new;
    S_new.reserve(this->numberOfParticles);
    double step = 1.0 / (double)this->numberOfParticles;
    double u = Util::uniformRandom(0.0, step);
    double c = this->particleSet[0]->weight;
    int i = 0;
    for (int j = 0; j < this->numberOfParticles; j++)
    {
        double uj = u + j * step;
        while (uj > c && i < this->numberOfParticles - 1)
        {
            i++;
            c += this->particleSet[i]->weight;
        }
        S_new.push_back(new Particle(this->particleSet[i]));
    }
    for (int k = 0; k < this->numberOfParticles; k++)
        delete this->particleSet[k];
    this->particleSet.clear();
    this->particleSet = S_new;

    double w0 = 1.0 / (double)this->numberOfParticles;
    for (int k = 0; k < this->numberOfParticles; k++)
        this->particleSet[k]->weight = w0;
    this->sumOfParticleWeights = 1.0;
}

Particle *ParticleFilter::getBestHypothesis() { return this->bestHypothesis; }

int ParticleFilter::computeMapIndex(int width, int height, int x, int y)
{
    (void)height;
    return x + y * width;
}

static constexpr double CELL_SIZE_M = 0.80; // world metres per maze cell

class ParticleFilterNode : public rclcpp::Node
{
public:
    ParticleFilterNode() : Node("particle_filter")
    {
        // ---- parameters (with sensible defaults; override via launch) ----
        num_particles_ = this->declare_parameter<int>("number_of_particles", 2000);
        init_uniform_ = this->declare_parameter<bool>("init_particles_uniform", false);
        init_x_ = this->declare_parameter<double>("initial_pose_x", 0.0);
        init_y_ = this->declare_parameter<double>("initial_pose_y", 0.0);
        init_theta_ = this->declare_parameter<double>("initial_pose_theta", 0.0);
        init_std_xy_ = this->declare_parameter<double>("initial_std_xy", 0.40);
        init_std_theta_ = this->declare_parameter<double>("initial_std_theta", 0.26);
        double a1 = this->declare_parameter<double>("motionModel_alpha1", 0.1);
        double a2 = this->declare_parameter<double>("motionModel_alpha2", 0.1);
        double a3 = this->declare_parameter<double>("motionModel_alpha3", 0.1);
        double a4 = this->declare_parameter<double>("motionModel_alpha4", 0.1);
        double minLikelihood = this->declare_parameter<double>("sensorModel_minLikelihood", 0.1);
        double sigmaHit = this->declare_parameter<double>("sensorModel_std", 0.2);

        have_last_odom_ = false;
        big_change_ = false;
        last_ox_ = last_oy_ = last_otheta_ = 0.0;

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map_occupancy", rclcpp::QoS(1).transient_local(),
            std::bind(&ParticleFilterNode::map_callback, this, _1));

        pf_ = nullptr;
        pending_a1_ = a1;
        pending_a2_ = a2;
        pending_a3_ = a3;
        pending_a4_ = a4;
        pending_minL_ = minLikelihood;
        pending_sigma_ = sigmaHit;

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("estimated_pose", 2);
        cloud_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("particlecloud", 2);
        grid_pose_pub_ = this->create_publisher<map_and_messages::msg::Pose>("pose_particle", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
        laser_offset_cached_ = false;
        grid_pose_received_ = false;

        // Subscribe to discrete localizer's pose for initial seeding
        initial_pose_sub_ = this->create_subscription<map_and_messages::msg::Pose>(
            "pose", 10,
            std::bind(&ParticleFilterNode::initial_pose_callback, this, _1));

        RCLCPP_INFO(this->get_logger(),
                    "Particle filter node up. Waiting for map on 'map_occupancy' and localized pose on 'pose'...");
    }

private:
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        if (stored_map_)
            return; // store once
        stored_map_ = msg;
        RCLCPP_INFO(this->get_logger(),
                    "Occupancy grid received. Waiting for localized pose on 'pose'...");
        try_init_filter();
    }

    void initial_pose_callback(const map_and_messages::msg::Pose::SharedPtr msg)
    {
        if (grid_pose_received_)
            return;
        initial_grid_col_ = msg->column;
        initial_grid_row_ = msg->row;
        initial_grid_orientation_ = msg->orientation;
        grid_pose_received_ = true;
        RCLCPP_INFO(this->get_logger(),
                    "Localized pose received: col=%d row=%d orient=%d",
                    initial_grid_col_, initial_grid_row_, initial_grid_orientation_);
        try_init_filter();
    }

    void try_init_filter()
    {
        if (!stored_map_ || !grid_pose_received_)
            return;
        if (pf_)
            return; // already built

        double wx, wy, wtheta;
        grid_pose_to_world(initial_grid_col_, initial_grid_row_,
                           initial_grid_orientation_, wx, wy, wtheta);

        RCLCPP_INFO(this->get_logger(),
                    "Seeding particle filter around world pose (%.2f, %.2f, %.2f rad)",
                    wx, wy, wtheta);

        pf_ = std::make_unique<ParticleFilter>(num_particles_);
        pf_->setMotionModelOdometry(pending_a1_, pending_a2_, pending_a3_, pending_a4_);
        pf_->setMeasurementModelLikelihoodField(*stored_map_, pending_minL_, pending_sigma_);

        pf_->initParticlesGaussian(wx, wy, wtheta,
                                   init_std_xy_, init_std_xy_, init_std_theta_);

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan_filtered", 10,
            std::bind(&ParticleFilterNode::scan_callback, this, _1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            std::bind(&ParticleFilterNode::odom_callback, this, _1));

        publish_cloud();
        publish_hypothesis();
        viz_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]()
            {
                if (!pf_)
                    return;
                publish_cloud();
                publish_hypothesis();
            });

        // Schedule an initial 90° turn so particles get motion-model input
        add_move_client_ = this->create_client<AddMove>("add_move");
        initial_turn_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&ParticleFilterNode::do_initial_turn, this));

        RCLCPP_INFO(this->get_logger(),
                    "Filter ready. Seeded from localizer, subscribed to scan_filtered + odom.");
    }

    // Convert discrete grid pose (col, row, orientation) to world coordinates.
    // Maze convention: col -> +x, row -> -y, cell size = CELL_SIZE_M.
    static void grid_pose_to_world(int col, int row, int orientation,
                                   double &wx, double &wy, double &wtheta)
    {
        wx = CELL_SIZE_M * col + CELL_SIZE_M / 2.0;
        wy = -CELL_SIZE_M * row - CELL_SIZE_M / 2.0;
        switch (orientation)
        {
        case 0:
            wtheta = 0.0;
            break; // RIGHT -> +x
        case 1:
            wtheta = M_PI / 2.0;
            break; // TOP -> +y
        case 2:
            wtheta = M_PI;
            break; // LEFT -> -x
        case 3:
        default:
            wtheta = -M_PI / 2.0;
            break; // BOTTOM -> -y
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!pf_)
            return;
        last_stamp_ = msg->header.stamp;

        double newX = msg->pose.pose.position.x;
        double newY = msg->pose.pose.position.y;
        double newTheta = yaw_from_quat(
            msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        odom_x_ = newX;
        odom_y_ = newY;
        odom_theta_ = newTheta;
        have_odom_pose_ = true;

        if (!have_last_odom_)
        {
            last_ox_ = newX;
            last_oy_ = newY;
            last_otheta_ = newTheta;
            have_last_odom_ = true;
            return;
        }

        double dX = newX - last_ox_;
        double dY = newY - last_oy_;
        double dTheta = Util::diffAngle(last_otheta_, newTheta);

        big_change_ = fabs(dX) > 0.025 || fabs(dY) > 0.025 || fabs(dTheta) > 0.1;
        if (big_change_)
        {
            {
                std::lock_guard<std::mutex> lk(pf_mutex_);
                pf_->sampleMotionModel(last_ox_, last_oy_, last_otheta_, newX, newY, newTheta);
            }
            last_ox_ = newX;
            last_oy_ = newY;
            last_otheta_ = newTheta;
            publish_cloud();
        }
        publish_hypothesis();
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (!pf_)
            return;
        last_stamp_ = msg->header.stamp;

        // Look up base_link -> laser transform once to correct beam origins
        if (!laser_offset_cached_) {
            std::string scan_frame = msg->header.frame_id;
            if (!scan_frame.empty() && scan_frame != "base_link") {
                try {
                    auto tf = tf_buffer_->lookupTransform(
                        "base_link", scan_frame, tf2::TimePointZero);
                    std::lock_guard<std::mutex> lk(pf_mutex_);
                    pf_->setLaserOffset(tf.transform.translation.x,
                                        tf.transform.translation.y);
                    RCLCPP_INFO(this->get_logger(),
                        "Laser offset from TF (base_link -> %s): dx=%.3f dy=%.3f",
                        scan_frame.c_str(),
                        tf.transform.translation.x, tf.transform.translation.y);
                } catch (const tf2::TransformException &e) {
                    RCLCPP_WARN(this->get_logger(),
                        "Could not look up base_link -> %s: %s. Using zero offset.",
                        scan_frame.c_str(), e.what());
                }
            }
            laser_offset_cached_ = true;
        }

        if (big_change_)
        {
            {
                std::lock_guard<std::mutex> lk(pf_mutex_);
                sensor_msgs::msg::LaserScan::ConstSharedPtr cscan = msg;
                pf_->measurementModel(cscan);
                pf_->resample();
            }
            big_change_ = false;
            publish_cloud();
        }
        publish_hypothesis();
    }

    void publish_cloud()
    {
        geometry_msgs::msg::PoseArray cloud;
        cloud.header.stamp = this->now();
        cloud.header.frame_id = "map";
        std::lock_guard<std::mutex> lk(pf_mutex_);
        auto *set = pf_->getParticleSet();
        cloud.poses.reserve(set->size());
        for (auto *p : *set)
        {
            geometry_msgs::msg::Pose pose;
            pose.position.x = p->x;
            pose.position.y = p->y;
            tf2::Quaternion q;
            q.setRPY(0, 0, p->theta);
            pose.orientation.x = q.x();
            pose.orientation.y = q.y();
            pose.orientation.z = q.z();
            pose.orientation.w = q.w();
            cloud.poses.push_back(pose);
        }
        cloud_pub_->publish(cloud);
    }

    void publish_hypothesis()
    {
        Particle best;
        {
            std::lock_guard<std::mutex> lk(pf_mutex_);
            best = *pf_->getBestHypothesis();
        }

        tf2::Quaternion q;
        q.setRPY(0, 0, best.theta);
        const rclcpp::Time stamp = this->now();

        // continuous pose for RViz
        geometry_msgs::msg::PoseStamped ps;
        ps.header.stamp = stamp;
        ps.header.frame_id = "map";
        ps.pose.position.x = best.x;
        ps.pose.position.y = best.y;
        ps.pose.orientation.x = q.x();
        ps.pose.orientation.y = q.y();
        ps.pose.orientation.z = q.z();
        ps.pose.orientation.w = q.w();
        pose_pub_->publish(ps);

        const double wx = best.x;
        const double wy = -best.y;
        const int raw_col = (int)std::floor(wx / CELL_SIZE_M);
        const int raw_row = (int)std::floor(wy / CELL_SIZE_M);

        if (!have_reported_)
        {

            reported_col_ = raw_col;
            reported_row_ = raw_row;
            have_reported_ = true;
        }
        else
        {
            reported_col_ = hysteresis_axis(wx, reported_col_);
            reported_row_ = hysteresis_axis(wy, reported_row_);
        }

        map_and_messages::msg::Pose gp;
        gp.column = reported_col_;
        gp.row = reported_row_;
        gp.orientation = theta_to_orientation(best.theta);
        grid_pose_pub_->publish(gp);

         if (have_odom_pose_) {
            // inverse of odom->base_link
            const double inv_th = -odom_theta_;
            const double inv_x = -(std::cos(inv_th) * odom_x_ - std::sin(inv_th) * odom_y_);
            const double inv_y = -(std::sin(inv_th) * odom_x_ + std::cos(inv_th) * odom_y_);

            // compose map->base_link with the inverse
            double mo_x = best.x + std::cos(best.theta) * inv_x - std::sin(best.theta) * inv_y;
            double mo_y = best.y + std::sin(best.theta) * inv_x + std::cos(best.theta) * inv_y;
            double mo_th = best.theta + inv_th;
            while (mo_th >  M_PI) mo_th -= 2.0 * M_PI;
            while (mo_th < -M_PI) mo_th += 2.0 * M_PI;

            tf2::Quaternion mo_q;
            mo_q.setRPY(0, 0, mo_th);

            geometry_msgs::msg::TransformStamped tf;
            tf.header.stamp = stamp;
            tf.header.frame_id = "map";
            tf.child_frame_id = "odom";
            tf.transform.translation.x = mo_x;
            tf.transform.translation.y = mo_y;
            tf.transform.translation.z = 0.0;
            tf.transform.rotation.x = mo_q.x(); tf.transform.rotation.y = mo_q.y();
            tf.transform.rotation.z = mo_q.z(); tf.transform.rotation.w = mo_q.w();
            tf_broadcaster_->sendTransform(tf);
        }
    }

    static int hysteresis_axis(double w, int cur)
    {
        const double HYST_M = 0.20;                // must be >0.20 m past a boundary to switch
        const double lo = cur * CELL_SIZE_M;       // boundary with cell cur-1
        const double hi = (cur + 1) * CELL_SIZE_M; // boundary with cell cur+1
        if (w < lo - HYST_M)
        {

            return (int)std::floor(w / CELL_SIZE_M);
        }
        if (w > hi + HYST_M)
        {

            return (int)std::floor(w / CELL_SIZE_M);
        }
        return cur;
    }

    static int theta_to_orientation(double theta)
    {

        double t = theta;
        while (t > M_PI)
            t -= 2 * M_PI;
        while (t < -M_PI)
            t += 2 * M_PI;
        if (t >= -M_PI / 4 && t < M_PI / 4)
            return 0; // RIGHT
        if (t >= M_PI / 4 && t < 3 * M_PI / 4)
            return 1; // UP
        if (t >= 3 * M_PI / 4 || t < -3 * M_PI / 4)
            return 2; // LEFT
        return 3;     // DOWN
    }
    void do_initial_turn()
    {
        initial_turn_timer_->cancel();
        using namespace std::chrono_literals;
        if (!add_move_client_->wait_for_service(5s)) {
            RCLCPP_WARN(this->get_logger(),
                "add_move service not available after 5s, skipping initial 90° turn");
            return;
        }
        auto moves = turn(90.0f);
        for (auto &m : moves) {
            auto req = std::make_shared<AddMove::Request>();
            req->left = m.left;
            req->right = m.right;
            add_move_client_->async_send_request(req);
        }
        RCLCPP_INFO(this->get_logger(),
            "Initial 90° turn sent to seed particle filter with motion");
    }
    // params
    int num_particles_;
    bool init_uniform_;
    double init_x_, init_y_, init_theta_, init_std_xy_, init_std_theta_;
    double pending_a1_, pending_a2_, pending_a3_, pending_a4_, pending_minL_, pending_sigma_;

    // filter + state
    std::unique_ptr<ParticleFilter> pf_;
    std::mutex pf_mutex_;
    bool have_last_odom_, big_change_;
    double last_ox_, last_oy_, last_otheta_;
    builtin_interfaces::msg::Time last_stamp_;
    bool have_odom_pose_ = false;
    double odom_x_ = 0.0, odom_y_ = 0.0, odom_theta_ = 0.0;

    // hysteresis state for the discrete /pose_particle output
    bool have_reported_ = false;
    int reported_row_ = 0, reported_col_ = 0;

    // deferred init: wait for both map and localized pose
    bool grid_pose_received_ = false;
    int initial_grid_col_ = 0, initial_grid_row_ = 0, initial_grid_orientation_ = 0;
    nav_msgs::msg::OccupancyGrid::SharedPtr stored_map_;

    // ROS handles
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<map_and_messages::msg::Pose>::SharedPtr initial_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr viz_timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr cloud_pub_;
    rclcpp::Publisher<map_and_messages::msg::Pose>::SharedPtr grid_pose_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    bool laser_offset_cached_;

    // initial turn after seeding
    rclcpp::Client<AddMove>::SharedPtr add_move_client_;
    rclcpp::TimerBase::SharedPtr initial_turn_timer_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParticleFilterNode>());
    rclcpp::shutdown();
    return 0;
}
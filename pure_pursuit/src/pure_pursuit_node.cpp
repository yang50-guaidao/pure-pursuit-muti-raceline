#include <sstream>
#include <string>
#include <cmath>
#include <vector>
#include <fstream>
#include <limits>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "raceline_msgs/srv/update_raceline.hpp"

using namespace std;

class PurePursuit : public rclcpp::Node
{
    // Implement PurePursuit
    // This is just a template, you are free to implement your own node!
public:
    PurePursuit() : Node("pure_pursuit_multi_node")
    {

        // Declare params
        this->declare_parameter("use_sim", true);
        this->declare_parameter("csv_path", "");
        this->declare_parameter("lane_csv_paths", std::vector<std::string>{});

        // this->declare_parameter("lookahead_dist", 1.0);
        this->declare_parameter("lookahead_min", 0.5);
        this->declare_parameter("lookahead_max", 2.0);
        this->declare_parameter("lookahead_gain", 0.5);

        this->declare_parameter("lookahead_window", 20);
        this->declare_parameter("lookahead_idx_fwd", 0);  // 0 = distance-based (default); >0 = fixed index offset
        this->declare_parameter("max_curvature", 0.5);

        this->declare_parameter("lookahead_ratio", 8.0);
        this->declare_parameter("use_velocity_lookahead", false);

        this->declare_parameter("fast_speed", 1.5);

        this->declare_parameter("slow_speed", 0.5);
        this->declare_parameter("steering_limit", 0.4189);
        this->declare_parameter("use_waypoint_speed", false);
        this->declare_parameter("brake_lookahead", 0);
        this->declare_parameter("max_accel", 0.0);  // m/s^2, 0 = unlimited
        this->declare_parameter("steer_alpha", 1.0); // 1.0 = no smoothing, 0.3 = heavy smoothing
        this->declare_parameter("use_steer_filter", true);
        this->declare_parameter("steer_deadband", 0.008);
        this->declare_parameter("publish_drive", true); // false when RRT handles driving
        this->declare_parameter("rrt_path_timeout", 0.5); // seconds before RRT path is considered stale
        this->declare_parameter("rrt_lookahead", 1.0);    // shorter lookahead used when RRT overrides steering

        // Multi-lane / opponent params
        this->declare_parameter("opponent_topic", std::string("/opp_racecar/odom"));
        this->declare_parameter("lane_occupied_dist", 0.4);
        this->declare_parameter("min_switch_interval_sec", 0.5);
        this->declare_parameter("lane_lookahead_idx", 30);

        // Get params
        this->get_parameter("use_sim", use_sim_);
        this->get_parameter("csv_path", csv_path_);
        this->get_parameter("lane_csv_paths", lane_csv_paths_);

        // this->get_parameter("lookahead_dist", lookahead_dist_);
        this->get_parameter("lookahead_min", lookahead_min_);
        this->get_parameter("lookahead_max", lookahead_max_);

        this->get_parameter("lookahead_ratio", lookahead_ratio_);
        this->get_parameter("use_velocity_lookahead", use_velocity_lookahead_);

        this->get_parameter("lookahead_window", lookahead_window_);
        this->get_parameter("lookahead_idx_fwd", lookahead_idx_fwd_);
        this->get_parameter("max_curvature", max_curvature_);

        this->get_parameter("fast_speed", fast_speed_);
        this->get_parameter("slow_speed", slow_speed_);
        this->get_parameter("steering_limit", steering_limit_);
        this->get_parameter("use_waypoint_speed", use_waypoint_speed_);
        this->get_parameter("brake_lookahead", brake_lookahead_);
        this->get_parameter("max_accel", max_accel_);
        this->get_parameter("steer_alpha", steer_alpha_);
        this->get_parameter("use_steer_filter", use_steer_filter_);
        this->get_parameter("steer_deadband", steer_deadband_);
        this->get_parameter("publish_drive", publish_drive_);
        this->get_parameter("rrt_path_timeout", rrt_path_timeout_);
        this->get_parameter("rrt_lookahead", rrt_lookahead_);

        this->get_parameter("opponent_topic", opponent_topic_);
        this->get_parameter("lane_occupied_dist", lane_occupied_dist_);
        this->get_parameter("min_switch_interval_sec", min_switch_interval_sec_);
        this->get_parameter("lane_lookahead_idx", lane_lookahead_idx_);

        // Load lanes (multi-lane preferred, single csv_path as fallback)
        init_lanes();

        // Initialize non-param variables
        current_idx_ = 0;
        adaptive_lookahead_ = lookahead_max_;
        prev_time_ = this->get_clock()->now();
        last_switch_time_ = this->get_clock()->now();

        // Create pubs
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive", 10);
        waypoint_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/waypoint_marker", 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/waypoint_path", 10);
        colored_path_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/waypoint_path_colored", 10);
        colored_points_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/waypoint_path_points", 10);
        driven_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/driven_path", 10);
        rrt_goal_pub_ = this->create_publisher<geometry_msgs::msg::Point>(
            "goal_point", 10);
        rrt_target_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/rrt_target_marker", 10); // not used?

        rrt_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "rrt_path", 1,
            [this](const nav_msgs::msg::Path::ConstSharedPtr msg) {
                rrt_path_ = *msg;
                rrt_path_stamp_ = this->get_clock()->now();
            });

        // Create sub based on sim or car
        if (use_sim_) {
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/ego_racecar/odom", 10,
                std::bind(&PurePursuit::odom_callback, this, std::placeholders::_1));
        } else {
            pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/pf/viz/inferred_pose", 10,
                std::bind(&PurePursuit::pose_callback, this, std::placeholders::_1));
        }

        // Opponent odom subscription (only if multi-lane and topic non-empty)
        if (!opponent_topic_.empty() && lanes_.size() > 1) {
            opp_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                opponent_topic_, 10,
                std::bind(&PurePursuit::opp_odom_callback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "Subscribed to opponent odom on '%s'",
                        opponent_topic_.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(),
                        "Opponent tracking disabled (topic='%s', lanes=%zu) — single-lane mode",
                        opponent_topic_.c_str(), lanes_.size());
        }

        update_raceline_srv_ = this->create_service<raceline_msgs::srv::UpdateRaceline>(
            "/pure_pursuit/update_raceline",
            std::bind(&PurePursuit::update_raceline_callback, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(),
                    "Pure Pursuit node initialized: %zu lane(s), active=%d (%s)",
                    lanes_.size(), active_lane_,
                    active_lane_ >= 0 ? lane_names_[active_lane_].c_str() : "none");

    }

    void update_raceline_callback(
        const std::shared_ptr<raceline_msgs::srv::UpdateRaceline::Request> req,
        std::shared_ptr<raceline_msgs::srv::UpdateRaceline::Response> res)
    {
        if (req->format != "pure_pursuit") {
            res->success = false;
            res->message = "expected format='pure_pursuit', got '" + req->format + "'";
            return;
        }
        if (req->cols != 4) {
            res->success = false;
            res->message = "expected cols=4, got " + std::to_string(req->cols);
            return;
        }
        if (req->data.size() != static_cast<size_t>(req->rows) * req->cols) {
            res->success = false;
            res->message = "data length mismatch";
            return;
        }
        std::vector<std::array<double, 4>> new_wps;
        new_wps.reserve(req->rows);
        for (uint32_t i = 0; i < req->rows; ++i) {
            std::array<double, 4> row{};
            for (uint32_t j = 0; j < 4; ++j) {
                row[j] = req->data[i * 4 + j];
            }
            new_wps.push_back(row);
        }
        // Hot-swap the currently-active lane only
        if (active_lane_ < 0 || active_lane_ >= static_cast<int>(lanes_.size())) {
            lanes_.assign(1, std::move(new_wps));
            lane_names_ = {"hotswap"};
            active_lane_ = 0;
        } else {
            lanes_[active_lane_] = std::move(new_wps);
        }
        waypoints_ = lanes_[active_lane_];
        current_idx_ = 0;
        res->success = true;
        res->message = "hot-swapped " + std::to_string(waypoints_.size()) +
                       " waypoints into lane '" + lane_names_[active_lane_] + "'";
        RCLCPP_INFO(this->get_logger(), "%s", res->message.c_str());
    }

private:
    // Pubs
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr waypoint_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr colored_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr colored_points_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr driven_path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr rrt_goal_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rrt_target_pub_;
    nav_msgs::msg::Path driven_path_msg_;

    // Subs
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr rrt_path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr opp_odom_sub_;

    // Services
    rclcpp::Service<raceline_msgs::srv::UpdateRaceline>::SharedPtr update_raceline_srv_;

    // RRT path override
    nav_msgs::msg::Path rrt_path_;
    rclcpp::Time rrt_path_stamp_{0, 0, RCL_ROS_TIME};
    double rrt_path_timeout_ = 0.5;
    double rrt_lookahead_ = 1.0;

    // Lanes [x, y, yaw, speed_ratio]; waypoints_ is a working snapshot of the active lane.
    std::vector<std::vector<std::array<double, 4>>> lanes_;
    std::vector<std::string> lane_names_;
    std::vector<std::array<double, 4>> waypoints_;
    int active_lane_ = -1;

    // Opponent state (NaN until first message)
    double opp_x_ = std::numeric_limits<double>::quiet_NaN();
    double opp_y_ = std::numeric_limits<double>::quiet_NaN();
    rclcpp::Time last_switch_time_;

    // Non-param variables
    int current_idx_;
    double adaptive_lookahead_;
    double prev_steering_angle_ = 0.0;

    // Static params
    bool use_sim_;
    std::string csv_path_;
    std::vector<std::string> lane_csv_paths_;
    std::string opponent_topic_;

    // Dynamic params
    // double lookahead_dist_;
    double lookahead_min_;  // [m]
    double lookahead_max_;

    double lookahead_ratio_;
    bool use_velocity_lookahead_;

    int lookahead_window_;  // indices
    int lookahead_idx_fwd_ = 0;  // >0 = fixed index offset from current_idx_; 0 = use distance-based lookahead
    double max_curvature_;


    double fast_speed_;     // [m/s]
    double slow_speed_;
    double steering_limit_; // [rad]
    bool use_waypoint_speed_;
    int brake_lookahead_;  // how many waypoints ahead to read speed from
    double max_accel_;     // max acceleration rate m/s^2, 0 = unlimited

    double steer_alpha_;   // steering EMA: 1.0 = raw, lower = smoother
    bool use_steer_filter_;
    double steer_deadband_;
    bool publish_drive_;

    double lane_occupied_dist_;
    double min_switch_interval_sec_;
    int lane_lookahead_idx_;

    double prev_speed_ = 0.0;
    rclcpp::Time prev_time_;


    void update_params()
    {
        // this->get_parameter("lookahead_dist", lookahead_dist_);
        this->get_parameter("lookahead_min", lookahead_min_);
        this->get_parameter("lookahead_max", lookahead_max_);

        this->get_parameter("lookahead_window", lookahead_window_);
        this->get_parameter("lookahead_idx_fwd", lookahead_idx_fwd_);
        this->get_parameter("max_curvature", max_curvature_);

        this->get_parameter("lookahead_ratio", lookahead_ratio_);
        this->get_parameter("use_velocity_lookahead", use_velocity_lookahead_);

        this->get_parameter("fast_speed", fast_speed_);
        this->get_parameter("slow_speed", slow_speed_);
        this->get_parameter("steering_limit", steering_limit_);
        this->get_parameter("use_waypoint_speed", use_waypoint_speed_);
        this->get_parameter("brake_lookahead", brake_lookahead_);
        this->get_parameter("max_accel", max_accel_);
        this->get_parameter("steer_alpha", steer_alpha_);
        this->get_parameter("use_steer_filter", use_steer_filter_);
        this->get_parameter("steer_deadband", steer_deadband_);
        this->get_parameter("publish_drive", publish_drive_);
        this->get_parameter("rrt_path_timeout", rrt_path_timeout_);
        this->get_parameter("rrt_lookahead", rrt_lookahead_);

        this->get_parameter("lane_occupied_dist", lane_occupied_dist_);
        this->get_parameter("min_switch_interval_sec", min_switch_interval_sec_);
        this->get_parameter("lane_lookahead_idx", lane_lookahead_idx_);
    }

    static std::string lane_basename(const std::string& path)
    {
        auto slash = path.find_last_of('/');
        std::string name = (slash == std::string::npos) ? path : path.substr(slash + 1);
        auto dot = name.find_last_of('.');
        return (dot == std::string::npos) ? name : name.substr(0, dot);
    }

    bool load_lane_file(const std::string& path,
                        std::vector<std::array<double, 4>>& out)
    {
        std::ifstream file(path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open lane file: %s", path.c_str());
            return false;
        }
        std::string line;
        while (std::getline(file, line)) {
            if (line.empty()) continue;
            std::istringstream ss(line);
            std::string token;
            std::array<double, 4> wp{};
            wp[2] = std::numeric_limits<double>::quiet_NaN();  // yaw default if 3-col CSV
            int i = 0;
            while (std::getline(ss, token, ',') && i < 4) {
                try {
                    wp[i++] = std::stod(token);
                } catch (const std::exception&) {
                    // skip malformed lines (e.g. headers)
                    i = -1;
                    break;
                }
            }
            if (i == 3) {
                // 3-col (x, y, v): shift speed into slot 3, mark yaw NaN
                wp[3] = wp[2];
                wp[2] = std::numeric_limits<double>::quiet_NaN();
                out.push_back(wp);
            } else if (i == 4) {
                out.push_back(wp);
            }
        }
        return !out.empty();
    }

    void init_lanes()
    {
        lanes_.clear();
        lane_names_.clear();

        std::vector<std::string> paths = lane_csv_paths_;
        if (paths.empty() && !csv_path_.empty()) {
            paths.push_back(csv_path_);
        }

        for (const auto& p : paths) {
            std::vector<std::array<double, 4>> wps;
            if (load_lane_file(p, wps)) {
                lane_names_.push_back(lane_basename(p));
                lanes_.push_back(std::move(wps));
                RCLCPP_INFO(this->get_logger(), "  loaded lane '%s' (%zu wps) from %s",
                            lane_names_.back().c_str(), lanes_.back().size(), p.c_str());
            }
        }

        if (lanes_.empty()) {
            RCLCPP_ERROR(this->get_logger(),
                         "No lanes loaded — check 'lane_csv_paths' or 'csv_path' params");
            active_lane_ = -1;
            return;
        }

        // Default to the highest-priority lane (last in list = optimal)
        active_lane_ = static_cast<int>(lanes_.size()) - 1;
        waypoints_ = lanes_[active_lane_];
    }

    void opp_odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        opp_x_ = msg->pose.pose.position.x;
        opp_y_ = msg->pose.pose.position.y;
    }

    int nearest_idx_in_lane(const std::vector<std::array<double, 4>>& lane,
                            double x, double y) const
    {
        int best = 0;
        double best_d2 = std::numeric_limits<double>::max();
        for (size_t i = 0; i < lane.size(); ++i) {
            double dx = lane[i][0] - x;
            double dy = lane[i][1] - y;
            double d2 = dx * dx + dy * dy;
            if (d2 < best_d2) {
                best_d2 = d2;
                best = static_cast<int>(i);
            }
        }
        return best;
    }

    // Returns true and sets `min_dist_out` if any waypoint within the forward
    // window of `lane` lies within `lane_occupied_dist_` of (opp_x_, opp_y_).
    bool lane_blocked(const std::vector<std::array<double, 4>>& lane,
                      double ego_x, double ego_y,
                      double& min_dist_out) const
    {
        int n = static_cast<int>(lane.size());
        if (n == 0) {
            min_dist_out = std::numeric_limits<double>::infinity();
            return false;
        }
        int near = nearest_idx_in_lane(lane, ego_x, ego_y);
        int window = std::min(lane_lookahead_idx_, n);
        double min_d = std::numeric_limits<double>::infinity();
        for (int k = 0; k < window; ++k) {
            int idx = (near + k) % n;
            double dx = lane[idx][0] - opp_x_;
            double dy = lane[idx][1] - opp_y_;
            double d = std::sqrt(dx * dx + dy * dy);
            if (d < min_d) min_d = d;
        }
        min_dist_out = min_d;
        return min_d < lane_occupied_dist_;
    }

    void update_active_lane(double ego_x, double ego_y)
    {
        if (lanes_.size() < 2) return;                       // nothing to switch to
        if (std::isnan(opp_x_) || std::isnan(opp_y_)) return; // no opponent yet

        std::vector<bool> blocked(lanes_.size(), false);
        std::vector<double> min_d(lanes_.size(), 0.0);
        for (size_t i = 0; i < lanes_.size(); ++i) {
            blocked[i] = lane_blocked(lanes_[i], ego_x, ego_y, min_d[i]);
        }

        // Pick highest-priority free lane (last index = highest priority).
        int desired = active_lane_;
        for (int i = static_cast<int>(lanes_.size()) - 1; i >= 0; --i) {
            if (!blocked[i]) { desired = i; break; }
        }
        // If everything is blocked, hold current lane (controller will slow due to brake_lookahead).
        if (std::all_of(blocked.begin(), blocked.end(), [](bool b){ return b; })) return;

        if (desired == active_lane_) return;

        auto now = this->get_clock()->now();
        if ((now - last_switch_time_).seconds() < min_switch_interval_sec_) return;

        RCLCPP_INFO(this->get_logger(),
                    "lane switch: %s -> %s  (blocked: %s min_d=%.2f, switching to min_d=%.2f)",
                    lane_names_[active_lane_].c_str(),
                    lane_names_[desired].c_str(),
                    blocked[active_lane_] ? "yes" : "no",
                    min_d[active_lane_],
                    min_d[desired]);

        active_lane_ = desired;
        waypoints_ = lanes_[active_lane_];
        // Reset the search anchor to the nearest point on the new lane.
        current_idx_ = nearest_idx_in_lane(waypoints_, ego_x, ego_y);
        last_switch_time_ = now;
    }

    double quaternion_to_yaw(double qx, double qy, double qz, double qw)
    {
        double term1 = 2.0 * (qw * qz + qx * qy);
        double term2 = 1.0 - 2.0 * (qy * qy + qz * qz);
        return std::atan2(term1, term2);
    }

    double compute_adaptive_lookahead()
    {
        // Velocity changes
        double L_velocity;
        if (use_velocity_lookahead_) {
            L_velocity = lookahead_max_ * prev_speed_ / lookahead_ratio_;
            L_velocity = std::max(lookahead_min_, std::min(lookahead_max_, L_velocity));
        }

        int n = static_cast<int>(waypoints_.size());
        double max_curvature_local = 0.0;

        for (int i = 1; i < lookahead_window_ - 1; i++) {
            int i0 = (current_idx_ + i - 1) % n;
            int i1 = (current_idx_ + i    ) % n;
            int i2 = (current_idx_ + i + 1) % n;

            double p0x = waypoints_[i0][0], p0y = waypoints_[i0][1];
            double p1x = waypoints_[i1][0], p1y = waypoints_[i1][1];
            double p2x = waypoints_[i2][0], p2y = waypoints_[i2][1];

            double d1x = p1x - p0x, d1y = p1y - p0y;
            double d2x = p2x - p1x, d2y = p2y - p1y;

            double cross = std::abs(d1x * d2y - d1y * d2x);
            double denom = std::sqrt(d1x*d1x + d1y*d1y) *
                        std::sqrt(d2x*d2x + d2y*d2y) *
                        std::sqrt((p2x-p0x)*(p2x-p0x) + (p2y-p0y)*(p2y-p0y));

            double curvature = (denom > 1e-6) ? (cross / denom) : 0.0;
            max_curvature_local = std::max(max_curvature_local, curvature);
        }

        // Normalize curvature to [0,1]
        double normalized = std::min(max_curvature_local / max_curvature_, 1.0);

        // veloicty changes
        double L_curvature;
        double L_final;
        if (use_velocity_lookahead_){
            L_curvature = lookahead_max_ - normalized * (lookahead_max_ - lookahead_min_);
            L_final = std::min(L_velocity, L_curvature);
            return std::max(lookahead_min_, std::min(lookahead_max_, L_final));
        } else {
            // High curvature use lookahead_min, low curvature use lookahead_max
            return lookahead_max_ - normalized * (lookahead_max_ - lookahead_min_);
        }
    }

    int find_lookahead_waypoint(double x, double y)
    {
        int n = static_cast<int>(waypoints_.size());

        // Update current_idx_ — search forward-only within a window from current
        // (prevents jumping backward on overlapping-track segments with noisy pose)
        int search_window = std::max(n / 4, 10);
        double min_dist = std::numeric_limits<double>::max();
        for (int i = 0; i < search_window; i++) {
            int idx = (current_idx_ + i) % n;
            double dx = waypoints_[idx][0] - x;
            double dy = waypoints_[idx][1] - y;
            double dist = std::sqrt(dx * dx + dy * dy);
            if (dist < min_dist) {
                min_dist = dist;
                current_idx_ = idx;
            }
        }

        // Mode A: fixed index offset (exploits curvature-biased waypoint density)
        if (lookahead_idx_fwd_ > 0) {
            int goal_idx = (current_idx_ + lookahead_idx_fwd_) % n;
            // Track adaptive_lookahead_ for logging/steering-geometry consistency
            double dx = waypoints_[goal_idx][0] - x;
            double dy = waypoints_[goal_idx][1] - y;
            adaptive_lookahead_ = std::sqrt(dx * dx + dy * dy);
            return goal_idx;
        }

        // Mode B: distance-based adaptive lookahead (legacy, uses compute_adaptive_lookahead)
        adaptive_lookahead_ = compute_adaptive_lookahead();
        int goal_idx = -1;
        for (int i = 0; i < n; i++) {
            int idx = (current_idx_ + i) % n;
            double dx = waypoints_[idx][0] - x;
            double dy = waypoints_[idx][1] - y;
            double dist = std::sqrt(dx * dx + dy * dy);
            if (dist >= adaptive_lookahead_) {
                goal_idx = idx;
                break;
            }
        }
        if (goal_idx < 0) {
            RCLCPP_WARN(this->get_logger(), "No waypoint found within lookahead distance");
            return -1;
        }
        return goal_idx;
    }

    void transform_to_vehicle_frame(double car_x, double car_y, double car_yaw,
                                  double goal_x, double goal_y,
                                  double& local_x, double& local_y)
    {
        // Translate so car is at origin
        double dx = goal_x - car_x;
        double dy = goal_y - car_y;

        // Rotate into vehicle frame
        local_x = dx * std::cos(car_yaw) + dy * std::sin(car_yaw);
        local_y = -dx * std::sin(car_yaw) + dy * std::cos(car_yaw);
    }

    double calculate_steering_angle(double local_x, double local_y)
    {
        // Calc curvature using actual target distance (not the lookahead threshold)
        double L = std::hypot(local_x, local_y);
        if (L < 1e-3) return 0.0;
        double curvature = (2.0 * local_y) / (L * L);

        // Convert curvature to steering angle
        double steering_angle = std::atan(curvature);

        // Clamp steering angle
        steering_angle = std::max(-steering_limit_, std::min(steering_limit_, steering_angle));

        return steering_angle;
    }

    void publish_drive(double steering_angle, int goal_idx, bool rrt_active = false)
    {
        // Reactive speed: linearly interpolate using normalized steer angle
        double normalized = std::min(std::abs(steering_angle) / steering_limit_, 1.0);
        double reactive_speed = fast_speed_ - normalized * (fast_speed_ - slow_speed_);

        double speed;
        // When RRT is overriding steering, waypoint speed is irrelevant (it's for the raceline).
        // Fall through to reactive speed, which is already proportional to how hard we're turning.
        if (!rrt_active && use_waypoint_speed_ && goal_idx >= 0 &&
            goal_idx < static_cast<int>(waypoints_.size())) {
            // Speed column is a ratio 0-1: 0 = slow_speed, 1 = fast_speed
            // brake_lookahead > 0: average of next N (balanced)
            // brake_lookahead < 0: min of next |N| (conservative, old method)
            // brake_lookahead = 0: current waypoint only
            int n = static_cast<int>(waypoints_.size());
            double ratio = waypoints_[goal_idx][3];
            if (brake_lookahead_ > 0) {
                double sum = ratio;
                for (int i = 1; i <= brake_lookahead_; i++) {
                    int idx = (goal_idx + i) % n;
                    sum += waypoints_[idx][3];
                }
                ratio = sum / (brake_lookahead_ + 1);
            } else if (brake_lookahead_ < 0) {
                for (int i = 1; i <= -brake_lookahead_; i++) {
                    int idx = (goal_idx + i) % n;
                    ratio = std::min(ratio, waypoints_[idx][3]);
                }
            }
            ratio = std::max(0.0, std::min(1.0, ratio));
            double wp_speed = slow_speed_ + ratio * (fast_speed_ - slow_speed_);
            // Trust waypoint speed — only use reactive as emergency brake
            // (reactive kicks in only if steering is near the limit)
            speed = (normalized > 0.7) ? std::min(wp_speed, reactive_speed) : wp_speed;
        } else {
            speed = reactive_speed;
        }

        // Rate-limit acceleration only (braking stays instant)
        if (max_accel_ > 0.0 && speed > prev_speed_) {
            auto now = this->get_clock()->now();
            double dt = (now - prev_time_).seconds();
            if (dt > 0.0 && dt < 1.0) {
                double max_increase = max_accel_ * dt;
                speed = std::min(speed, prev_speed_ + max_increase);
            }
            prev_time_ = now;
        } else {
            prev_time_ = this->get_clock()->now();
        }
        prev_speed_ = speed;

        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = this->get_clock()->now();
        drive_msg.header.frame_id = "base_link";
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = speed;
        drive_pub_->publish(drive_msg);
    }

    void visualize_target_waypoint(double x, double y)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->get_clock()->now();
        marker.header.frame_id = "map";
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.w = 1.0;
        waypoint_pub_->publish(marker);
    }

    void visualize_waypoint_path()
    {
        auto now = this->get_clock()->now();
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = now;
        path_msg.header.frame_id = "map";

        // Colored LINE_STRIP — each point tinted by its speed ratio
        visualization_msgs::msg::Marker cmarker;
        cmarker.header.stamp = now;
        cmarker.header.frame_id = "map";
        cmarker.ns = "waypoint_path_colored";
        cmarker.id = 0;
        cmarker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        cmarker.action = visualization_msgs::msg::Marker::ADD;
        cmarker.scale.x = 0.04;  // line thickness
        cmarker.pose.orientation.w = 1.0;

        // Colored SPHERE_LIST — per-waypoint dots
        visualization_msgs::msg::Marker pmarker;
        pmarker.header.stamp = now;
        pmarker.header.frame_id = "map";
        pmarker.ns = "waypoint_path_points";
        pmarker.id = 0;
        pmarker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        pmarker.action = visualization_msgs::msg::Marker::ADD;
        pmarker.scale.x = pmarker.scale.y = pmarker.scale.z = 0.08;
        pmarker.pose.orientation.w = 1.0;

        for (const auto& wp : waypoints_) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = wp[0];
            pose.pose.position.y = wp[1];
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);

            // Same point for colored line + sphere markers
            geometry_msgs::msg::Point p;
            p.x = wp[0]; p.y = wp[1]; p.z = 0.05;
            cmarker.points.push_back(p);
            pmarker.points.push_back(p);

            // Map speed ratio → color gradient (red slow → cyan fast)
            double ratio = std::clamp(wp[3], 0.0, 1.0);
            std_msgs::msg::ColorRGBA c;
            if (ratio < 0.5) {
                double t = ratio * 2.0;  // 0..1
                c.r = 1.0;
                c.g = t;
                c.b = 0.0;
            } else {
                double t = (ratio - 0.5) * 2.0;  // 0..1
                c.r = 1.0 - t;
                c.g = 1.0;
                c.b = t;
            }
            c.a = 1.0;
            cmarker.colors.push_back(c);
            pmarker.colors.push_back(c);
        }
        // Close the loop visually
        if (!waypoints_.empty()) {
            geometry_msgs::msg::Point p;
            p.x = waypoints_[0][0]; p.y = waypoints_[0][1]; p.z = 0.05;
            cmarker.points.push_back(p);
            cmarker.colors.push_back(cmarker.colors.front());
        }

        path_pub_->publish(path_msg);
        colored_path_pub_->publish(cmarker);
        colored_points_pub_->publish(pmarker);
    }

    void publish_driven_path(double x, double y)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = this->get_clock()->now();
        pose.header.frame_id = "map";
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;
        driven_path_msg_.poses.push_back(pose);

        // Keep last 2000 poses to avoid unbounded growth
        if (driven_path_msg_.poses.size() > 2000) {
            driven_path_msg_.poses.erase(driven_path_msg_.poses.begin());
        }

        driven_path_msg_.header.stamp = this->get_clock()->now();
        driven_path_msg_.header.frame_id = "map";
        driven_path_pub_->publish(driven_path_msg_);
    }

    void run_pure_pursuit(double x, double y, double qx, double qy, double qz, double qw)
    {
        if (waypoints_.empty()) return;

        // Multi-lane decision: maybe switch active lane based on opponent occupancy
        update_active_lane(x, y);

        // Get cars current yaw
        double yaw = quaternion_to_yaw(qx, qy, qz, qw);

        // Find the best waypoint to track
        int goal_idx = find_lookahead_waypoint(x, y);
        if (goal_idx < 0) return;

        // Transform goal waypoint to vehicle frame
        double goal_x, goal_y;
        transform_to_vehicle_frame(x, y, yaw,
                                    waypoints_[goal_idx][0],
                                    waypoints_[goal_idx][1],
                                    goal_x, goal_y);

        // Publish goal point for RRT (in vehicle frame)
        geometry_msgs::msg::Point goal_msg;
        goal_msg.x = goal_x;
        goal_msg.y = goal_y;
        rrt_goal_pub_->publish(goal_msg);

        // Override with RRT path if fresh
        bool rrt_active = false;
        double age = (this->get_clock()->now() - rrt_path_stamp_).seconds();
        if (!rrt_path_.poses.empty() && age < rrt_path_timeout_) {
            // Walk RRT path (local frame, car at origin) and pick first pose >= rrt_lookahead_
            // Shorter lookahead than the racing lookahead so we aim near the apex of the detour
            // (where |y| is largest), producing stronger steering to actually clear the obstacle.
            const geometry_msgs::msg::Point* rrt_target = &rrt_path_.poses.back().pose.position;
            for (const auto& pose : rrt_path_.poses) {
                if (std::hypot(pose.pose.position.x, pose.pose.position.y) >= rrt_lookahead_) {
                    rrt_target = &pose.pose.position;
                    break;
                }
            }
            if (std::hypot(rrt_target->x, rrt_target->y) > 0.05) {
                goal_x = rrt_target->x;
                goal_y = rrt_target->y;
                rrt_active = true;
            }
        }

        // Calc steering angle (uses actual target distance, so works for both raceline and RRT targets)
        double raw_steering = calculate_steering_angle(goal_x, goal_y);
        double steering_angle;

        if (use_steer_filter_) {
            double delta = raw_steering - prev_steering_angle_;
            if (std::abs(delta) < steer_deadband_) {
                // change < deadband --> treat as noise, hold previous val
                steering_angle = prev_steering_angle_;
            } else {
                steering_angle = steer_alpha_ * raw_steering + (1.0 - steer_alpha_) * prev_steering_angle_;
            }
        } else {
            // filter off --> pass raw steering
            steering_angle = raw_steering;
        }

        prev_steering_angle_ = steering_angle;

        // Log RRT state changes for tuning
        static bool prev_rrt_active = false;
        if (rrt_active != prev_rrt_active) {
            RCLCPP_INFO(this->get_logger(), "[RRT] %s (steer=%.3f rad)",
                rrt_active ? "ACTIVE — overriding pure pursuit" : "INACTIVE — back to raceline",
                steering_angle);
            prev_rrt_active = rrt_active;
        }

        // Publish drive message (skip if RRT is handling driving)
        if (publish_drive_) publish_drive(steering_angle, goal_idx, rrt_active);

        // Visualize the full path
        visualize_waypoint_path();

        // Visualize current target waypoint
        visualize_target_waypoint(waypoints_[goal_idx][0], waypoints_[goal_idx][1]);

        // Publish driven path (actual car trajectory)
        publish_driven_path(x, y);

    }

    // Callback for sim
    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        update_params();
        run_pure_pursuit(msg->pose.pose.position.x,
                         msg->pose.pose.position.y,
                         msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z,
                         msg->pose.pose.orientation.w);
    }

    // Callback for real car
    void pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
    {
        update_params();
        run_pure_pursuit(msg->pose.position.x,
                         msg->pose.position.y,
                         msg->pose.orientation.x,
                         msg->pose.orientation.y,
                         msg->pose.orientation.z,
                         msg->pose.orientation.w);
    }

};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}

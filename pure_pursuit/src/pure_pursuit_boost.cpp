#include <sstream>
#include <string>
#include <cmath>
#include <vector>
#include <fstream>
#include <limits>

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
/// CHECK: include needed ROS msg type headers and libraries

using namespace std;

class PurePursuitBoost : public rclcpp::Node
{
public:
    PurePursuitBoost() : Node("pure_pursuit_boost_node")
    {

        // Declare params
        this->declare_parameter("use_sim", true);
        this->declare_parameter("csv_path", "");

        // this->declare_parameter("lookahead_dist", 1.0);
        this->declare_parameter("lookahead_min", 0.5);
        this->declare_parameter("lookahead_max", 2.0);
        this->declare_parameter("lookahead_gain", 0.5);

        this->declare_parameter("lookahead_window", 20);
        this->declare_parameter("max_curvature", 0.5);

        this->declare_parameter("lookahead_ratio", 8.0);
        this->declare_parameter("use_velocity_lookahead", false);

        this->declare_parameter("fast_speed", 1.5);
        
        this->declare_parameter("slow_speed", 0.5);
        this->declare_parameter("steering_limit", 0.4189);
        this->declare_parameter("use_waypoint_speed", false);
        this->declare_parameter("brake_lookahead", 0);  // legacy, still works if no col5
        this->declare_parameter("min_brake_lookahead", 5);   // used when col5 = 0
        this->declare_parameter("avg_brake_lookahead", 3);   // used when col5 = 1
        this->declare_parameter("max_accel", 0.0);  // m/s^2, 0 = unlimited
        this->declare_parameter("steer_alpha", 1.0); // 1.0 = no smoothing, 0.3 = heavy smoothing
        this->declare_parameter("use_steer_filter", true);
        this->declare_parameter("steer_deadband", 0.008);

        // Get params
        this->get_parameter("use_sim", use_sim_);
        this->get_parameter("csv_path", csv_path_);

        // this->get_parameter("lookahead_dist", lookahead_dist_);
        this->get_parameter("lookahead_min", lookahead_min_);
        this->get_parameter("lookahead_max", lookahead_max_);

        this->get_parameter("lookahead_ratio", lookahead_ratio_);
        this->get_parameter("use_velocity_lookahead", use_velocity_lookahead_);

        this->get_parameter("lookahead_window", lookahead_window_);
        this->get_parameter("max_curvature", max_curvature_);

        this->get_parameter("fast_speed", fast_speed_);
        this->get_parameter("slow_speed", slow_speed_);
        this->get_parameter("steering_limit", steering_limit_);
        this->get_parameter("use_waypoint_speed", use_waypoint_speed_);
        this->get_parameter("brake_lookahead", brake_lookahead_);
        this->get_parameter("min_brake_lookahead", min_brake_lookahead_);
        this->get_parameter("avg_brake_lookahead", avg_brake_lookahead_);
        this->get_parameter("max_accel", max_accel_);
        this->get_parameter("steer_alpha", steer_alpha_);
        this->get_parameter("use_steer_filter", use_steer_filter_);
        this->get_parameter("steer_deadband", steer_deadband_);

        // Load waypoints from CSV
        load_waypoints(csv_path_);

        // Initialize non-param variables
        current_idx_ = 0;
        adaptive_lookahead_ = lookahead_max_;
        prev_time_ = this->get_clock()->now();
        // prev_speed_ = 0.0;
        // prev_steering_angle_ = 0.0;

        // Create pubs
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive", 10);
        waypoint_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/waypoint_marker", 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/waypoint_path", 10);
        driven_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/driven_path", 10);

        // Create sub based on sim or car
        if (use_sim_) {
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/ego_racecar/odom", 10,
                std::bind(&PurePursuitBoost::odom_callback, this, std::placeholders::_1));
        } else {
            pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/pf/viz/inferred_pose", 10,
                std::bind(&PurePursuitBoost::pose_callback, this, std::placeholders::_1));
        }

        RCLCPP_INFO(this->get_logger(), "Pure Pursuit node initialized");

    }

private:
    // Pubs
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr waypoint_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr driven_path_pub_;
    nav_msgs::msg::Path driven_path_msg_;

    // Subs
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

    // Waypoints [x, y, yaw, speed_ratio, brake_mode]
    // brake_mode: 0 = min lookahead (conservative), 1 = avg lookahead (balanced)
    std::vector<std::array<double, 5>> waypoints_;

    // Non-param variables
    int current_idx_;
    double adaptive_lookahead_;
    double prev_steering_angle_ = 0.0;

    // Static params
    bool use_sim_;
    std::string csv_path_;

    // Dynamic params
    // double lookahead_dist_;
    double lookahead_min_;  // [m]
    double lookahead_max_;
    
    double lookahead_ratio_;
    bool use_velocity_lookahead_;

    int lookahead_window_;  // indices
    double max_curvature_;


    double fast_speed_;     // [m/s]
    double slow_speed_;
    double steering_limit_; // [rad]
    bool use_waypoint_speed_;
    int brake_lookahead_;  // legacy: how many waypoints ahead to read speed from
    int min_brake_lookahead_;  // per-waypoint: min of next N (col5=0)
    int avg_brake_lookahead_;  // per-waypoint: avg of next N (col5=1)
    double max_accel_;     // max acceleration rate m/s^2, 0 = unlimited

    double steer_alpha_;   // steering EMA: 1.0 = raw, lower = smoother
    bool use_steer_filter_;
    double steer_deadband_;

    double prev_speed_ = 0.0;
    rclcpp::Time prev_time_;


    void update_params()
    {
        // this->get_parameter("lookahead_dist", lookahead_dist_);
        this->get_parameter("lookahead_min", lookahead_min_);
        this->get_parameter("lookahead_max", lookahead_max_);

        this->get_parameter("lookahead_window", lookahead_window_);
        this->get_parameter("max_curvature", max_curvature_); 
        
        this->get_parameter("lookahead_ratio", lookahead_ratio_);
        this->get_parameter("use_velocity_lookahead", use_velocity_lookahead_);

        this->get_parameter("fast_speed", fast_speed_);
        this->get_parameter("slow_speed", slow_speed_);
        this->get_parameter("steering_limit", steering_limit_);
        this->get_parameter("use_waypoint_speed", use_waypoint_speed_);
        this->get_parameter("brake_lookahead", brake_lookahead_);
        this->get_parameter("min_brake_lookahead", min_brake_lookahead_);
        this->get_parameter("avg_brake_lookahead", avg_brake_lookahead_);
        this->get_parameter("max_accel", max_accel_);
        this->get_parameter("steer_alpha", steer_alpha_);
        this->get_parameter("use_steer_filter", use_steer_filter_);
        this->get_parameter("steer_deadband", steer_deadband_);
    }

    void load_waypoints(const std::string& path)
    {
        std::ifstream file(path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open waypoints file: %s", path.c_str());
            return;
        }

        std::string line;
        while (std::getline(file, line)) {
            std::istringstream ss(line);
            std::string token;
            std::array<double, 5> waypoint = {0, 0, 0, 0, 0};  // col5 defaults to 0 (min)
            int i = 0;
            while (std::getline(ss, token, ',') && i < 5) {
                waypoint[i++] = std::stod(token);
            }
            waypoints_.push_back(waypoint);
        }
        file.close();
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
        int goal_idx = -1;
        int n = static_cast<int>(waypoints_.size());

        adaptive_lookahead_ = compute_adaptive_lookahead();
        // adaptive_lookahead_ = lookahead_dist_;

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

        // Update current_idx_ — only search forward within a window
        // Prevents jumping to a different segment on tracks that overlap
        int search_window = std::max(n / 4, 10);  // search up to 25% of track ahead
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

    double calculate_steering_angle(double local_y)
    {
        // Calc curvature
        double curvature = (2.0 * local_y) / (adaptive_lookahead_ * adaptive_lookahead_);

        // Convert curvature to steering angle
        double steering_angle = std::atan(curvature);

        // Clamp steering angle
        steering_angle = std::max(-steering_limit_, std::min(steering_limit_, steering_angle));

        return steering_angle;
    }

    void publish_drive(double steering_angle, int goal_idx)
    {
        // Reactive speed: linearly interpolate using normalized steer angle
        double normalized = std::min(std::abs(steering_angle) / steering_limit_, 1.0);
        double reactive_speed = fast_speed_ - normalized * (fast_speed_ - slow_speed_);

        double speed;
        if (use_waypoint_speed_ && goal_idx >= 0 &&
            goal_idx < static_cast<int>(waypoints_.size())) {
            // Speed column is a ratio 0-1: 0 = slow_speed, 1 = fast_speed
            // Col5 (brake_mode): 0 = min of next N, 1 = avg of next N
            int n = static_cast<int>(waypoints_.size());
            double ratio = waypoints_[goal_idx][3];
            int brake_mode = static_cast<int>(waypoints_[goal_idx][4]);
            int lookahead = (brake_mode == 1) ? avg_brake_lookahead_ : min_brake_lookahead_;

            if (lookahead > 0) {
                if (brake_mode == 1) {
                    // Average method
                    double sum = ratio;
                    for (int i = 1; i <= lookahead; i++) {
                        int idx = (goal_idx + i) % n;
                        sum += waypoints_[idx][3];
                    }
                    ratio = sum / (lookahead + 1);
                } else {
                    // Min method (conservative)
                    for (int i = 1; i <= lookahead; i++) {
                        int idx = (goal_idx + i) % n;
                        ratio = std::min(ratio, waypoints_[idx][3]);
                    }
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
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
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
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->get_clock()->now();
        path_msg.header.frame_id = "map";

        for (const auto& wp : waypoints_) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = wp[0];
            pose.pose.position.y = wp[1];
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }

        path_pub_->publish(path_msg);
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

        // Calc steering angle
        double raw_steering = calculate_steering_angle(goal_y);
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

        // Publish drive message
        publish_drive(steering_angle, goal_idx);

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
    rclcpp::spin(std::make_shared<PurePursuitBoost>());
    rclcpp::shutdown();
    return 0;
}
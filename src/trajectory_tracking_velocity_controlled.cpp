#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <vector>
#include <cmath>
#include <limits>

class HelixTrajectoryTracker : public rclcpp::Node {
public:
    HelixTrajectoryTracker()
    : Node("helix_trajectory_tracker"), received_pose_(false), current_wp_idx_(0)
    {
        // --- Generate helix waypoints and path message ---
        double radius = 1.0;
        double omega = 2 * M_PI / 5.0;
        double vx = 0.5;
        double duration = 20.0;
        double dt = 0.2;

        planned_path_.header.frame_id = "map";
        for (double t = 0.0; t <= duration; t += dt) {
            double x = vx * t;
            double y = radius * std::cos(omega * t);
            double z = radius * std::sin(omega * t) + 2.0;
            waypoints_.push_back({x, y, z});

            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = z;
            pose.pose.orientation.w = 1.0; // No rotation
            planned_path_.poses.push_back(pose);
        }
        RCLCPP_INFO(this->get_logger(), "Generated %zu helix waypoints.", waypoints_.size());

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose",
            rclcpp::SensorDataQoS(),
            std::bind(&HelixTrajectoryTracker::pose_callback, this, std::placeholders::_1)
        );

        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

        planned_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/helix_planned_path", 1);

        actual_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/drone_actual_path", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 10 Hz
            std::bind(&HelixTrajectoryTracker::control_loop, this));
    }

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_pose_ = *msg;
        received_pose_ = true;

        // Add current pose to actual path
        geometry_msgs::msg::PoseStamped pose = *msg;
        pose.header.frame_id = "map"; // Make sure frame matches RViz fixed frame
        actual_path_.header = pose.header;
        actual_path_.poses.push_back(pose);

        // Optionally limit path length for performance
        if (actual_path_.poses.size() > 1000)
            actual_path_.poses.erase(actual_path_.poses.begin());
    }

    // Compute cross-track error: minimum distance from current position to any waypoint
    double compute_cross_track_error(double x, double y, double z) {
        double min_dist = std::numeric_limits<double>::max();
        for (const auto& wp : waypoints_) {
            double dx = wp[0] - x;
            double dy = wp[1] - y;
            double dz = wp[2] - z;
            double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
            if (dist < min_dist) {
                min_dist = dist;
            }
        }
        return min_dist;
    }

    void control_loop() {
        // Publish planned and actual path for RViz2
        planned_path_.header.stamp = this->now();
        planned_path_pub_->publish(planned_path_);
        actual_path_.header.stamp = this->now();
        actual_path_pub_->publish(actual_path_);

        if (!received_pose_ || current_wp_idx_ >= waypoints_.size())
            return;

        double x = current_pose_.pose.position.x;
        double y = current_pose_.pose.position.y;
        double z = current_pose_.pose.position.z;

        double x_tgt = waypoints_[current_wp_idx_][0];
        double y_tgt = waypoints_[current_wp_idx_][1];
        double z_tgt = waypoints_[current_wp_idx_][2];

        double ex = x_tgt - x;
        double ey = y_tgt - y;
        double ez = z_tgt - z;

        double dist = std::sqrt(ex*ex + ey*ey + ez*ez);

        // --- Cross-track error computation ---
        double cross_track_error = compute_cross_track_error(x, y, z);

        // If close to waypoint, go to next
        if (dist < 0.2) {
            RCLCPP_INFO(this->get_logger(), "Reached waypoint %d: (%.2f, %.2f, %.2f)", current_wp_idx_, x_tgt, y_tgt, z_tgt);
            current_wp_idx_++;
            if (current_wp_idx_ >= waypoints_.size()) {
                RCLCPP_INFO(this->get_logger(), "Helix trajectory complete.");
                geometry_msgs::msg::Twist vel_msg;
                vel_pub_->publish(vel_msg);
                return;
            }
            return;
        }

        // Simple proportional controller for velocity
        double Kp = 0.7;
        double vx = Kp * ex;
        double vy = Kp * ey;
        double vz = Kp * ez;

        // Limit maximum velocity
        double v_max = 1.0;
        double v_norm = std::sqrt(vx*vx + vy*vy + vz*vz);
        if (v_norm > v_max) {
            vx = vx * v_max / v_norm;
            vy = vy * v_max / v_norm;
            vz = vz * v_max / v_norm;
        }

        geometry_msgs::msg::Twist vel_msg;
        vel_msg.linear.x = vx;
        vel_msg.linear.y = vy;
        vel_msg.linear.z = vz;
        vel_pub_->publish(vel_msg);

        RCLCPP_INFO(this->get_logger(),
            "Tracking WP %d: cmd_vel = [%.2f, %.2f, %.2f], error = [%.2f, %.2f, %.2f], Cross-Track Error: %.3f",
            current_wp_idx_, vx, vy, vz, ex, ey, ez, cross_track_error);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planned_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr actual_path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::PoseStamped current_pose_;
    bool received_pose_;
    size_t current_wp_idx_;
    std::vector<std::vector<double>> waypoints_;
    nav_msgs::msg::Path planned_path_;
    nav_msgs::msg::Path actual_path_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HelixTrajectoryTracker>());
    rclcpp::shutdown();
    return 0;
}

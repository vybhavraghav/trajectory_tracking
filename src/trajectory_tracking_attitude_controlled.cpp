#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <vector>

class SpiralAttitudeController : public rclcpp::Node {
public:
    SpiralAttitudeController()
    : Node("spiral_attitude_controller")
    {
        // Declare parameters with default values
        this->declare_parameter("mass", 2.25);
        this->declare_parameter("hover_thrust", 0.38);
        this->declare_parameter("pid.pos.p", 4.0);  // Increased from 2.0
        this->declare_parameter("pid.pos.i", 0.1);  // Small integral gain for steady-state error
        this->declare_parameter("pid.pos.d", 3.0);  // Increased from 2.0
        this->declare_parameter("pid.max_integral", 1.0);
        this->declare_parameter("max_tilt_angle", 0.4);
        this->declare_parameter("trajectory.period", 120.0);  // Increased from 60.0 for slower motion
        
        // Get parameter values
        mass_ = this->get_parameter("mass").as_double();
        hover_thrust_ = this->get_parameter("hover_thrust").as_double();
        Kp_ = Eigen::Vector3d::Constant(this->get_parameter("pid.pos.p").as_double());
        Ki_ = Eigen::Vector3d::Constant(this->get_parameter("pid.pos.i").as_double());
        Kd_ = Eigen::Vector3d::Constant(this->get_parameter("pid.pos.d").as_double());
        max_integral_ = this->get_parameter("pid.max_integral").as_double();
        max_tilt_ = this->get_parameter("max_tilt_angle").as_double();
        
        g_ = 9.81;

        // Spiral parameters
        spiral_radius_ = 2.0;      // meters
        spiral_omega_ = 0.4;       // rad/waypoint
        spiral_z0_ = 5.0;          // center height (set to 5 meters)
        spiral_x0_ = 0.0;          // starting x
        spiral_pitch_ = 0.2;       // spiral pitch (distance between loops along x)
        num_waypoints_ = 100;       // number of waypoints in the spiral


        // Generate spiral waypoints (horizontal spiral along x, in the yz-plane)
        for (int i = 0; i < num_waypoints_; ++i) {
            double theta = i * spiral_omega_;
            double x = spiral_x0_ + spiral_pitch_ * theta;
            double y = spiral_radius_ * std::cos(theta);
            double z = spiral_z0_ + spiral_radius_ * std::sin(theta);
            spiral_waypoints_.emplace_back(x, y, z);
        }
        current_waypoint_idx_ = 0;
        target_ = spiral_waypoints_[0];

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose",
            rclcpp::SensorDataQoS(),
            std::bind(&SpiralAttitudeController::pose_callback, this, std::placeholders::_1)
        );
        vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/mavros/local_position/velocity_local",
            rclcpp::SensorDataQoS(),
            std::bind(&SpiralAttitudeController::vel_callback, this, std::placeholders::_1)
        );
        att_pub_ = this->create_publisher<mavros_msgs::msg::AttitudeTarget>(
            "/mavros/setpoint_raw/attitude", 10);
        actual_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/actual_path", 10);
        desired_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/desired_path", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/waypoint_marker", 1);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(25), // 40 Hz
            std::bind(&SpiralAttitudeController::control_loop, this));

        current_pose_received_ = false;
        current_vel_received_ = false;

        actual_path_.header.frame_id = "map";
        desired_path_.header.frame_id = "map";

        // Fill desired path for visualization
        for (const auto& wp : spiral_waypoints_) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = wp.x();
            pose.pose.position.y = wp.y();
            pose.pose.position.z = wp.z();
            pose.pose.orientation.w = 1.0;
            desired_path_.poses.push_back(pose);
        }

        // Initialize integral term
        integral_error_ = Eigen::Vector3d::Zero();
        last_time_ = this->now();

        // Add trajectory timing parameters
        // this->declare_parameter("trajectory.period", 60.0);  // Time to complete one revolution
        trajectory_period_ = this->get_parameter("trajectory.period").as_double();
        trajectory_start_time_ = this->now();
    }

private:
    void send_hover_command() {
        mavros_msgs::msg::AttitudeTarget att_msg;
        att_msg.header.stamp = this->now();
        att_msg.type_mask = 7;  // Use only attitude and thrust
        att_msg.orientation.w = 1.0;  // Identity quaternion
        att_msg.orientation.x = 0.0;
        att_msg.orientation.y = 0.0;
        att_msg.orientation.z = 0.0;
        att_msg.thrust = hover_thrust_;
        att_pub_->publish(att_msg);
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_pose_ = *msg;
        current_pose_received_ = true;
        last_pose_time_ = this->now();
    }
    void vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        current_vel_ = *msg;
        current_vel_received_ = true;
        last_vel_time_ = this->now();
    }

    double compute_cross_track_error(const Eigen::Vector3d& pos) {
        return (target_ - pos).norm();
    }

    void publish_waypoint_marker() {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "waypoint";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = target_.x();
        marker.pose.position.y = target_.y();
        marker.pose.position.z = target_.z();
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.8f;
        marker.lifetime = rclcpp::Duration(0, 0); // forever
        marker_pub_->publish(marker);
    }

    Eigen::Vector3d compute_desired_position(double t) {
        // Compute position along spiral based on time
        double theta = 2.0 * M_PI * t / trajectory_period_;
        double x = spiral_x0_ + spiral_pitch_ * theta;
        double y = spiral_radius_ * std::cos(theta);
        double z = spiral_z0_ + spiral_radius_ * std::sin(theta);
        return Eigen::Vector3d(x, y, z);
    }

    Eigen::Vector3d compute_desired_velocity(double t) {
        // Analytical derivative of position
        double theta = 2.0 * M_PI * t / trajectory_period_;
        double omega = 2.0 * M_PI / trajectory_period_;  // Angular velocity
        
        double dx = spiral_pitch_ * omega;  // Constant velocity along x
        double dy = -spiral_radius_ * omega * std::sin(theta);
        double dz = spiral_radius_ * omega * std::cos(theta);
        return Eigen::Vector3d(dx, dy, dz);
    }

    Eigen::Vector3d compute_desired_acceleration(double t) {
        // Analytical second derivative of position
        double theta = 2.0 * M_PI * t / trajectory_period_;
        double omega = 2.0 * M_PI / trajectory_period_;
        
        double ddx = 0.0;  // No acceleration along x
        double ddy = -spiral_radius_ * omega * omega * std::cos(theta);
        double ddz = -spiral_radius_ * omega * omega * std::sin(theta);
        return Eigen::Vector3d(ddx, ddy, ddz);
    }

    void control_loop() {
        // Check data freshness
        const double timeout = 0.5;  // seconds
        auto now = this->now();
        if (!current_pose_received_ || !current_vel_received_ ||
            (now - last_pose_time_).seconds() > timeout ||
            (now - last_vel_time_).seconds() > timeout) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Stale pose/velocity data - sending hover command");
            send_hover_command();
            return;
        }

        // Re-publish marker occasionally to keep it visible in RViz2
        static int marker_count = 0;
        if (marker_count++ % 20 == 0) { // every 0.5s at 40 Hz
            publish_waypoint_marker();
        }

        // Publish desired path for RViz2
        desired_path_.header.stamp = this->now();
        desired_path_pub_->publish(desired_path_);

        if (!current_pose_received_ || !current_vel_received_)
            return;

        // Compute time along trajectory
        double t = (this->now() - trajectory_start_time_).seconds();
        
        // Compute desired states
        Eigen::Vector3d pos_des = compute_desired_position(t);
        Eigen::Vector3d vel_des = compute_desired_velocity(t);
        Eigen::Vector3d acc_ff = compute_desired_acceleration(t);  // Feed-forward term

        // Current position and velocity
        Eigen::Vector3d pos(
            current_pose_.pose.position.x,
            current_pose_.pose.position.y,
            current_pose_.pose.position.z
        );
        Eigen::Vector3d vel(
            current_vel_.twist.linear.x,
            current_vel_.twist.linear.y,
            current_vel_.twist.linear.z
        );

        // Compute errors
        Eigen::Vector3d pos_error = pos_des - pos;
        Eigen::Vector3d vel_error = vel_des - vel;

        // Update integral (with anti-windup)
        double dt = (this->now() - last_time_).seconds();
        last_time_ = this->now();
        if (dt > 0.0 && dt < 1.0) {
            integral_error_ += pos_error * dt;
            for (int i = 0; i < 3; ++i) {
                integral_error_[i] = std::clamp(integral_error_[i], -max_integral_, max_integral_);
            }
        }

        // PID control with feed-forward acceleration
        Eigen::Vector3d acc_des = acc_ff  // Feed-forward term
                                + Kp_.cwiseProduct(pos_error)
                                + Ki_.cwiseProduct(integral_error_)
                                + Kd_.cwiseProduct(vel_error);

        // Apply velocity and acceleration limits
        const double max_velocity = 2.0;  // m/s
        const double max_acceleration = 2.0;  // m/s^2
        
        // Limit acceleration
        if (acc_des.norm() > max_acceleration) {
            acc_des = acc_des.normalized() * max_acceleration;
        }

        // Limit velocity
        Eigen::Vector3d vel_cmd = vel_des;
        if (vel_cmd.norm() > max_velocity) {
            vel_cmd = vel_cmd.normalized() * max_velocity;
        }
        
        // Add gravity compensation
        Eigen::Vector3d acc_total = acc_des + Eigen::Vector3d(0, 0, g_);

        // Desired thrust (projected onto body z)
        double thrust = hover_thrust_ * acc_total.norm() / g_; // Normalize so that g_ => 0.38
        thrust = std::clamp(thrust, 0.0, 1.0);

        // Desired orientation: align body z with acc_total
        Eigen::Vector3d zb_des = acc_total.normalized();

        // -------- Clamp maximum tilt angle --------
        double zb_xy = std::sqrt(zb_des.x() * zb_des.x() + zb_des.y() * zb_des.y());
        double max_xy = std::tan(max_tilt_);
        if (zb_xy > max_xy) {
            zb_des.x() *= max_xy / zb_xy;
            zb_des.y() *= max_xy / zb_xy;
            zb_des.z() = std::sqrt(1 - zb_des.x() * zb_des.x() - zb_des.y() * zb_des.y());
        }
        // ------------------------------------------

        Eigen::Vector3d xb_des(std::cos(0.0), std::sin(0.0), 0.0); // Yaw = 0
        Eigen::Vector3d yb_des = zb_des.cross(xb_des).normalized();
        xb_des = yb_des.cross(zb_des).normalized();

        Eigen::Matrix3d R_des;
        R_des.col(0) = xb_des;
        R_des.col(1) = yb_des;
        R_des.col(2) = zb_des;

        Eigen::Quaterniond q_des(R_des);

        // Publish attitude target with attitude and thrust (not body rates)
        mavros_msgs::msg::AttitudeTarget att_msg;
        att_msg.header.stamp = this->now();
        att_msg.type_mask = 7; // Use only attitude and thrust (bitmask = 7)
        att_msg.orientation.w = q_des.w();
        att_msg.orientation.x = q_des.x();
        att_msg.orientation.y = q_des.y();
        att_msg.orientation.z = q_des.z();
        att_msg.thrust = thrust;

        att_pub_->publish(att_msg);

        // Log cross-track error
        double cte = compute_cross_track_error(pos);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "pos[%.2f %.2f %.2f] target[%.2f %.2f %.2f] idx=%d CTE=%.3f thrust=%.2f",
            pos.x(), pos.y(), pos.z(), target_.x(), target_.y(), target_.z(),
            current_waypoint_idx_, cte, thrust);

        // Publish actual path for RViz2
        geometry_msgs::msg::PoseStamped actual_pose;
        actual_pose.header.frame_id = "map";
        actual_pose.pose = current_pose_.pose;
        actual_path_.poses.push_back(actual_pose);
        actual_path_.header.stamp = this->now();
        actual_path_pub_->publish(actual_path_);

        prev_target_ = target_;
    
    }

    // Members
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
    rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr att_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr actual_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr desired_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::TwistStamped current_vel_;
    bool current_pose_received_;
    bool current_vel_received_;

    // Trajectory and control
    std::vector<Eigen::Vector3d> spiral_waypoints_;
    Eigen::Vector3d target_, prev_target_;
    int current_waypoint_idx_;
    int num_waypoints_;
    double mass_, g_;
    double spiral_radius_, spiral_omega_, spiral_z0_, spiral_x0_, spiral_pitch_;
    nav_msgs::msg::Path actual_path_;
    nav_msgs::msg::Path desired_path_;

    // Integral control
    Eigen::Vector3d integral_error_;
    rclcpp::Time last_time_;

    // PID gains and control parameters
    Eigen::Vector3d Kp_;
    Eigen::Vector3d Ki_;
    Eigen::Vector3d Kd_;
    double max_integral_;
    double max_tilt_;
    double hover_thrust_;
    
    // Timing
    rclcpp::Time last_pose_time_;
    rclcpp::Time last_vel_time_;

    // Safety limits
    const double max_velocity_ = 1.0;  // m/s - reduced for better tracking
    const double max_acceleration_ = 1.0;  // m/s^2 - reduced for better tracking

    // Trajectory timing
    double trajectory_period_;
    rclcpp::Time trajectory_start_time_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpiralAttitudeController>());
    rclcpp::shutdown();
    return 0;
}

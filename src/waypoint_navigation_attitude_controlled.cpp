
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <limits>

class WaypointAttitudeController : public rclcpp::Node {
public:
    WaypointAttitudeController()
    : Node("waypoint_attitude_controller")
    {
        mass_ = 2.25;
        g_ = 9.81;
        target_ << 0.0, 2.0, 2.0;

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose",
            rclcpp::SensorDataQoS(),
            std::bind(&WaypointAttitudeController::pose_callback, this, std::placeholders::_1)
        );
        vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/mavros/local_position/velocity_local",
            rclcpp::SensorDataQoS(),
            std::bind(&WaypointAttitudeController::vel_callback, this, std::placeholders::_1)
        );
        att_pub_ = this->create_publisher<mavros_msgs::msg::AttitudeTarget>(
            "/mavros/setpoint_raw/attitude", 10);
        actual_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/actual_path", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/waypoint_marker", 1);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(25), // 40 Hz
            std::bind(&WaypointAttitudeController::control_loop, this));

        current_pose_received_ = false;
        current_vel_received_ = false;

        actual_path_.header.frame_id = "map";
        publish_waypoint_marker();
    }

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_pose_ = *msg;
        current_pose_received_ = true;
    }
    void vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        current_vel_ = *msg;
        current_vel_received_ = true;
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

    void control_loop() {
        // Re-publish marker occasionally to keep it visible in RViz2
        static int marker_count = 0;
        if (marker_count++ % 20 == 0) { // every 0.5s at 40 Hz
            publish_waypoint_marker();
        }

        if (!current_pose_received_ || !current_vel_received_)
            return;

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

        // Controller gains
        Eigen::Vector3d Kp(2.0, 2.0, 2.0);
        Eigen::Vector3d Kd(2.5, 2.5, 2.5);

        // Errors
        Eigen::Vector3d pos_error = target_ - pos;
        Eigen::Vector3d vel_error = -vel;

        // --- Deadband and velocity scaling ---
        double distance = pos_error.norm();
        // const double deadband = 0.10; // 10 cm
        Eigen::Vector3d acc_des = Eigen::Vector3d::Zero();
        // if (distance > deadband) {
        //     double scale = std::min(1.0, distance / 1.0); // scale down within 1m
        //     acc_des = scale * (Kp.cwiseProduct(pos_error) + Kd.cwiseProduct(vel_error));
        // }
        // -------------------------------------
        acc_des =  (Kp.cwiseProduct(pos_error) + Kd.cwiseProduct(vel_error));

        // Add gravity compensation
        Eigen::Vector3d acc_total = acc_des + Eigen::Vector3d(0, 0, g_);

        // Desired thrust (projected onto body z)
        const double hover_thrust = 0.38; // Your drone's hover thrust value
        double thrust = hover_thrust * acc_total.norm() / g_; // Normalize so that g_ => 0.38
        thrust = std::clamp(thrust, 0.0, 1.0);

        // Desired orientation: align body z with acc_total
        Eigen::Vector3d zb_des = acc_total.normalized();

        // -------- Clamp maximum tilt angle --------
        double max_tilt = 0.4; // radians (~34 degrees)
        double zb_xy = std::sqrt(zb_des.x() * zb_des.x() + zb_des.y() * zb_des.y());
        double max_xy = std::tan(max_tilt);
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
        // body rates are ignored

        att_pub_->publish(att_msg);

        // Log cross-track error
        double cte = compute_cross_track_error(pos);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "pos[%.2f %.2f %.2f] target[%.2f %.2f %.2f] CTE=%.3f thrust=%.2f",
            pos.x(), pos.y(), pos.z(), target_.x(), target_.y(), target_.z(), cte, thrust);

        // Publish actual path for RViz2
        geometry_msgs::msg::PoseStamped actual_pose;
        actual_pose.header.frame_id = "map";
        actual_pose.pose = current_pose_.pose;
        actual_path_.poses.push_back(actual_pose);
        actual_path_.header.stamp = this->now();
        actual_path_pub_->publish(actual_path_);
    }

    // Members
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
    rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr att_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr actual_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::TwistStamped current_vel_;
    bool current_pose_received_;
    bool current_vel_received_;

    Eigen::Vector3d target_;
    double mass_, g_;
    nav_msgs::msg::Path actual_path_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointAttitudeController>());
    rclcpp::shutdown();
    return 0;
}

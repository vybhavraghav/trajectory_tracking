#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cmath>

class SingleWaypointVelocityControl : public rclcpp::Node {
public:
    SingleWaypointVelocityControl()
    : Node("single_wp_velocity_control"), received_pose_(false)
    {
        // Set your single waypoint here
        target_x_ = 0.0;
        target_y_ = 0.0;
        target_z_ = 3.0;

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose",
            rclcpp::SensorDataQoS(),
            std::bind(&SingleWaypointVelocityControl::pose_callback, this, std::placeholders::_1)
        );

        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

        // RViz visualization publishers
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/drone_actual_path", 10);
        wp_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/target_waypoint_marker", 1);
        planned_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 1);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 10 Hz
            std::bind(&SingleWaypointVelocityControl::control_loop, this));
    }

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_pose_ = *msg;
        received_pose_ = true;

        // For path visualization
        geometry_msgs::msg::PoseStamped pose = *msg;
        pose.header.frame_id = "map";
        actual_path_.header = pose.header;
        actual_path_.poses.push_back(pose);
        if (actual_path_.poses.size() > 1000)
            actual_path_.poses.erase(actual_path_.poses.begin());
    }

    void publish_rviz_visuals() {
        // Publish actual path
        actual_path_.header.stamp = this->now();
        path_pub_->publish(actual_path_);

        // Publish target waypoint as a marker
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "waypoint";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = target_x_;
        marker.pose.position.y = target_y_;
        marker.pose.position.z = target_z_;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.r = 0.2;
        marker.color.g = 0.8;
        marker.color.b = 0.2;
        marker.color.a = 1.0;
        wp_marker_pub_->publish(marker);

        // Publish planned path (straight line to waypoint)
        nav_msgs::msg::Path planned_path;
        planned_path.header.frame_id = "map";
        planned_path.header.stamp = this->now();
        if (received_pose_ && !actual_path_.poses.empty()) {
            planned_path.poses.push_back(actual_path_.poses.front());
        }
        geometry_msgs::msg::PoseStamped wp_pose;
        wp_pose.header.frame_id = "map";
        wp_pose.pose.position.x = target_x_;
        wp_pose.pose.position.y = target_y_;
        wp_pose.pose.position.z = target_z_;
        wp_pose.pose.orientation.w = 1.0;
        planned_path.poses.push_back(wp_pose);
        planned_path_pub_->publish(planned_path);
    }

    void control_loop() {
        publish_rviz_visuals();

        if (!received_pose_)
            return;

        double x = current_pose_.pose.position.x;
        double y = current_pose_.pose.position.y;
        double z = current_pose_.pose.position.z;

        double ex = target_x_ - x;
        double ey = target_y_ - y;
        double ez = target_z_ - z;

        double dist = std::sqrt(ex*ex + ey*ey + ez*ez);

        // Stop if close enough
        if (dist < 0.2) {
            RCLCPP_INFO(this->get_logger(), "Reached target waypoint.");
            // Publish zero velocity to stop
            geometry_msgs::msg::Twist stop_msg;
            vel_pub_->publish(stop_msg);
            return;
        }

        // --- Velocity Control Logic ---
        double Kp = 0.7;
        double vx = Kp * ex;
        double vy = Kp * ey;
        double vz = Kp * ez;

        // Limit max velocity
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
        vel_msg.angular.x = 0.0;
        vel_msg.angular.y = 0.0;
        vel_msg.angular.z = 0.0; // No yaw rate control here

        vel_pub_->publish(vel_msg);

        RCLCPP_INFO(this->get_logger(),
            "cmd_vel: [%.2f, %.2f, %.2f], error: [%.2f, %.2f, %.2f], dist=%.2f",
            vx, vy, vz, ex, ey, ez, dist);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr wp_marker_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planned_path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::PoseStamped current_pose_;
    bool received_pose_;
    double target_x_, target_y_, target_z_;
    nav_msgs::msg::Path actual_path_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SingleWaypointVelocityControl>());
    rclcpp::shutdown();
    return 0;
}

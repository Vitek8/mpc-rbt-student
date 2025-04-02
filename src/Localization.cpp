#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "Localization.hpp"

LocalizationNode::LocalizationNode() : 
    rclcpp::Node("localization_node"),
    last_time_(this->get_clock()->now()) {

    // Odometry message initialization
    odometry_.header.frame_id = "map";
    odometry_.child_frame_id = "base_link";

    // Subscriber for joint_states
    joint_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10, std::bind(&LocalizationNode::jointCallback, this, std::placeholders::_1));

    // Publisher for odometry
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 10);

    // tf_briadcaster 
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    x = 0;
    y = 0;
    theta = 0;
    RCLCPP_INFO(get_logger(), "Localization node started.");
}

void LocalizationNode::jointCallback(const sensor_msgs::msg::JointState & msg) {
    auto current_time = this->get_clock()->now();
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    double left_wheel_vel = msg.velocity[0];
    double right_wheel_vel = msg.velocity[1];

    updateOdometry(left_wheel_vel, right_wheel_vel, dt);

    publishOdometry();

    publishTransform();
}

void LocalizationNode::updateOdometry(double left_wheel_vel, double right_wheel_vel, double dt) {
    double wheel_base = robot_config::HALF_DISTANCE_BETWEEN_WHEELS;

    double linear_velocity = (left_wheel_vel + right_wheel_vel) / 2.0;
    double angular_velocity = (right_wheel_vel - left_wheel_vel) / wheel_base;

    theta += angular_velocity * dt;
    theta = std::atan2(std::sin(theta), std::cos(theta));

    x += linear_velocity * std::cos(theta) * dt;
    y += linear_velocity * std::sin(theta) * dt;

    odometry_.header.stamp = this->get_clock()->now();
    odometry_.pose.pose.position.x = x;
    odometry_.pose.pose.position.y = y;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    odometry_.pose.pose.orientation = tf2::toMsg(q);

    odometry_.twist.twist.linear.x = linear_velocity;
    odometry_.twist.twist.linear.y = linear_velocity;
    odometry_.twist.twist.angular.x = angular_velocity;
}

void LocalizationNode::publishOdometry() {
    odometry_publisher_->publish(odometry_);
}

void LocalizationNode::publishTransform() {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = "base_link";

    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    t.transform.rotation = tf2::toMsg(q);

    // Broadcast the transform
    tf_broadcaster_->sendTransform(t);
}

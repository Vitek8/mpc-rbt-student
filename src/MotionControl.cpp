#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "MotionControl.hpp"

MotionControlNode::MotionControlNode() :
    rclcpp::Node("motion_control_node") {

        // Subscribers for odometry and laser scans
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry", 10, std::bind(&MotionControlNode::odomCallback, this, std::placeholders::_1));

        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/tiago_base/Hokuyo_URG_04LX_UG01", 10, std::bind(&MotionControlNode::lidarCallback, this, std::placeholders::_1));

        // Publisher for robot control
        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Client for path planning
        plan_client_ = this->create_client<nav_msgs::srv::GetPlan>("/plan_path");

        // Action server
        nav_server_ = rclcpp_action::create_server<nav2_msgs::action::NavigateToPose>(
        this,
        "/go_to_goal",
        std::bind(&MotionControlNode::navHandleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&MotionControlNode::navHandleCancel, this, std::placeholders::_1),
        std::bind(&MotionControlNode::navHandleAccepted, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Motion control node started.");

        // Connect to path planning service server
        while (!plan_client_->wait_for_service(std::chrono::seconds(1)))
        {
          RCLCPP_WARN(get_logger(), "Waiting for plan_path service...");
        }
        RCLCPP_INFO(get_logger(), "Path succesfully connected to service server...");
    }

void MotionControlNode::checkCollision() {
    auto thresh = 0.5;

    if (laser_scan_.ranges.empty()) {
      RCLCPP_INFO(get_logger(), "NOT ENOUGH RANGES TO CHECK COLLISIONS.");
      return;
    }

    //for(auto range : laser_scan_.ranges) {
  for (size_t i = laser_scan_.ranges.size() / 2 - 20; i <= laser_scan_.ranges.size() / 2 + 20; i++) {
      if (laser_scan_.ranges[i] < thresh) {
          geometry_msgs::msg::Twist stop;
          twist_publisher_->publish(stop);
		  goal_handle_->canceled(std::make_shared<nav2_msgs::action::NavigateToPose::Result>());
          RCLCPP_WARN(get_logger(), "Emergency stop! Obstacle at index %zu", i);
          break;
      }
    }
}

void MotionControlNode::updateTwist() {
    if (path_.poses.empty())
      return;

    const double v_max = 0.2;
    const double angular_limit = 0.5;
    const double lookahead_distance = 0.1;
    const double Kp_ang = 1.5; // Zesílení pro úhlové řízení

    // Vezmeme první bod na trase
    geometry_msgs::msg::PoseStamped target = path_.poses.front();

    // Rozdíl pozice
    double dx = target.pose.position.x - current_pose_.pose.position.x;
    double dy = target.pose.position.y - current_pose_.pose.position.y;

    // Vzdálenost k cíli
    double distance = std::hypot(dx, dy);
    if (distance < lookahead_distance) {
      path_.poses.erase(path_.poses.begin());
      return;
    }

    tf2::Quaternion q(
      current_pose_.pose.orientation.x,
      current_pose_.pose.orientation.y,
      current_pose_.pose.orientation.z,
      current_pose_.pose.orientation.w
    );
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // Cílový úhel (kam chci jet) - ve světových souřadnicích
    double target_angle = std::atan2(dy, dx);

    // Úhlová chyba
    double angle_error = target_angle - yaw;

    // Normalizace úhlu do intervalu [-pi, pi]
    angle_error = std::atan2(std::sin(angle_error), std::cos(angle_error));

    // Twist message
    geometry_msgs::msg::Twist twist;

    if (std::abs(angle_error) < M_PI / 4) {
      twist.linear.x = v_max;
    } else {
      twist.linear.x = 0.0; // když cíl není před námi, raději stojíme a otáčíme
    }

    // Úhlová rychlost (proporčně)
    twist.angular.z = std::clamp(Kp_ang * angle_error, -angular_limit, angular_limit);

    twist_publisher_->publish(twist);
  }



rclcpp_action::GoalResponse MotionControlNode::navHandleGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const nav2_msgs::action::NavigateToPose::Goal> goal) {
  (void)uuid;
  goal_pose_ = goal->pose;
  RCLCPP_INFO(get_logger(), "Goal accepted: x=%.2f y=%.2f",
              goal_pose_.pose.position.x, goal_pose_.pose.position.y);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MotionControlNode::navHandleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
  (void)goal_handle;
  RCLCPP_INFO(get_logger(), "Goal canceled.");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MotionControlNode::navHandleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
    goal_handle_ = goal_handle;
    auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
    request->start = current_pose_;
    request->goal = goal_pose_;
    request->tolerance = 0.05;

    auto future = plan_client_->async_send_request(
        request, std::bind(&MotionControlNode::pathCallback, this, std::placeholders::_1));
}

void MotionControlNode::execute() {
  rclcpp::Rate loop_rate(100);

  while (rclcpp::ok()) {
    if (goal_handle_->is_canceling()) {
      geometry_msgs::msg::Twist stop;
      twist_publisher_->publish(stop);
      goal_handle_->canceled(std::make_shared<nav2_msgs::action::NavigateToPose::Result>());
      RCLCPP_INFO(get_logger(), "Navigation canceled.");
      return;
    }

    checkCollision();
    updateTwist();

    auto feedback = std::make_shared<nav2_msgs::action::NavigateToPose::Feedback>();
    feedback->current_pose = current_pose_;
    goal_handle_->publish_feedback(feedback);

    double dx = goal_pose_.pose.position.x - current_pose_.pose.position.x;
    double dy = goal_pose_.pose.position.y - current_pose_.pose.position.y;
    if (std::hypot(dx, dy) < 0.1) {
      geometry_msgs::msg::Twist stop;
      twist_publisher_->publish(stop);
      goal_handle_->succeed(std::make_shared<nav2_msgs::action::NavigateToPose::Result>());
      RCLCPP_INFO(get_logger(), "Goal reached.");
      return;
    }
    loop_rate.sleep();
  }
}

void MotionControlNode::pathCallback(rclcpp::Client<nav_msgs::srv::GetPlan>::SharedFuture future) {
  auto response = future.get();
  if (response && !response->plan.poses.empty()) {
    path_ = response->plan;
    RCLCPP_INFO(get_logger(), "Path received with %zu poses", path_.poses.size());
    std::thread(&MotionControlNode::execute, this).detach();
  }
  else {
    RCLCPP_WARN(get_logger(), "Empty path received.");
    goal_handle_->abort(std::make_shared<nav2_msgs::action::NavigateToPose::Result>());
  }
}

void MotionControlNode::odomCallback(const nav_msgs::msg::Odometry & msg) {
    current_pose_.header = msg.header;
    current_pose_.pose = msg.pose.pose;
}

void MotionControlNode::lidarCallback(const sensor_msgs::msg::LaserScan & msg) {
  laser_scan_ = msg;
}

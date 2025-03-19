#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node

{
public:
    MinimalPublisher()
    : Node("battery")
    {

        this->declare_parameter("max_voltage", 42.0);
        this->declare_parameter("min_voltage", 36.0);

        param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

        auto cb = [this](const rclcpp::Parameter & p) {
            RCLCPP_INFO(
              this->get_logger(), "cb: Received an update to parameter \"%s\" of type %s: \"%ld\"",
              p.get_name().c_str(),
              p.get_type_name().c_str(),
              p.as_int());
        };
        cb_handle1_ = param_subscriber_->add_parameter_callback("max_voltage", cb);
        cb_handle2_ = param_subscriber_->add_parameter_callback("min_voltage", cb);

        publisher_ = this->create_publisher<std_msgs::msg::Float32>("battery_percentage", 10);

        // Correct the callback binding here
        subscription_ = this->create_subscription<std_msgs::msg::Float32>(
        "battery_voltage", 10, std::bind(&MinimalPublisher::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const std_msgs::msg::Float32::SharedPtr msg) const {
        RCLCPP_INFO(this->get_logger(), "Voltage: %f", msg->data);
        auto message = std_msgs::msg::Float32();

        auto max_voltage = this->get_parameter("max_voltage").as_double();
        auto min_voltage = this->get_parameter("min_voltage").as_double();

        auto percentage = (msg->data - min_voltage) / (max_voltage - min_voltage) * 100;
        RCLCPP_INFO(this->get_logger(), "Percentage: %f", percentage);
        message.data = percentage;
        publisher_->publish(message);
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;

    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle1_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle2_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}


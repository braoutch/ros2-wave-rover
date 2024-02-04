#include <ROS2Subscriber.hpp>
#include <QDebug>
#include <chrono>

using namespace std::chrono_literals;

ROS2Subscriber::ROS2Subscriber() : rclcpp::Node("WaveRobotController") {

    _liveness_publisher = this->create_publisher<builtin_interfaces::msg::Time>("/liveness", 10);
    _liveness_timer = this->create_wall_timer(
    500ms, std::bind(&ROS2Subscriber::LivenessCallback, this));
    qDebug() << "ROS2 Node initialized.";
}

ROS2Subscriber::~ROS2Subscriber(){

}

bool ROS2Subscriber::SubscribeToTopic(const QString& topic, std::function<void(const geometry_msgs::msg::Twist::SharedPtr)> callback) {
    qDebug() << "Topic subscription.";
    _twist_subscriptions = this->create_subscription<geometry_msgs::msg::Twist>(
                                                "/cmd_vel", rclcpp::SensorDataQoS().reliable(), [callback](const geometry_msgs::msg::Twist::SharedPtr msg){
        callback(msg);
    });

    return true;
}

void ROS2Subscriber::LivenessCallback()
{
  builtin_interfaces::msg::Time message;
  rclcpp::Time time = this->get_clock()->now();
  message.sec = time.seconds();
  message.sec = time.nanoseconds();
//      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  _liveness_publisher->publish(message);
}

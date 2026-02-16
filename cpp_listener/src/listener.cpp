#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class ChatterListener : public rclcpp::Node {
public:
  ChatterListener() : Node("cpp_listener") {
    using std::placeholders::_1;

    sub_ = this->create_subscription<std_msgs::msg::String>(
      "/talker",
      rclcpp::QoS(rclcpp::KeepLast(10)),
      std::bind(&ChatterListener::on_msg, this, _1)
    );

    RCLCPP_INFO(this->get_logger(), "Listening on /talker ...");
  }

private:
  void on_msg(const std_msgs::msg::String::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "Received: %s", msg->data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChatterListener>());
  rclcpp::shutdown();
  return 0;
}

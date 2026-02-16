#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class CppTalker : public rclcpp::Node {
public:
  CppTalker() : Node("cpp_talker"), count_(0) {
    pub_ = this->create_publisher<std_msgs::msg::String>(
      "/talker",
      rclcpp::QoS(rclcpp::KeepLast(10))
    );

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&CppTalker::on_timer, this)
    );

    RCLCPP_INFO(this->get_logger(), "Publishing on /talker ...");
  }

private:
  void on_timer() {
    auto msg = std_msgs::msg::String();
    msg.data = "Hello from cpp_talker #" + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: %s", msg.data.c_str());
    pub_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CppTalker>());
  rclcpp::shutdown();
  return 0;
}

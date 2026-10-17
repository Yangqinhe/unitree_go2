#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"

namespace
{
constexpr int32_t kSportApiHello = 1016;
}  // namespace

class Go2HelloNode : public rclcpp::Node
{
public:
  Go2HelloNode()
  : Node("go2_hello_node")
  {
    topic_ = this->declare_parameter<std::string>("topic", "/api/sport/request");
    response_topic_ = this->declare_parameter<std::string>("response_topic", "/api/sport/response");
    repeat_ = this->declare_parameter<bool>("repeat", false);
    period_ms_ = this->declare_parameter<int>("period_ms", 1000);
    publish_count_ = this->declare_parameter<int>("publish_count", 3);

    publisher_ = this->create_publisher<unitree_api::msg::Request>(topic_, 10);
    response_subscriber_ = this->create_subscription<unitree_api::msg::Response>(
      response_topic_,
      10,
      std::bind(&Go2HelloNode::onResponse, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms_),
      std::bind(&Go2HelloNode::publishHello, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Ready to publish Unitree hello command api_id=%d to %s",
      kSportApiHello,
      topic_.c_str());
  }

private:
  void publishHello()
  {
    if (publisher_->get_subscription_count() == 0) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "No subscriber on %s yet. Check Unitree DDS/network setup.",
        topic_.c_str());
      return;
    }

    unitree_api::msg::Request request;
    request.header.identity.api_id = kSportApiHello;

    publisher_->publish(request);
    sent_count_++;
    RCLCPP_INFO(
      this->get_logger(),
      "Published hello command %d/%d",
      sent_count_,
      publish_count_);

    if (!repeat_ && sent_count_ >= publish_count_) {
      timer_->cancel();
      shutdown_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        []() { rclcpp::shutdown(); });
    }
  }

  void onResponse(const unitree_api::msg::Response::SharedPtr response)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Sport response api_id=%d status=%d data=%s",
      response->header.identity.api_id,
      response->header.status.code,
      response->data.c_str());
  }

  std::string topic_;
  std::string response_topic_;
  bool repeat_;
  int period_ms_;
  int publish_count_;
  int sent_count_ = 0;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr publisher_;
  rclcpp::Subscription<unitree_api::msg::Response>::SharedPtr response_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr shutdown_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2HelloNode>());
  rclcpp::shutdown();
  return 0;
}

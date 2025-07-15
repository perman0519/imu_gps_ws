#include <memory> // For std::make_shared
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

// GPSTimestampRepublisher 클래스는 rclcpp::Node를 상속받습니다.
// 이는 ROS 2 노드의 기본 기능을 제공합니다.
class GPSTimestampRepublisher : public rclcpp::Node
{
public:
  // 생성자: 노드를 초기화하고, 서브스크라이버와 퍼블리셔를 설정합니다.
  GPSTimestampRepublisher() : Node("gps_repub") // 노드 이름을 "gps_repub"로 설정합니다.
  {
    // /fix 토픽을 구독하는 서브스크라이버를 생성합니다.
    // NavSatFix 메시지 타입을 사용하며, 콜백 함수는 this->callback을 지정합니다.
    // 큐 사이즈는 10으로 설정합니다.
    subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/fix",
      10,
      std::bind(&GPSTimestampRepublisher::callback, this, std::placeholders::_1));

    // /gps/fix 토픽으로 메시지를 발행하는 퍼블리셔를 생성합니다.
    // NavSatFix 메시지 타입을 사용하며, 큐 사이즈는 10으로 설정합니다.
    publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);

    // 노드가 시작되었음을 로깅합니다.
    RCLCPP_INFO(this->get_logger(), "GPS timestamp republisher started.");
  }

private:
  // GPS 메시지를 수신했을 때 호출되는 콜백 함수입니다.
  // msg는 수신된 NavSatFix 메시지에 대한 공유 포인터입니다.
  void callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    // 수신된 메시지의 헤더 타임스탬프를 현재 ROS 2 시간으로 업데이트합니다.
    // get_clock()->now()는 현재 ROS 시간을 rclcpp::Time 객체로 반환하고,
    // to_msg()는 이를 ROS 2 메시지(builtin_interfaces/msg/Time) 형식으로 변환합니다.
    msg->header.stamp = this->get_clock()->now();

    // 타임스탬프가 업데이트된 메시지를 발행합니다.
    // SharedPtr에서 실제 메시지 객체를 참조하기 위해 *msg를 사용합니다.
    publisher_->publish(*msg);
  }

  // 서브스크라이버와 퍼블리셔 객체를 저장할 멤버 변수입니다.
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
};

// 메인 함수: ROS 2 시스템을 초기화하고 노드를 실행합니다.
int main(int argc, char * argv[])
{
  // rclcpp 라이브러리를 초기화합니다.
  rclcpp::init(argc, argv);

  // GPSTimestampRepublisher 노드의 인스턴스를 생성하고 공유 포인터로 관리합니다.
  auto gps_republisher_node = std::make_shared<GPSTimestampRepublisher>();

  // 노드를 스핀(spin)하여 콜백 함수가 호출되도록 합니다.
  // 이 함수는 노드가 종료될 때까지 블로킹됩니다.
  rclcpp::spin(gps_republisher_node);

  // 노드가 종료되면 rclcpp 라이브러리를 종료합니다.
  rclcpp::shutdown();
  return 0;
}

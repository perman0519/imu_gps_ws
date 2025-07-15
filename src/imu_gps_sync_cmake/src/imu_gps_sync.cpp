#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class GPSIMUSyncNode : public rclcpp::Node
{
public:
    GPSIMUSyncNode() : Node("gps_imu_sync_node")
    {
        // 두 센서 토픽 구독자 생성
        gps_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::NavSatFix>>(
            this, "/gps/fix");
        imu_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Imu>>(
            this, "/imu/data");

        // 동기화 객체 생성
        sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(
            MySyncPolicy(30), *gps_sub_, *imu_sub_);

        // 동기화 콜백 등록
        sync_->registerCallback(std::bind(&GPSIMUSyncNode::sync_callback, this,
                                         std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "GPS-IMU 동기화 노드 실행 중...");
    }

private:
    // ApproximateTime 정책 정의 (0.1초 허용 범위)
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::NavSatFix, sensor_msgs::msg::Imu> MySyncPolicy;

    void sync_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& gps_msg,
                      const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg)
    {
        // 타임스탬프를 초 단위로 변환
        double gps_time = gps_msg->header.stamp.sec + gps_msg->header.stamp.nanosec * 1e-9;
        double imu_time = imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec * 1e-9;

        RCLCPP_INFO(this->get_logger(),
                   "SYNCED GPS time: %.3f | IMU time: %.3f", gps_time, imu_time);

        // 여기에 EKF 입력 처리 또는 퍼블리시 가능
        // process_synchronized_data(gps_msg, imu_msg);
    }

    // 멤버 변수들
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::NavSatFix>> gps_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Imu>> imu_sub_;
    std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<GPSIMUSyncNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}

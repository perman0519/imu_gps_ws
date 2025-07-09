#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix, Imu
from message_filters import Subscriber, ApproximateTimeSynchronizer

def sync_callback(gps_msg, imu_msg):
    gps_time = gps_msg.header.stamp.to_sec()
    imu_time = imu_msg.header.stamp.to_sec()
    rospy.loginfo("SYNCED GPS time: %.3f | IMU time: %.3f" % (gps_time, imu_time))
    # 여기에 EKF 입력 처리 or publish 가능

def main():
    rospy.init_node('gps_imu_sync_node')

    # 두 센서 토픽 구독
    gps_sub = Subscriber('/gps/fix', NavSatFix)
    imu_sub = Subscriber('/imu/data', Imu)

    # 동기화 객체 생성
    ats = ApproximateTimeSynchronizer(
        [gps_sub, imu_sub],    # 센서들
        queue_size=30,         # 버퍼 크기
        slop=0.1               # 시간 차 허용 범위 (0.1초 이내면 OK)
    )
    ats.registerCallback(sync_callback)

    rospy.loginfo("GPS-IMU 동기화 노드 실행 중...")
    rospy.spin()

if __name__ == '__main__':
    main()

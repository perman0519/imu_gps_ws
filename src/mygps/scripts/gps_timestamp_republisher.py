#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix

class GPSTimestampRepublisher:
    def __init__(self):
        rospy.init_node('gps_timestamp_republisher')

        # 원래 GPS 토픽 구독 (드라이버가 퍼블리시하는 토픽)
        self.sub = rospy.Subscriber('/smc_2000/fix', NavSatFix, self.callback, queue_size=10)

        # 보정 후 퍼블리시할 토픽
        self.pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size=10)

        rospy.loginfo("GPS timestamp republisher started.")

    def callback(self, msg):
        msg.header.stamp = rospy.Time.now()
        self.pub.publish(msg)

if __name__ == '__main__':
    GPSTimestampRepublisher()
    rospy.spin()

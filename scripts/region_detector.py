#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Int32

# 中心格點位置對應的區域編號
region_map = {
    (0.5,  0.5): 1,
    (0.5, -0.5): 2,
    (0.5, -1.5): 3,
    (-0.5, 0.5): 4,
    (-0.5, -0.5): 5,
    (-0.5, -1.5): 6,
    (-1.5, 0.5): 7,
    (-1.5, -0.5): 8,
    (-1.5, -1.5): 9
}

# 預先轉成 list 做最近距離查找
region_centers = list(region_map.keys())

pub = None

def amcl_callback(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    # 找最近的中心格點
    nearest = min(region_centers, key=lambda pt: (pt[0] - x)**2 + (pt[1] - y)**2)
    region_id = region_map[nearest]

    rospy.loginfo(f"[RegionDetector] 目前座標 ({x:.2f}, {y:.2f}) -> 最近中心點 {nearest} -> 區域 {region_id}")
    pub.publish(Int32(region_id))

if __name__ == '__main__':
    rospy.init_node('region_detector')
    pub = rospy.Publisher('/current_region', Int32, queue_size=10)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_callback)
    rospy.spin()

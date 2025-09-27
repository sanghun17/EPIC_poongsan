#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import tf2_ros
from geometry_msgs.msg import TransformStamped

latest_odom = None
latest_cloud = None

def odom_callback(msg):
    global latest_odom
    latest_odom = msg

def cloud_callback(msg):
    global latest_cloud
    latest_cloud = msg

def main():
    rospy.init_node('odometry_repeater_synced')

    rospy.Subscriber("/lio_sam/mapping/odometry", Odometry, odom_callback)
    rospy.Subscriber("/lio_sam/deskew/cloud_deskewed", PointCloud2, cloud_callback)
    
    pub_odom = rospy.Publisher("/quad_0/lidar_slam/odom2", Odometry, queue_size=10)
    pub_cloud = rospy.Publisher("/quad_0/lidar_slam/cloud", PointCloud2, queue_size=10)
    
    # TF broadcaster 생성
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    rate = rospy.Rate(200)  # 200 Hz

    while not rospy.is_shutdown():
        if latest_odom and latest_cloud:
            # 현재 ROS time 사용
            current_time = rospy.Time.now()
            
            # 현재 시간을 오도메트리에 적용
            synced_odom = latest_odom
            synced_odom.header.stamp = current_time
            synced_odom.header.frame_id = "world"  # 또는 필요한 frame_id
            
            # 현재 시간을 포인트클라우드에 적용
            synced_cloud = latest_cloud
            synced_cloud.header.stamp = current_time
            
            # 동기화된 메시지들 발행
            pub_odom.publish(synced_odom)
            pub_cloud.publish(synced_cloud)
            
            # sensor -> base_link tf 발행
            transform = TransformStamped()
            transform.header.stamp = current_time
            transform.header.frame_id = "sensor"
            transform.child_frame_id = "base_link"
            
            # 오도메트리의 position과 orientation 사용
            transform.transform.translation.x = latest_odom.pose.pose.position.x
            transform.transform.translation.y = latest_odom.pose.pose.position.y
            transform.transform.translation.z = latest_odom.pose.pose.position.z
            
            transform.transform.rotation.x = latest_odom.pose.pose.orientation.x
            transform.transform.rotation.y = latest_odom.pose.pose.orientation.y
            transform.transform.rotation.z = latest_odom.pose.pose.orientation.z
            transform.transform.rotation.w = latest_odom.pose.pose.orientation.w
            
            tf_broadcaster.sendTransform(transform)
            
        rate.sleep()

if __name__ == "__main__":
    main() 
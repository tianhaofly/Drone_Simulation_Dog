#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from apriltag_ros.msg import AprilTagDetectionArray


class AprilTagRelativePoseNode(object):
    def __init__(self):
        rospy.init_node('px4_apriltag_relpose', anonymous=False)

        # 目标 tag 的 ID（与打印出来、贴在狗背上的 AprilTag ID 一致）
        self.tag_id = rospy.get_param('~tag_id', 0)

        # 输出的话题：狗平台在无人机机体坐标系（base_link）下的位姿
        self.pub = rospy.Publisher('dog_platform_in_uav', PoseStamped, queue_size=10)

        # 订阅 apriltag_ros 的检测结果
        self.sub = rospy.Subscriber('tag_detections', AprilTagDetectionArray, self.detections_cb)

        rospy.loginfo('px4_apriltag_relpose node started. Waiting for tag_detections...')

    def detections_cb(self, msg):
        if not msg.detections:
            return

        for det in msg.detections:
            # 每个 detection 里 id 是一个数组（支持同一检测多个 tag），这里只关心包含目标 ID 的
            if self.tag_id in det.id:
                pose_stamped = PoseStamped()
                # apriltag_ros 的 pose.header.frame_id 通常就是相机坐标系
                # 在你的仿真里，相机帧就是 base_link，所以这里直接用即可
                pose_stamped.header = det.pose.header
                pose_stamped.pose = det.pose.pose.pose

                # 这里假设 tag 中心就是“狗背平台”的原点
                # 如果将来平台坐标系相对 tag 有偏移，可以在这里做一次坐标变换

                self.pub.publish(pose_stamped)
                break


if __name__ == '__main__':
    try:
        node = AprilTagRelativePoseNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

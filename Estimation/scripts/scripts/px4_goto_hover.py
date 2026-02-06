#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


class Px4GotoHover(object):
    def __init__(self):
        rospy.init_node('px4_goto_hover', anonymous=False)

        # 目标位置 (x, y, z)
        self.target_x = 1.0
        self.target_y = 1.0
        self.target_z = 2.0

        self.state = State()
        self.current_pose = None

        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_cb)
        self.local_pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_pos_cb)
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

        rospy.loginfo('Waiting for MAVROS services...')
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/set_mode')
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        self.rate = rospy.Rate(20)  # 20 Hz

    def state_cb(self, msg):
        self.state = msg

    def local_pos_cb(self, msg):
        self.current_pose = msg

    def wait_for_connection(self):
        rospy.loginfo('Waiting for FCU connection...')
        while not rospy.is_shutdown() and not self.state.connected:
            self.rate.sleep()
        rospy.loginfo('FCU connected.')

    def wait_for_pose(self):
        rospy.loginfo('Waiting for first local position...')
        while not rospy.is_shutdown() and self.current_pose is None:
            self.rate.sleep()
        rospy.loginfo('Got first local position.')

    def send_initial_setpoints(self, count=100):
        # 先在当前位置发送一段时间的 setpoint，满足 PX4 对 OFFBOARD 的要求
        pose = PoseStamped()
        pose.pose.position = self.current_pose.pose.position
        pose.pose.orientation = self.current_pose.pose.orientation

        rospy.loginfo('Sending initial position setpoints at current location...')
        for _ in range(count):
            if rospy.is_shutdown():
                return
            pose.header.stamp = rospy.Time.now()
            self.setpoint_pub.publish(pose)
            self.rate.sleep()

    def set_offboard_and_arm(self):
        rospy.loginfo('Switching to OFFBOARD and arming...')
        last_request = rospy.Time.now()

        while not rospy.is_shutdown() and (not self.state.armed or self.state.mode != 'OFFBOARD'):
            now = rospy.Time.now()

            # 持续发送当前位置 setpoint，防止 offboard 丢失
            pose = PoseStamped()
            pose.pose.position = self.current_pose.pose.position
            pose.pose.orientation = self.current_pose.pose.orientation
            pose.header.stamp = now
            self.setpoint_pub.publish(pose)

            if self.state.mode != 'OFFBOARD' and (now - last_request) > rospy.Duration(1.0):
                try:
                    resp = self.set_mode_client(base_mode=0, custom_mode='OFFBOARD')
                    if resp.mode_sent:
                        rospy.loginfo('OFFBOARD enabled')
                except rospy.ServiceException as e:
                    rospy.logwarn('Failed to set OFFBOARD: %s', e)
                last_request = now

            elif not self.state.armed and (now - last_request) > rospy.Duration(1.0):
                try:
                    resp = self.arming_client(True)
                    if resp.success:
                        rospy.loginfo('Vehicle armed')
                except rospy.ServiceException as e:
                    rospy.logwarn('Failed to arm: %s', e)
                last_request = now

            self.rate.sleep()

        rospy.loginfo('OFFBOARD active and vehicle armed.')

    def goto_and_hover(self):
        # 目标点：保持当前姿态，只改变位置到 (0, 0, 8)
        target_pose = PoseStamped()
        target_pose.pose.position.x = self.target_x
        target_pose.pose.position.y = self.target_y
        target_pose.pose.position.z = self.target_z

        # 保持当前姿态（yaw 等不变）
        target_pose.pose.orientation = self.current_pose.pose.orientation

        rospy.loginfo('Going to target position (%.2f, %.2f, %.2f) and hovering...',
                      self.target_x, self.target_y, self.target_z)

        reached = False
        stable_count = 0
        stable_required = 50  # 50 * (1/20s) = 2.5s 稳定在目标附近
        pos_tolerance = 0.1  # 10 cm 位置容差

        while not rospy.is_shutdown():
            if self.current_pose is None:
                self.rate.sleep()
                continue

            target_pose.header.stamp = rospy.Time.now()
            self.setpoint_pub.publish(target_pose)

            # 计算与目标点距离
            dx = self.current_pose.pose.position.x - self.target_x
            dy = self.current_pose.pose.position.y - self.target_y
            dz = self.current_pose.pose.position.z - self.target_z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)

            if dist < pos_tolerance:
                stable_count += 1
            else:
                stable_count = 0

            if not reached and stable_count >= stable_required:
                reached = True
                rospy.loginfo('Target position reached and hovering (distance %.3f m).', dist)

            self.rate.sleep()

    def run(self):
        self.wait_for_connection()
        self.wait_for_pose()
        self.send_initial_setpoints()
        self.set_offboard_and_arm()
        self.goto_and_hover()


if __name__ == '__main__':
    try:
        node = Px4GotoHover()
        node.run()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool


class Px4TakeoffHover(object):
    """Simple takeoff script: OFFBOARD + arm + climb to target altitude and hover.

    Usage assumption:
      - Gazebo world已启动，并已spawn iris；
      - PX4 SITL和MAVROS均已启动；
      - 飞机初始在地面附近静止；
      - 只需运行本节点即可从当前位置垂直起飞到目标高度悬停。
    """

    def __init__(self):
        rospy.init_node("px4_takeoff_hover", anonymous=True)

        # Parameters
        self.target_alt = rospy.get_param("~target_alt", 8.0)  # 目标高度（相对于local_z原点）
        self.rate_hz = rospy.get_param("~rate", 20.0)
        self.initial_setpoints = rospy.get_param("~initial_setpoints", 100)

        # State
        self.state = State()
        self.pose = None

        # Subscribers
        self.state_sub = rospy.Subscriber("/mavros/state", State, self.state_cb, queue_size=10)
        self.pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_cb, queue_size=10)

        # Publisher
        self.sp_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)

        # Services (lazy init)
        self.set_mode_srv = None
        self.arming_srv = None

    def state_cb(self, msg):
        self.state = msg

    def pose_cb(self, msg):
        self.pose = msg

    def wait_for_connection_and_pose(self):
        rospy.loginfo("[px4_takeoff_hover] Waiting for MAVROS connection...")
        rate = rospy.Rate(5.0)
        while not rospy.is_shutdown() and not self.state.connected:
            rate.sleep()
        rospy.loginfo("[px4_takeoff_hover] MAVROS connected.")

        rospy.loginfo("[px4_takeoff_hover] Waiting for local_position/pose...")
        while not rospy.is_shutdown() and self.pose is None:
            rospy.sleep(0.05)
        rospy.loginfo("[px4_takeoff_hover] Local position received.")

    def init_services(self):
        if self.set_mode_srv is None:
            rospy.loginfo("[px4_takeoff_hover] Waiting for /mavros/set_mode service...")
            rospy.wait_for_service("/mavros/set_mode")
            self.set_mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)

        if self.arming_srv is None:
            rospy.loginfo("[px4_takeoff_hover] Waiting for /mavros/cmd/arming service...")
            rospy.wait_for_service("/mavros/cmd/arming")
            self.arming_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)

    def publish_setpoint(self, x, y, z):
        sp = PoseStamped()
        sp.header.stamp = rospy.Time.now()
        sp.header.frame_id = "map"  # MAVROS会处理到local frame
        sp.pose.position.x = x
        sp.pose.position.y = y
        sp.pose.position.z = z
        sp.pose.orientation.w = 1.0
        self.sp_pub.publish(sp)

    def run(self):
        self.wait_for_connection_and_pose()
        self.init_services()

        rate = rospy.Rate(self.rate_hz)

        # 以当前xy位置作为起飞点，目标高度为target_alt
        x0 = self.pose.pose.position.x
        y0 = self.pose.pose.position.y
        z0 = self.pose.pose.position.z

        rospy.loginfo("[px4_takeoff_hover] Current position: x=%.2f y=%.2f z=%.2f", x0, y0, z0)
        rospy.loginfo("[px4_takeoff_hover] Target altitude: %.2f m", self.target_alt)

        # 1) 先在当前位置发送一段稳定setpoint（即使此时还不是OFFBOARD，主要是保证有合理位置期望）
        rospy.loginfo("[px4_takeoff_hover] Sending initial setpoints before arming...")
        for _ in range(self.initial_setpoints):
            if rospy.is_shutdown():
                return
            self.publish_setpoint(x0, y0, z0)
            rate.sleep()

        # 2) 先解锁（在当前默认模式下，例如 POSCTL/ALTCTL），行为更接近你用 QGC 的流程
        if not self.state.armed:
            try:
                resp = self.arming_srv(value=True)
                if resp.success:
                    rospy.loginfo("[px4_takeoff_hover] Vehicle arming requested.")
                else:
                    rospy.logwarn("[px4_takeoff_hover] Arming request was rejected by FCU.")
            except rospy.ServiceException as e:
                rospy.logwarn("[px4_takeoff_hover] Failed to call arming: %s", str(e))

        # 解锁后再稍微维持当前setpoint一小段时间
        for _ in range(int(self.rate_hz * 1.0)):
            if rospy.is_shutdown():
                return
            self.publish_setpoint(x0, y0, z0)
            rate.sleep()

        # 3) 切换到OFFBOARD模式（此时已经armed，且一直有setpoint）
        if self.state.mode != "OFFBOARD":
            try:
                resp = self.set_mode_srv(custom_mode="OFFBOARD")
                if resp.mode_sent:
                    rospy.loginfo("[px4_takeoff_hover] OFFBOARD mode requested.")
                else:
                    rospy.logwarn("[px4_takeoff_hover] OFFBOARD mode request was rejected by FCU.")
            except rospy.ServiceException as e:
                rospy.logwarn("[px4_takeoff_hover] Failed to call set_mode: %s", str(e))

        # 模式切换过程继续维持当前位置setpoint，保证 offboard 有足够频率的期望值
        for _ in range(int(self.rate_hz * 1.0)):
            if rospy.is_shutdown():
                return
            self.publish_setpoint(x0, y0, z0)
            rate.sleep()

        # 4) 开始上升到目标高度（简单阶跃到目标高度，位置控制会平滑跟踪）
        rospy.loginfo("[px4_takeoff_hover] Climbing to target altitude...")
        climb_time = rospy.get_param("~climb_time", 10.0)  # 仅用于日志，可视需要调整
        start_time = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            elapsed = now - start_time

            # 发送目标高度的固定setpoint
            self.publish_setpoint(x0, y0, self.target_alt)

            # 简单的到达判定：高度误差小于0.3m 且耗时超过2秒
            current_z = self.pose.pose.position.z if self.pose is not None else 0.0
            if elapsed > 2.0 and abs(current_z - self.target_alt) < 0.3:
                rospy.loginfo("[px4_takeoff_hover] Reached target altitude and hovering.")
                break

            rate.sleep()

        # 悬停阶段：再保持一小段时间，然后退出节点，把控制权交给其他 OFFBOARD 脚本
        rospy.loginfo("[px4_takeoff_hover] Holding position at target altitude for 2 seconds, then exiting...")
        hold_duration = rospy.get_param("~hold_duration", 2.0)
        hold_start = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            if now - hold_start > hold_duration:
                rospy.loginfo("[px4_takeoff_hover] Takeoff sequence finished.")
                break

            self.publish_setpoint(x0, y0, self.target_alt)
            rate.sleep()


def main():
    node = Px4TakeoffHover()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()

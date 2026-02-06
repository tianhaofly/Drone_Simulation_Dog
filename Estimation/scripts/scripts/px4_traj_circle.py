#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool


class OffboardCircleController(object):
    """Simple PX4 offboard position controller that flies a horizontal circle.

    - Uses local ENU frame from /mavros/local_position/pose.
    - Circle is centered at the current position when the node starts.
    - Can be started at any time after takeoff.
    """

    def __init__(self):
        rospy.init_node("px4_traj_circle", anonymous=True)

        # Parameters
        self.radius = rospy.get_param("~radius", 5.0)          # [m]
        self.angular_speed = rospy.get_param("~omega", 0.2)    # [rad/s]
        self.altitude_offset = rospy.get_param("~dz", 0.0)     # [m], relative to current z
        self.enable_offboard_switch = rospy.get_param("~enable_offboard_switch", True)
        self.enable_arming = rospy.get_param("~enable_arming", False)

        # State
        self.current_state = State()
        self.current_pose = None
        self.center_x = None
        self.center_y = None
        self.center_z = None
        self.start_time = None

        # Subscribers
        self.state_sub = rospy.Subscriber("/mavros/state", State, self.state_cb, queue_size=10)
        self.pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_cb, queue_size=10)

        # Publisher
        self.setpoint_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)

        # Service proxies (created lazily when needed)
        self.set_mode_srv = None
        self.arming_srv = None

    def state_cb(self, msg):
        self.current_state = msg

    def pose_cb(self, msg):
        self.current_pose = msg

    def wait_for_connection(self):
        rospy.loginfo("[px4_traj_circle] Waiting for MAVROS connection...")
        rate = rospy.Rate(5.0)
        while not rospy.is_shutdown() and not self.current_state.connected:
            rate.sleep()
        rospy.loginfo("[px4_traj_circle] MAVROS connected.")

    def wait_for_pose(self):
        rospy.loginfo("[px4_traj_circle] Waiting for local position...")
        while not rospy.is_shutdown() and self.current_pose is None:
            rospy.sleep(0.05)
        rospy.loginfo("[px4_traj_circle] Local position received.")

    def init_services(self):
        if self.set_mode_srv is None:
            rospy.loginfo("[px4_traj_circle] Waiting for /mavros/set_mode service...")
            rospy.wait_for_service("/mavros/set_mode")
            self.set_mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)

        if self.arming_srv is None:
            rospy.loginfo("[px4_traj_circle] Waiting for /mavros/cmd/arming service...")
            rospy.wait_for_service("/mavros/cmd/arming")
            self.arming_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)

    def send_initial_setpoints(self, count=100, rate_hz=20.0):
        """Send a stream of initial setpoints to satisfy PX4 offboard requirements."""
        rate = rospy.Rate(rate_hz)

        pose = PoseStamped()
        pose.header.frame_id = "map"  # local ENU; actual frame is set by MAVROS
        pose.pose.position.x = self.center_x
        pose.pose.position.y = self.center_y
        pose.pose.position.z = self.center_z

        for _ in range(count):
            if rospy.is_shutdown():
                break
            pose.header.stamp = rospy.Time.now()
            self.setpoint_pub.publish(pose)
            rate.sleep()

    def try_enable_offboard_and_arm(self):
        if not self.enable_offboard_switch and not self.enable_arming:
            return

        self.init_services()

        # Try to switch to OFFBOARD
        if self.enable_offboard_switch and self.current_state.mode != "OFFBOARD":
            try:
                resp = self.set_mode_srv(custom_mode="OFFBOARD")
                if resp.mode_sent:
                    rospy.loginfo("[px4_traj_circle] OFFBOARD mode requested.")
                else:
                    rospy.logwarn("[px4_traj_circle] OFFBOARD mode request was rejected by FCU.")
            except rospy.ServiceException as e:
                rospy.logwarn("[px4_traj_circle] Failed to call set_mode: %s", str(e))

        # Optionally arm (only if not already armed)
        if self.enable_arming and not self.current_state.armed:
            try:
                resp = self.arming_srv(value=True)
                if resp.success:
                    rospy.loginfo("[px4_traj_circle] Vehicle arming requested.")
                else:
                    rospy.logwarn("[px4_traj_circle] Arming request was rejected by FCU.")
            except rospy.ServiceException as e:
                rospy.logwarn("[px4_traj_circle] Failed to call arming: %s", str(e))

    def run(self):
        self.wait_for_connection()
        self.wait_for_pose()

        # Use current pose as circle center
        self.center_x = self.current_pose.pose.position.x
        self.center_y = self.current_pose.pose.position.y
        self.center_z = self.current_pose.pose.position.z + self.altitude_offset

        rospy.loginfo("[px4_traj_circle] Starting circle trajectory: R=%.2f m, omega=%.2f rad/s, center=(%.2f, %.2f, %.2f)",
                      self.radius, self.angular_speed, self.center_x, self.center_y, self.center_z)

        # PX4 requires a stream of setpoints before switching to OFFBOARD
        self.send_initial_setpoints()
        self.try_enable_offboard_and_arm()

        self.start_time = rospy.Time.now().to_sec()

        rate_hz = 20.0
        rate = rospy.Rate(rate_hz)

        while not rospy.is_shutdown():
            t = rospy.Time.now().to_sec() - self.start_time
            angle = self.angular_speed * t

            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"  # MAVROS will transform to the appropriate local frame

            pose.pose.position.x = self.center_x + self.radius * math.cos(angle)
            pose.pose.position.y = self.center_y + self.radius * math.sin(angle)
            pose.pose.position.z = self.center_z

            # Keep orientation fixed (level)
            pose.pose.orientation.w = 1.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0

            self.setpoint_pub.publish(pose)
            rate.sleep()


def main():
    controller = OffboardCircleController()
    try:
        controller.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()

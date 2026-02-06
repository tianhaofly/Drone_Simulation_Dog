import numpy as np
from scipy.spatial.transform import Rotation as R

import rospy
from geometry_msgs.msg import PoseStamped
from apriltag_ros.msg import AprilTagDetectionArray


def compute_transform_A_to_B(C_to_D_position, C_to_D_quaternion):
    """计算A坐标系相对于B坐标系的位姿

    参数:
        C_to_D_position: C相对D的位置 (x, y, z)
        C_to_D_quaternion: C相对D的四元数 (x, y, z, w)

    返回:
        A到B的位置和四元数
    """
    # 1. 定义基本变换
    # A相对于C：绕Z轴旋转90度，原点相同
    R_A_to_C = R.from_euler('z', 90, degrees=True)

    # B相对于D：绕Z轴转270度，再绕X轴转180度，原点相同
    R_B_to_D = R.from_euler('z', 270, degrees=True) * R.from_euler('x', 180, degrees=True)

    # C相对于D：输入参数
    R_C_to_D = R.from_quat(C_to_D_quaternion)

    # 2. 计算旋转部分的变换
    # A相对于D的旋转：R_A_to_D = R_C_to_D * R_A_to_C
    R_A_to_D = R_C_to_D * R_A_to_C

    # B相对于D的旋转：已知为R_B_to_D
    # 我们需要的是A相对于B的旋转：R_A_to_B = R_B_to_D⁻¹ * R_A_to_D
    # 等价于 R_A_to_B = R_D_to_B * R_A_to_D
    R_D_to_B = R_B_to_D.inv()
    R_A_to_B = R_D_to_B * R_A_to_D

    # 3. 计算位置部分的变换
    # 位置向量
    pos_C_to_D = np.array(C_to_D_position)

    # 由于A和C原点相同，所以A相对于D的位置就是C相对于D的位置
    pos_A_to_D = pos_C_to_D

    # 由于B和D原点相同，所以D相对于B的位置是零向量
    # B相对于D的位置也是零向量

    # A相对于B的位置：pos_A_to_B = R_D_to_B * (pos_A_to_D - pos_B_to_D)
    # 由于pos_B_to_D = 0，所以：
    pos_A_to_B = R_D_to_B.apply(pos_A_to_D)

    return pos_A_to_B, R_A_to_B.as_quat()


publisher = None


def tag_detections_callback(msg):
    global publisher

    if publisher is None:
        return

    if not msg.detections:
        return

    detection = msg.detections[0]

    position = detection.pose.pose.pose.position
    orientation = detection.pose.pose.pose.orientation

    C_to_D_position = [position.x, position.y, position.z]
    C_to_D_quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]

    A_to_B_position, A_to_B_quaternion = compute_transform_A_to_B(
        C_to_D_position, C_to_D_quaternion
    )

    pose_msg = PoseStamped()
    pose_msg.header.stamp = msg.header.stamp
    pose_msg.header.frame_id = "B"  # A在B坐标系下的位姿

    pose_msg.pose.position.x = float(A_to_B_position[0])
    pose_msg.pose.position.y = float(A_to_B_position[1])
    pose_msg.pose.position.z = float(A_to_B_position[2])

    pose_msg.pose.orientation.x = float(A_to_B_quaternion[0])
    pose_msg.pose.orientation.y = float(A_to_B_quaternion[1])
    pose_msg.pose.orientation.z = float(A_to_B_quaternion[2])
    pose_msg.pose.orientation.w = float(A_to_B_quaternion[3])

    publisher.publish(pose_msg)


def main():
    global publisher

    rospy.init_node("coordinate_tran_node", anonymous=False)

    # 发布A相对于B的位姿（位置+四元数）
    publisher = rospy.Publisher("/A_to_B_pose", PoseStamped, queue_size=1)

    # 订阅/tag_detections，获取C相对D的位姿
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, tag_detections_callback, queue_size=1)

    rospy.loginfo("coordinate_tran_node started: subscribing /tag_detections, publishing /A_to_B_pose")
    rospy.spin()


if __name__ == "__main__":
    main()

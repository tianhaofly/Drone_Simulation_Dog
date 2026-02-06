#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler, quaternion_multiply, quaternion_matrix
import numpy as np
from scipy.spatial.transform import Rotation as R

try:
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtCore, QtWidgets
except ImportError:
    pg = None


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

    # A相对于B的位置：pos_A_to_B = R_D_to_B.apply(pos_A_to_D)
    pos_A_to_B = R_D_to_B.apply(pos_A_to_D)

    return pos_A_to_B, R_A_to_B.as_quat()


class GazeboPoseTransformNode(object):
    def _relative_pose(self, p_from, q_from, p_to, q_to):
        """
        计算 to 在 from 坐标系下的位置和姿态。
        p_from, p_to: np.array([x, y, z])
        q_from, q_to: np.array([x, y, z, w])
        返回: (rel_xyz, rel_quat, rel_euler)
        """
        # 旋转矩阵
        R_from = quaternion_matrix(q_from)[0:3, 0:3]
        R_from_inv = R_from.T
        dp = p_to - p_from
        rel_xyz = R_from_inv.dot(dp)
        # 相对四元数: q_rel = q_from* 的逆 * q_to
        from tf.transformations import quaternion_inverse, euler_from_quaternion
        q_rel = quaternion_multiply(quaternion_inverse(q_from), q_to)
        rel_euler = euler_from_quaternion(q_rel)
        return rel_xyz, q_rel, rel_euler

    def __init__(self):
        # Parameters
        self.world_frame = rospy.get_param('~world_frame', 'world')
        self.dog_model_name = rospy.get_param('~dog_model_name', 'go2w_gazebo')
        self.uav_model_name = rospy.get_param('~uav_model_name', 'iris')

        # Offsets from model base to target frames (in model base frame), only XYZ translation.
        # GO2W: qr_platform_joint origin="0 0 0.10" relative to trunk (trunk与base重合)
        self.platform_offset_xyz = rospy.get_param('~platform_offset_xyz', [0.0, 0.0, 0.10])

        # Iris downward camera pose relative to base_link from iris.sdf:
        # <pose>0 0 -0.05 0 1.5708 0</pose>
        # 按你的要求，这里只用 Z 方向的平移偏置，不使用相机自身的旋转偏置。
        self.camera_offset_xyz = rospy.get_param('~camera_offset_xyz', [0.0, 0.0, -0.05])

        # 是否开启示波器式实时波形显示（0: 关闭, 1: 开启）
        self.enable_plot = bool(rospy.get_param('~enable_plot', 0))

        # 数据缓存（Gazebo 平台在相机坐标系下的 7 维 + 视觉识别得到的 7 维）
        # 顺序: [x, y, z, qx, qy, qz, qw]
        self.max_points = 500
        self.truth_t = []
        self.truth_vals = [[] for _ in range(7)]
        self.vision_t = []
        self.vision_vals = [[] for _ in range(7)]
        self.t0 = None  # 统一时间基准

        # 如果需要画图且 pyqtgraph 可用，则初始化绘图界面
        if self.enable_plot and pg is not None:
            self._init_plot()
        elif self.enable_plot and pg is None:
            rospy.logwarn("enable_plot=1 但未安装 pyqtgraph，无法显示波形窗口")

        # Publishers
        self.pub_platform_pose = rospy.Publisher('dog_platform_pose', PoseStamped, queue_size=10)
        self.pub_camera_pose = rospy.Publisher('uav_downward_camera_pose', PoseStamped, queue_size=10)
        # A相对于B的位姿（由 /aruco/pose 计算得到）
        self.pub_A_to_B_pose = rospy.Publisher('/A_to_B_pose', PoseStamped, queue_size=1)

        # Subscribers
        self.sub_model_states = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_cb, queue_size=1)
        # 订阅 ArUco 识别结果 /aruco/pose（PoseStamped），获取 C 相对 D 的位姿
        self.sub_tag_detections = rospy.Subscriber('/aruco/pose', PoseStamped, self.tag_detections_cb, queue_size=1)

    def _init_plot(self):
        """使用 pyqtgraph 初始化 7×2 示波器式波形界面，固定y轴范围"""
        self.app = QtWidgets.QApplication.instance()
        if self.app is None:
            self.app = QtWidgets.QApplication([])

        labels = ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
        ylims = [(-6, 6), (-6, 6), (-6, 0), (-1, 1), (-1, 1), (-1, 1), (-1, 1)]

        self.win = pg.GraphicsLayoutWidget(title="Gazebo vs Vision Pose")
        self.win.resize(900, 800)

        self.plots = []
        self.truth_curves = []
        self.vision_curves = []
        self.info_labels = []

        for i, name in enumerate(labels):
            p = self.win.addPlot(row=i, col=0)
            p.showGrid(x=True, y=True, alpha=0.3)
            p.setLabel('left', name)
            if i == len(labels) - 1:
                p.setLabel('bottom', 'time', units='s')
            p.setYRange(ylims[i][0], ylims[i][1])

            truth_curve = p.plot(pen=pg.mkPen('b', width=1), name='gazebo')
            vision_curve = p.plot(pen=pg.mkPen('r', width=1, style=QtCore.Qt.DashLine), name='vision')

            # 左下角文本，锚点左下，白色
            info = pg.TextItem(anchor=(0, 1), color=(255, 255, 255))
            # 初始位置设为(0, y_min)，后续每次刷新动态调整
            info.setPos(0, ylims[i][0])
            p.addItem(info)

            self.plots.append(p)
            self.truth_curves.append(truth_curve)
            self.vision_curves.append(vision_curve)
            self.info_labels.append(info)

        self.win.show()

        # 使用 Qt 自身的 QTimer 周期性刷新曲线（高刷新率），避免跨线程操作Qt
        self.plot_timer = QtCore.QTimer()
        self.plot_timer.timeout.connect(self._update_plot)
        self.plot_timer.start(20)  # 20 ms ~= 50 Hz

    @staticmethod
    def _pose_to_np(position, orientation):
        p = np.array([position.x, position.y, position.z])
        q = np.array([orientation.x, orientation.y, orientation.z, orientation.w])
        return p, q

    @staticmethod
    def _apply_transform_translate_only(p_parent, q_parent, offset_xyz):
        """Apply only XYZ translation offset in parent frame, keep orientation same as parent."""
        R_parent = quaternion_matrix(q_parent)[0:3, 0:3]
        offset = np.array(offset_xyz)
        p_child = p_parent + R_parent.dot(offset)
        q_child = q_parent
        return p_child, q_child

    def _record_truth(self, rel_xyz, rel_quat):
        """记录 Gazebo 确定的参考相对位姿，统一时间基准，并限流采样"""
        if not self.enable_plot or pg is None:
            return

        t = rospy.get_time()
        if self.t0 is None:
            self.t0 = t
        t_rel = t - self.t0

        # 只每隔0.05s采样一次
        min_interval = 0.05
        if hasattr(self, '_last_truth_t'):
            if t_rel - self._last_truth_t < min_interval:
                return
        self._last_truth_t = t_rel

        vals = [
            float(rel_xyz[0]),
            float(rel_xyz[1]),
            float(rel_xyz[2]),
            float(rel_quat[0]),
            float(rel_quat[1]),
            float(rel_quat[2]),
            float(rel_quat[3]),
        ]
        self.truth_t.append(t_rel)
        for i in range(7):
            self.truth_vals[i].append(vals[i])

        # 限制缓存长度
        if len(self.truth_t) > self.max_points:
            self.truth_t = self.truth_t[-self.max_points:]
            for i in range(7):
                self.truth_vals[i] = self.truth_vals[i][-self.max_points:]

    def _record_vision(self, pos, quat):
        """记录视觉识别得到的相对位姿，统一时间基准"""
        if not self.enable_plot or pg is None:
            return

        t = rospy.get_time()
        if self.t0 is None:
            self.t0 = t
        t_rel = t - self.t0
        vals = [
            float(pos[0]),
            float(pos[1]),
            float(pos[2]),
            float(quat[0]),
            float(quat[1]),
            float(quat[2]),
            float(quat[3]),
        ]
        self.vision_t.append(t_rel)
        for i in range(7):
            self.vision_vals[i].append(vals[i])

        # 限制缓存长度
        if len(self.vision_t) > self.max_points:
            self.vision_t = self.vision_t[-self.max_points:]
            for i in range(7):
                self.vision_vals[i] = self.vision_vals[i][-self.max_points:]

    def _update_plot(self):
        """定时刷新 7×2 波形显示，自动对齐X轴范围，Y轴固定（pyqtgraph）"""
        if not self.enable_plot or pg is None:
            return

        if not self.truth_t and not self.vision_t:
            return

        # 固定X轴宽度为20s，超过20s后窗口滚动
        t_window = 20.0
        t_max = 0
        if self.truth_t:
            t_max = max(t_max, self.truth_t[-1])
        if self.vision_t:
            t_max = max(t_max, self.vision_t[-1])
        t_max = max(t_max, t_window)
        t_min = max(0, t_max - t_window)

        for i, p in enumerate(self.plots):
            if self.truth_t:
                self.truth_curves[i].setData(self.truth_t, self.truth_vals[i])
            if self.vision_t:
                self.vision_curves[i].setData(self.vision_t, self.vision_vals[i])
            p.setXRange(t_min, t_max, padding=0.0)

            # 左下角显示最新值和差值，位置随x/y轴动态调整
            v_val = self.vision_vals[i][-1] if self.vision_vals[i] else None
            t_val = self.truth_vals[i][-1] if self.truth_vals[i] else None
            y_min = p.viewRange()[1][0] if p else -6
            if v_val is not None and t_val is not None:
                diff = v_val - t_val
                info_str = f"v:{v_val:.3f} g:{t_val:.3f} d:{diff:.3f}"
            elif v_val is not None:
                info_str = f"v:{v_val:.3f}"
            elif t_val is not None:
                info_str = f"g:{t_val:.3f}"
            else:
                info_str = ''
            self.info_labels[i].setText(info_str)
            # 左下角，x=当前t_min，y=当前y_min
            self.info_labels[i].setPos(t_min, y_min)

        # 刷新由 Qt 事件循环统一处理，这里不再手动调用 processEvents

    def tag_detections_cb(self, msg):
        """订阅 /aruco/pose（PoseStamped），计算并发布 A 相对于 B 的位姿"""

        # /aruco/pose: geometry_msgs/PoseStamped
        position = msg.pose.position
        orientation = msg.pose.orientation

        C_to_D_position = [position.x, position.y, position.z]
        C_to_D_quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]

        A_to_B_position, A_to_B_quaternion = compute_transform_A_to_B(
            C_to_D_position, C_to_D_quaternion
        )

        pose_msg = PoseStamped()
        pose_msg.header.stamp = msg.header.stamp
        pose_msg.header.frame_id = 'B'  # A 在 B 坐标系下的位姿

        pose_msg.pose.position.x = float(A_to_B_position[0])
        pose_msg.pose.position.y = float(A_to_B_position[1])
        pose_msg.pose.position.z = float(A_to_B_position[2])

        pose_msg.pose.orientation.x = float(A_to_B_quaternion[0])
        pose_msg.pose.orientation.y = float(A_to_B_quaternion[1])
        pose_msg.pose.orientation.z = float(A_to_B_quaternion[2])
        pose_msg.pose.orientation.w = float(A_to_B_quaternion[3])

        self.pub_A_to_B_pose.publish(pose_msg)
        # 记录视觉识别的相对位姿用于波形显示
        self._record_vision(A_to_B_position, A_to_B_quaternion)

    def model_states_cb(self, msg):
        try:
            # Find indices for dog and UAV models
            dog_idx = msg.name.index(self.dog_model_name)
            uav_idx = msg.name.index(self.uav_model_name)
        except ValueError:
            # One of the models not found in this message
            return

        # ----- Dog platform pose -----
        dog_pose = msg.pose[dog_idx]
        p_dog, q_dog = self._pose_to_np(dog_pose.position, dog_pose.orientation)

        p_platform, q_platform = self._apply_transform_translate_only(
            p_dog,
            q_dog,
            self.platform_offset_xyz,
        )

        platform_msg = PoseStamped()
        platform_msg.header.stamp = rospy.Time.now()
        platform_msg.header.frame_id = self.world_frame
        platform_msg.pose.position.x = float(p_platform[0])
        platform_msg.pose.position.y = float(p_platform[1])
        platform_msg.pose.position.z = float(p_platform[2])
        platform_msg.pose.orientation.x = float(q_platform[0])
        platform_msg.pose.orientation.y = float(q_platform[1])
        platform_msg.pose.orientation.z = float(q_platform[2])
        platform_msg.pose.orientation.w = float(q_platform[3])
        self.pub_platform_pose.publish(platform_msg)

        # ----- UAV downward camera pose -----
        uav_pose = msg.pose[uav_idx]
        p_uav, q_uav = self._pose_to_np(uav_pose.position, uav_pose.orientation)

        p_cam, q_cam = self._apply_transform_translate_only(
            p_uav,
            q_uav,
            self.camera_offset_xyz,
        )

        cam_msg = PoseStamped()
        cam_msg.header.stamp = rospy.Time.now()
        cam_msg.header.frame_id = self.world_frame
        cam_msg.pose.position.x = float(p_cam[0])
        cam_msg.pose.position.y = float(p_cam[1])
        cam_msg.pose.position.z = float(p_cam[2])
        cam_msg.pose.orientation.x = float(q_cam[0])
        cam_msg.pose.orientation.y = float(q_cam[1])
        cam_msg.pose.orientation.z = float(q_cam[2])
        cam_msg.pose.orientation.w = float(q_cam[3])
        self.pub_camera_pose.publish(cam_msg)

        # ----- Platform in camera frame -----
        rel_xyz, rel_quat, rel_euler = self._relative_pose(p_cam, q_cam, p_platform, q_platform)
        # 打印/发布
        rospy.loginfo_throttle(1.0, "Platform in camera frame: xyz = [%.3f, %.3f, %.3f], euler = [%.3f, %.3f, %.3f], quat = [%.4f, %.4f, %.4f, %.4f]" % (
            rel_xyz[0], rel_xyz[1], rel_xyz[2],
            rel_euler[0], rel_euler[1], rel_euler[2],
            rel_quat[0], rel_quat[1], rel_quat[2], rel_quat[3]
        ))

        # 可选：发布为PoseStamped
        rel_msg = PoseStamped()
        rel_msg.header.stamp = rospy.Time.now()
        rel_msg.header.frame_id = 'uav_downward_camera_pose'  # 以相机为参考系
        rel_msg.pose.position.x = float(rel_xyz[0])
        rel_msg.pose.position.y = float(rel_xyz[1])
        rel_msg.pose.position.z = float(rel_xyz[2])
        rel_msg.pose.orientation.x = float(rel_quat[0])
        rel_msg.pose.orientation.y = float(rel_quat[1])
        rel_msg.pose.orientation.z = float(rel_quat[2])
        rel_msg.pose.orientation.w = float(rel_quat[3])
        if not hasattr(self, 'pub_platform_in_camera'):
            self.pub_platform_in_camera = rospy.Publisher('platform_in_camera', PoseStamped, queue_size=10)
        self.pub_platform_in_camera.publish(rel_msg)

        # 记录 Gazebo 参考相对位姿用于波形显示
        self._record_truth(rel_xyz, rel_quat)

def main():
    rospy.init_node('gazebo_pose_transform_node')
    node = GazeboPoseTransformNode()
    rospy.loginfo('gazebo_pose_transform_node started')
    # 如果启用了绘图且 pyqtgraph 可用，则进入 Qt 事件循环；
    # 否则使用 rospy.spin() 即可。
    if node.enable_plot and pg is not None and hasattr(node, 'app') and node.app is not None:
        node.app.exec_()
    else:
        rospy.spin()


if __name__ == '__main__':
    main()

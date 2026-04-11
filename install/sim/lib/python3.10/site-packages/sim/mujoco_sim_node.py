#!/usr/bin/env python3
"""
mujoco_sim_node.py - MuJoCo 仿真器 ROS 2 桥接节点

功能：
- 加载 MuJoCo MJCF 模型进行仿真
- 订阅关节力矩指令 (/joint_torques_cmd)
- 发布关节状态 (/joint_states)
- 发布 IMU 数据 (/imu/data)
- 发布 TF 变换 (/tf)
- 支持可视化窗口和仿真控制

作者: UIKA Team
"""

import os
import threading
import time
from typing import List, Optional

import mujoco
import mujoco.viewer
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import TransformStamped, Quaternion
from sensor_msgs.msg import Imu, JointState
from sensor_msgs.msg import MagneticField
from std_msgs.msg import Float64MultiArray, Float32
from std_srvs.srv import Empty, SetBool
from tf2_ros import TransformBroadcaster


class MujocoSimNode(Node):
    """
    MuJoCo 仿真器 ROS 2 桥接节点

    关节顺序（12 个驱动关节）:
    0: FL_hip_joint    1: FL_thigh_joint    2: FL_calf_joint
    3: FR_hip_joint    4: FR_thigh_joint    5: FR_calf_joint
    6: BL_hip_joint    7: BL_thigh_joint    8: BL_calf_joint
    9: BR_hip_joint   10: BR_thigh_joint   11: BR_calf_joint

    qpos 布局（浮动基座）:
    [0:3]   - 基座位置 (x, y, z)
    [3:7]   - 基座四元数 (w, x, y, z)
    [7:19]  - 12 个关节角度

    qvel 布局:
    [0:3]   - 基座线速度
    [3:6]   - 基座角速度
    [6:18]  - 12 个关节速度
    """

    def __init__(self, name: str = 'mujoco_sim_node'):
        """初始化 MuJoCo 仿真节点"""
        super().__init__(name)

        # ============================================================
        # 参数声明
        # ============================================================
        self.declare_parameter('model_path', 'mjcf/uika.xml')
        self.declare_parameter('hz', 1000.0)
        self.declare_parameter('use_viewer', True)
        self.declare_parameter('real_time_factor', 1.0)
        self.declare_parameter('add_sensor_noise', False)
        self.declare_parameter('position_noise_std', 0.001)
        self.declare_parameter('velocity_noise_std', 0.01)
        self.declare_parameter('imu_acc_noise_std', 0.1)
        self.declare_parameter('imu_gyro_noise_std', 0.01)

        # 获取参数
        self.model_path = self.get_parameter('model_path').value
        self.hz = self.get_parameter('hz').value
        self.use_viewer = self.get_parameter('use_viewer').value
        self.real_time_factor = self.get_parameter('real_time_factor').value
        self.add_noise = self.get_parameter('add_sensor_noise').value
        self.pos_noise_std = self.get_parameter('position_noise_std').value
        self.vel_noise_std = self.get_parameter('velocity_noise_std').value
        self.acc_noise_std = self.get_parameter('imu_acc_noise_std').value
        self.gyro_noise_std = self.get_parameter('imu_gyro_noise_std').value

        self.get_logger().info(f'MuJoCo 仿真节点初始化中...')
        self.get_logger().info(f'模型路径: {self.model_path}')
        self.get_logger().info(f'仿真频率: {self.hz} Hz')
        self.get_logger().info(f'使用可视化窗口: {self.use_viewer}')

        # ============================================================
        # MuJoCo 模型加载
        # ============================================================
        self.model = None
        self.data = None
        self.viewer = None

        self._load_model()

        # ============================================================
        # 关节配置
        # ============================================================
        # 关节名称列表（与 URDF 顺序一致）
        self.joint_names = [
            'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
            'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
            'BL_hip_joint', 'BL_thigh_joint', 'BL_calf_joint',
            'BR_hip_joint', 'BR_thigh_joint', 'BR_calf_joint'
        ]
        self.num_joints = 12

        # 关节力矩指令（由订阅者更新）
        self.torque_cmd = np.zeros(self.num_joints)

        # ============================================================
        # ROS 2 发布者
        # ============================================================
        # 关节状态发布者
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # IMU 数据发布者
        self.imu_pub = self.create_publisher(
            Imu,
            '/imu/data',
            10
        )

        # TF 变换发布者
        self.tf_broadcaster = TransformBroadcaster(self)

        # 仿真时间发布者（调试用）
        self.sim_time_pub = self.create_publisher(
            Float32,
            '/sim/simulation_time',
            10
        )

        # 接触力发布者（调试用）
        self.contact_pub = self.create_publisher(
            Float64MultiArray,
            '/sim/contact_forces',
            10
        )

        # ============================================================
        # ROS 2 订阅者
        # ============================================================
        self.create_subscription(
            Float64MultiArray,
            '/joint_torques_cmd',
            self._torque_cmd_callback,
            10
        )

        # ============================================================
        # ROS 2 服务
        # ============================================================
        self.create_service(Empty, '/reset', self._reset_callback)
        self.create_service(SetBool, '/pause', self._pause_callback)

        # ============================================================
        # 仿真状态
        # ============================================================
        self.is_paused = False
        self.simulation_time = 0.0
        self.step_count = 0

        # ============================================================
        # 线程同步
        # ============================================================
        self.lock = threading.Lock()
        self.running = True

        # ============================================================
        # 定时器（用于发布消息）
        # ============================================================
        # 以 1000Hz 发布关节状态
        pub_period = 1.0 / self.hz
        self.create_timer(pub_period, self._publish_callback)

        self.get_logger().info('MuJoCo 仿真节点初始化完成!')

    def _load_model(self):
        """加载 MuJoCo 模型"""
        try:
            # 获取工作空间根目录
            # __file__ = .../install/sim/lib/python3.10/site-packages/sim/mujoco_sim_node.py
            # site-packages = .../install/sim/lib/python3.10/site-packages
            # lib = .../install/sim/lib
            # sim = .../install/sim
            # install = .../install
            # workspace = ... (parent of install)
            module_dir = os.path.dirname(os.path.abspath(__file__))
            site_packages_dir = os.path.dirname(module_dir)  # .../site-packages
            lib_dir = os.path.dirname(site_packages_dir)    # .../lib
            sim_install_dir = os.path.dirname(lib_dir)      # .../sim
            install_dir = os.path.dirname(sim_install_dir)  # .../install
            workspace_root = os.path.dirname(install_dir)    # parent of install

            # 构建可能的路径列表
            possible_paths = [
                # 从 description 包加载 (推荐)
                os.path.join(workspace_root, 'description', 'share', 'description', 'UIKA', 'mjcf', self.model_path),
                os.path.join(workspace_root, 'description', 'share', 'description', 'UIKA', 'mjcf', 'uika.xml'),
                # 从当前工作目录加载 (备用)
                os.path.join(os.getcwd(), 'install', 'description', 'share', 'description', 'UIKA', 'mjcf', self.model_path),
                # 直接路径
                self.model_path,
            ]

            model_file = None
            for path in possible_paths:
                if os.path.exists(path):
                    model_file = path
                    self.get_logger().info(f'找到模型文件: {path}')
                    break

            if model_file is None:
                raise FileNotFoundError(f'模型文件不存在: {self.model_path}')

            # 加载模型
            self.model = mujoco.MjModel.from_xml_path(model_file)
            self.data = mujoco.MjData(self.model)

            self.get_logger().info(f'模型加载成功!')
            self.get_logger().info(f'关节数量: {self.model.njnt}')
            self.get_logger().info(f'驱动关节数量: {self.model.nu}')

        except Exception as e:
            self.get_logger().error(f'模型加载失败: {e}')
            raise

    def _torque_cmd_callback(self, msg: Float64MultiArray):
        """关节力矩指令回调"""
        with self.lock:
            if len(msg.data) == self.num_joints:
                self.torque_cmd[:] = msg.data
            else:
                self.get_logger().warn(
                    f'力矩指令维度不匹配: 期望 {self.num_joints}, 收到 {len(msg.data)}'
                )

    def _reset_callback(self, request: Empty, response):
        """重置仿真回调"""
        with self.lock:
            # 重置 MuJoCo 数据
            mujoco.mj_resetData(self.model, self.data)
            self.simulation_time = 0.0
            self.step_count = 0
            self.torque_cmd[:] = 0.0
            self.get_logger().info('仿真已重置')

        return response

    def _pause_callback(self, request: SetBool.Request, response: SetBool.Response):
        """暂停/继续仿真回调"""
        with self.lock:
            self.is_paused = request.data
            if self.is_paused:
                self.get_logger().info('仿真已暂停')
            else:
                self.get_logger().info('仿真已继续')
            response.success = True
            response.message = 'paused' if self.is_paused else 'running'
        return response

    def _publish_callback(self):
        """定时发布回调（1000Hz）"""
        if self.is_paused:
            return

        with self.lock:
            # 应用控制指令到仿真器
            # self.data.ctrl[:12] 包含 12 个关节的控制输入
            self.data.ctrl[:self.num_joints] = self.torque_cmd

            # 执行一步仿真
            # dt = 1.0 / self.hz (默认 0.001s)
            mujoco.mj_step(self.model, self.data)

            # 更新仿真时间
            self.simulation_time += self.model.opt.timestep
            self.step_count += 1

            # 发布消息
            self._publish_joint_states()
            self._publish_imu()
            self._publish_tf()
            self._publish_debug()

    def _publish_joint_states(self):
        """发布关节状态"""
        msg = JointState()

        # 设置时间戳
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'

        # 关节名称
        msg.name = self.joint_names

        # 位置 (qpos[7:19])
        # 注意：MuJoCo 的关节位置在 qpos 中的索引需要映射
        joint_pos = np.zeros(self.num_joints)
        joint_vel = np.zeros(self.num_joints)

        for i, name in enumerate(self.joint_names):
            # 查找关节 ID
            joint_id = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_JOINT,
                name
            )
            if joint_id >= 0:
                joint_pos[i] = self.data.qpos[joint_id + 3]  # +3 跳过基座位置
                joint_vel[i] = self.data.qvel[joint_id + 3]  # +3 跳过基座速度

        # 添加噪声（如果启用）
        if self.add_noise:
            joint_pos += np.random.normal(0, self.pos_noise_std, self.num_joints)
            joint_vel += np.random.normal(0, self.vel_noise_std, self.num_joints)

        msg.position = joint_pos.tolist()
        msg.velocity = joint_vel.tolist()

        # 力矩（从仿真数据获取）
        # 注意：这里使用 qfrc_applied 作为施加的力矩
        msg.effort = self.torque_cmd.tolist()

        self.joint_state_pub.publish(msg)

    def _publish_imu(self):
        """发布 IMU 数据"""
        msg = Imu()

        # 时间戳
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_imu'

        # 基座四元数 (qpos[3:7])
        q = self.data.qpos[3:7]
        msg.orientation.x = q[1]
        msg.orientation.y = q[2]
        msg.orientation.z = q[3]
        msg.orientation.w = q[0]

        # 协方差（未知，设为 -1 表示不使用）
        msg.orientation_covariance = [-1.0] * 9

        # 基座角速度 (qvel[3:6]) - 转换为体坐标系
        # MuJoCo 的 qvel[3:6] 已经是体坐标系下的角速度
        gyro = self.data.qvel[3:6]

        # 添加噪声
        if self.add_noise:
            gyro += np.random.normal(0, self.gyro_noise_std, 3)

        msg.angular_velocity.x = gyro[0]
        msg.angular_velocity.y = gyro[1]
        msg.angular_velocity.z = gyro[2]
        msg.angular_velocity_covariance = [-1.0] * 9

        # 线加速度 (从 qacc 计算 + 重力补偿)
        # MuJoCo 中 qacc[0:3] 是线加速度
        # 需要转换到体坐标系并加上重力
        base_acc = self.data.qacc[0:3]

        # 重力在体坐标系中的分量
        # 重力在世界坐标系中是 [0, 0, -9.81]
        # 需要旋转到体坐标系
        quat = self.data.qpos[3:7]
        R = self._quaternion_to_rotation_matrix(quat)
        gravity_body = R.T @ np.array([0, 0, -9.81])

        # 加上重力补偿
        acc = base_acc - gravity_body

        # 添加噪声
        if self.add_noise:
            acc += np.random.normal(0, self.acc_noise_std, 3)

        msg.linear_acceleration.x = acc[0]
        msg.linear_acceleration.y = acc[1]
        msg.linear_acceleration.z = acc[2]
        msg.linear_acceleration_covariance = [-1.0] * 9

        self.imu_pub.publish(msg)

    def _publish_tf(self):
        """发布 TF 变换 (world -> base)"""
        t = TransformStamped()

        # 时间戳
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base'

        # 基座位置 (qpos[0:3])
        t.transform.translation.x = self.data.qpos[0]
        t.transform.translation.y = self.data.qpos[1]
        t.transform.translation.z = self.data.qpos[2]

        # 基座四元数 (qpos[3:7])
        q = self.data.qpos[3:7]
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]
        t.transform.rotation.w = q[0]

        self.tf_broadcaster.sendTransform(t)

    def _publish_debug(self):
        """发布调试信息（低频率）"""
        # 每 100 步发布一次（约 10Hz）
        if self.step_count % 100 == 0:
            # 仿真时间
            sim_time_msg = Float32()
            sim_time_msg.data = float(self.simulation_time)
            self.sim_time_pub.publish(sim_time_msg)

    def _quaternion_to_rotation_matrix(self, q: np.ndarray) -> np.ndarray:
        """四元数转旋转矩阵

        四元数格式: [w, x, y, z]
        """
        w, x, y, z = q[0], q[1], q[2], q[3]

        R = np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
            [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
            [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]
        ])

        return R

    def start_viewer(self):
        """启动 MuJoCo 可视化窗口（可选）"""
        if self.use_viewer and self.data is not None:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self.get_logger().info('MuJoCo 可视化窗口已启动')
            return True
        return False

    def run(self):
        """运行仿真主循环"""
        self.get_logger().info('开始 MuJoCo 仿真...')

        # 启动可视化窗口
        if self.use_viewer:
            # 在单独线程中运行 viewer
            viewer_thread = threading.Thread(target=self.start_viewer)
            viewer_thread.daemon = True
            viewer_thread.start()

        # 主循环（通过定时器发布 ROS 消息）
        # rclpy.spin(self) 会在此处阻塞
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            self.get_logger().info('收到中断信号，正在关闭...')
        finally:
            self.shutdown()

    def shutdown(self):
        """关闭节点"""
        self.running = False
        if self.viewer is not None:
            self.viewer.close()
        self.destroy_node()


def main(args=None):
    """主入口函数"""
    rclpy.init(args=args)

    node = MujocoSimNode()
    node.run()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

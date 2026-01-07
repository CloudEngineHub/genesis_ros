import genesis as gs
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion
from .base_sensor import BaseSensor
from ..gs_ros_utils import (
    create_qos_profile,
    get_current_timestamp,
    gs_quat_to_ros_quat,
)


class ImuSensor(BaseSensor):
    """ROS 2 IMU sensor that measures and publishes linear acceleration and angular velocity."""

    def __init__(
        self,
        sensor_config,
        node,
        scene,
        namespace,
        robot,
        time_offset,
        entities_info=None,
        robot_name=None,
    ):
        super().__init__(
            sensor_config,
            node,
            scene,
            namespace,
            robot,
            time_offset,
            entities_info,
            robot_name,
        )

    def add_sensor(self):
        """Instantiate the Genesis IMU sensor and setup its ROS 2 Imu message publisher."""
        gs.logger.info("imu Sensor created")

        frame_id = self.ros_options.get("frame_id", "")
        frequency = self.ros_options.get("frequency", 1.0)
        topic = self.ros_options.get("topic")

        def timer_callback(imu_pub, robot):
            if self.scene.is_built:
                assert imu._is_built, f"imu sensor is not built"
                data = imu.read()
                imu_msg = Imu()
                imu_msg.header.frame_id = frame_id
                imu_msg.header.stamp = get_current_timestamp(
                    self.scene, time_offset=self.time_offset
                )
                quat = gs_quat_to_ros_quat(
                    robot.get_quat().detach().cpu().numpy().tolist()[0]
                )
                imu_msg.orientation = Quaternion(
                    x=quat[0], y=quat[1], z=quat[2], w=quat[3]
                )
                ang_vel = data.ang_vel.detach().cpu().numpy().tolist()[0]
                imu_msg.angular_velocity = Vector3(
                    x=ang_vel[0], y=ang_vel[1], z=ang_vel[2]
                )
                lin_acc = data.lin_acc.detach().cpu().numpy().tolist()[0]
                imu_msg.linear_acceleration = Vector3(
                    x=lin_acc[0], y=lin_acc[1], z=lin_acc[2]
                )
                imu_pub.publish(imu_msg)

        entity_idx = self.robot.idx
        link_name = self.rigid_options.get("link")
        link = self.robot.get_link(link_name)
        if link is None:
            raise ValueError(f"Link '{link_name}' not found in entity robot")
        link_idx_local = link.idx_local

        pos_offset = self.rigid_options.get("pos_offset", (0, 0, 0))
        euler_offset = self.rigid_options.get("euler_offset", (0, 0, 0))
        draw_debug = self.general_options.get("draw_debug", False)

        imu = self.scene.add_sensor(
            gs.sensors.IMU(
                entity_idx=entity_idx,
                link_idx_local=link_idx_local,
                pos_offset=pos_offset,
                euler_offset=euler_offset,
                draw_debug=draw_debug,
                acc_resolution=self.sensor_config.get("acc_resolution", 0.0),
                acc_axes_skew=self.sensor_config.get("acc_axes_skew", 0.0),
                acc_bias=self.sensor_config.get("acc_bias", (0.0, 0.0, 0.0)),
                acc_noise=self.sensor_config.get("acc_noise", (0.0, 0.0, 0.0)),
                acc_random_walk=self.sensor_config.get(
                    "acc_random_walk", (0.0, 0.0, 0.0)
                ),
                gyro_resolution=self.sensor_config.get("gyro_resolution", 0.0),
                gyro_axes_skew=self.sensor_config.get("gyro_axes_skew", 0.0),
                gyro_bias=self.sensor_config.get("gyro_bias", (0.0, 0.0, 0.0)),
                gyro_noise=self.sensor_config.get("gyro_noise", (0.0, 0.0, 0.0)),
                gyro_random_walk=self.sensor_config.get(
                    "gyro_random_walk", (0.0, 0.0, 0.0)
                ),
                debug_acc_color=self.sensor_config.get(
                    "debug_acc_color", (0.0, 1.0, 1.0, 0.5)
                ),
                debug_acc_scale=self.sensor_config.get("debug_acc_scale", 0.01),
                debug_gyro_color=self.sensor_config.get(
                    "debug_gyro_color", (1.0, 1.0, 0.0, 0.5)
                ),
                debug_gyro_scale=self.sensor_config.get("debug_gyro_scale", 0.01),
            )
        )

        self.sensor_object = imu

        imu_qos_profile = create_qos_profile(
            self.ros_options.get("qos_history"),
            self.ros_options.get("qos_depth"),
            self.ros_options.get("qos_reliability"),
            self.ros_options.get("qos_durability"),
        )
        imu_pub = self.node.create_publisher(
            Imu, f"{self.namespace}/{topic}", imu_qos_profile
        )
        self.sensor_publishers = [imu_pub]

        timer = self.node.create_timer(
            1 / frequency, lambda: timer_callback(imu_pub, self.robot)
        )
        setattr(self, f"{self.sensor_name}_imu_timer", timer)
        self.register_sensor(self.sensor_object, self.sensor_publishers)
        return self.sensor_object, self.sensor_publishers

import genesis as gs
from sensor_msgs.msg import PointCloud2
from .base_sensor import BaseSensor
from ..gs_ros_utils import create_qos_profile, get_current_timestamp
from .sensor_helper import grid_raycaster_to_pcd_msg


class GridLidarSensor(BaseSensor):
    """ROS 2 sensor that implements a grid-pattern Lidar, publishing PointCloud2 messages."""

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
        """Instantiate the Genesis grid-pattern Lidar and setup its PointCloud2 publisher."""
        gs.logger.info("grid_lidar Sensor created")

        frame_id = self.ros_options.get("frame_id", "")
        frequency = self.ros_options.get("frequency", 1.0)
        topic = self.ros_options.get("topic")

        add_noise = self.sensor_config.get("add_noise", False)
        noise_mean = self.sensor_config.get("noise_mean", 0.0)
        noise_std = self.sensor_config.get("noise_std", 0.0)

        def timer_callback(pcd_publisher, add_noise):
            if self.scene.is_built:
                assert grid_lidar._is_built, f"grid lidar sensor is not built"
                pcd_msg = grid_raycaster_to_pcd_msg(
                    grid_lidar,
                    stamp=get_current_timestamp(
                        self.scene, time_offset=self.time_offset
                    ),
                    frame_id=frame_id,
                    add_noise=add_noise,
                    noise_mean=noise_mean,
                    noise_std=noise_std,
                )
                pcd_publisher.publish(pcd_msg)

        entity_idx = self.robot.idx
        link_name = self.rigid_options.get("link")
        link = self.robot.get_link(link_name)
        if link is None:
            raise ValueError(f"Link '{link_name}' not found in entity robot")
        link_idx_local = link.idx_local

        min_range = self.sensor_config.get("min_range", 0.05)
        max_range = self.sensor_config.get("max_range", 100.0)
        pos_offset = self.rigid_options.get("pos_offset", (0, 0, 0))
        euler_offset = self.rigid_options.get("euler_offset", (0, 0, 0))

        grid_pattern = self.sensor_config.get("grid_pattern", {})
        resolution = grid_pattern.get("resolution", 0.5)
        grid_size = grid_pattern.get("size", (1.0, 1.0))
        ray_direction_euler = grid_pattern.get("direction", (0, 0, 90))
        draw_debug = self.general_options.get("draw_debug", False)

        if draw_debug:
            draw_point_radius = self.sensor_config.get("draw_point_radius", 0.02)
            ray_start_color = self.sensor_config.get(
                "ray_start_color", (0.5, 0.5, 1.0, 1.0)
            )
            ray_hit_color = self.sensor_config.get(
                "ray_hit_color", (1.0, 0.5, 0.5, 1.0)
            )
        else:
            draw_point_radius = 0.0
            ray_start_color = (0.0, 0.0, 0.0, 0.0)
            ray_hit_color = (0.0, 0.0, 0.0, 0.0)

        return_points_in_world_frame = self.sensor_config.get(
            "return_points_in_world_frame", False
        )

        pattern_cfg = gs.sensors.GridPattern(
            resolution=resolution, size=grid_size, direction=ray_direction_euler
        )
        grid_lidar = self.scene.add_sensor(
            gs.sensors.Lidar(
                pattern=pattern_cfg,
                entity_idx=entity_idx,
                link_idx_local=link_idx_local,
                return_world_frame=return_points_in_world_frame,
                pos_offset=pos_offset,
                euler_offset=euler_offset,
                min_range=min_range,
                max_range=max_range,
                draw_debug=draw_debug,
                debug_sphere_radius=draw_point_radius,
                debug_ray_start_color=ray_start_color,
                debug_ray_hit_color=ray_hit_color,
            )
        )

        self.sensor_object = grid_lidar

        grid_lidar_qos_profile = create_qos_profile(
            self.ros_options.get("qos_history"),
            self.ros_options.get("qos_depth"),
            self.ros_options.get("qos_reliability"),
            self.ros_options.get("qos_durability"),
        )
        grid_lidar_pub = self.node.create_publisher(
            PointCloud2, f"{self.namespace}/{topic}", grid_lidar_qos_profile
        )
        self.sensor_publishers = [grid_lidar_pub]

        timer = self.node.create_timer(
            1 / frequency, lambda: timer_callback(grid_lidar_pub, add_noise)
        )
        setattr(self, f"{self.sensor_name}_grid_lidar_timer", timer)
        self.register_sensor(self.sensor_object, self.sensor_publishers)
        return self.sensor_object, self.sensor_publishers

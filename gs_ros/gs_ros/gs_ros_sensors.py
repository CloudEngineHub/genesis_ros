import genesis as gs
from rclpy.node import Node
from .sensors.sensor_helper import make_cv2_bridge
from .sensors import (
    CameraSensor,
    GridLidarSensor,
    SectionalLidarSensor,
    LidarSensor,
    LaserScanSensor,
    ImuSensor,
    ContactForceSensor,
    ContactSensor,
)


class GsRosSensors:
    """Factory class to manage and instantiate various ROS 2 sensors for a robot."""

    def __init__(
        self,
        scene,
        namespace,
        time_offset,
        robot_name=None,
        entities_info=None,
    ):
        """Initialize the sensor factory with simulation and robot context."""
        gs.logger.info("Starting all sensor data publishers")
        self.scene = scene
        self.bridge = make_cv2_bridge()
        self.namespace = namespace
        self.robot = entities_info[robot_name]["entity_attr"]
        self.time_offset = time_offset
        self.entities_info = entities_info
        self.robot_name = robot_name
        self.sensors = {}
        self.cameras = {}
        self.all_ros_nodes = []

    def add_sensor(self, sensor_config):
        """Instantiate a specific sensor based on configuration and register its publishers."""
        general_options = sensor_config.get("general_sensor_options", {})
        sensor_name = sensor_config.get("name")
        if sensor_name is None:
            raise ValueError("Sensor name not specified, sensor options invalid")

        sensor_type = general_options.get("sensor_type")
        if sensor_type is None:
            raise ValueError("Sensor type not specified, sensor options invalid")

        sensor_mapping = {
            "cam": ("CAM_NODE", CameraSensor),
            "grid_lidar": ("GRID_LIDAR_NODE", GridLidarSensor),
            "sectional_lidar": ("SECTIONAL_LIDAR_NODE", SectionalLidarSensor),
            "lidar": ("LIDAR_NODE", LidarSensor),
            "laser_scan": ("LASER_SCAN_NODE", LaserScanSensor),
            "imu": ("IMU_NODE", ImuSensor),
            "contact_force": ("CONTACT_FORCE_NODE", ContactForceSensor),
            "contact": ("CONTACT_NODE", ContactSensor),
        }

        if sensor_type not in sensor_mapping:
            gs.logger.error(f"Unknown sensor type: {sensor_type}")
            return None

        node_prefix, sensor_class = sensor_mapping[sensor_type]
        node = Node(f"{node_prefix}_{self.namespace}_{sensor_name}")
        self.all_ros_nodes.append(node)

        sensor_instance = sensor_class(
            sensor_config=sensor_config,
            node=node,
            scene=self.scene,
            namespace=self.namespace,
            robot=self.robot,
            time_offset=self.time_offset,
            entities_info=self.entities_info,
            robot_name=self.robot_name,
        )

        sensor_object, sensor_publishers = sensor_instance.add_sensor()

        # Backward compatibility for GsRosSensors internal state tracking
        if sensor_type == "cam":
            self.cameras[sensor_name] = [sensor_object, sensor_object._idx]
            self.sensors[sensor_name] = "CAM"
        elif sensor_type in ["grid_lidar", "sectional_lidar"]:
            self.sensors[sensor_name] = "SECLIDAR"
        elif sensor_type in ["lidar", "laser_scan"]:
            self.sensors[sensor_name] = "LIDAR"
        elif sensor_type == "imu":
            self.sensors[sensor_name] = "IMU"
        elif sensor_type == "contact_force":
            self.sensors[sensor_name] = "CONTACT_FORCE"
        elif sensor_type == "contact":
            self.sensors[sensor_name] = "CONTACT"

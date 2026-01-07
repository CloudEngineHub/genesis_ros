from .sensors.sensor_helper import *
from .gs_ros_robot_control import *
from .gs_ros_sensors import *
from .gs_ros_services import *
from .gs_ros_sim import *
from .gs_ros_utils import make_gs_scene, get_current_timestamp

from rclpy.node import Node
import rclpy
import yaml

from gs_simulation_interfaces import SimulationInterface


class GsRosBridge:
    """Main bridge class between Genesis simulation and ROS 2."""

    def __init__(
        self,
        ros_node,
        file_path=None,
        enable_simulation_interfaces=True,
        ros_clock_node=None,
        ros_control_node=None,
        ros_service_node=None,
        add_debug_objects=False,
    ):
        """Initialize the bridge, setting up ROS nodes, scene, and entities."""

        self.ros_node = ros_node
        self.all_nodes_to_spin = [self.ros_node]
        if ros_clock_node is not None:
            self.ros_clock_node = ros_clock_node
            self.all_nodes_to_spin.append(self.ros_clock_node)
        else:
            self.ros_clock_node = ros_node

        if ros_service_node is not None:
            self.ros_service_node = ros_service_node
            self.all_nodes_to_spin.append(self.ros_service_node)
        else:
            self.ros_service_node = ros_node

        if ros_control_node is not None:
            self.ros_control_node = ros_control_node
            self.all_nodes_to_spin.append(self.ros_control_node)
        else:
            self.ros_control_node = ros_node

        self.scene = gs.Scene()
        if enable_simulation_interfaces:
            self.simulation_interface = SimulationInterface(
                self.scene, on_build_callback=self.initialise_robots
            )
            self.scene_manager = self.simulation_interface.scene_manager
            self.entities_info = self.scene_manager.entities_info
            self.world_info = self.scene_manager.world_info
        else:
            self.scene_manager = None
            self.entities_info = {}
            self.world_info = {}

        if file_path is not None:
            with open(file_path, "r") as file:
                self.parent_config = yaml.safe_load(file)
            self.scene = make_gs_scene(scene_config=self.parent_config["scene"])
            if enable_simulation_interfaces:
                self.scene_manager.scene = self.scene
            self.sim = GsRosSim(self.scene, self.parent_config["scene"])

            if self.parent_config.get("world") is not None:
                world_name = self.parent_config["world"]["name"]
                gs.logger.info(f"Adding world {world_name} to scene")
                self.sim.spawn_from_config(
                    entity_config=self.parent_config["world"],
                    entity_name=world_name,
                    entities_info=self.world_info,
                )

            if self.parent_config.get("objects") is not None:
                for object_name, object_config in self.parent_config.get(
                    "objects", {}
                ).items():
                    gs.logger.info(f"Adding object {object_name} to scene")
                    self.sim.spawn_from_config(
                        entity_config=object_config,
                        entity_name=object_name,
                        entities_info=self.entities_info,
                    )

            if self.parent_config.get("robots") is not None:
                for robot_name, robot_config in self.parent_config.get(
                    "robots", {}
                ).items():
                    namespace = robot_config.get("namespace", "")
                    gs.logger.info(f"Adding robot {namespace} to scene")
                    self.sim.spawn_from_config(
                        entity_config=robot_config,
                        entity_name=robot_name,
                        entities_info=self.entities_info,
                    )
                    setattr(
                        self,
                        f"{robot_name}_robot_control",
                        GsRosRobotControl(
                            self.scene,
                            self.ros_control_node,
                            robot_config.get("control", {}),
                            robot_name=robot_name,
                            entities_info=self.entities_info,
                            time_offset=self.scene_manager.time_offset,
                        ),
                    )
                    sensor_factory = GsRosSensors(
                        self.scene,
                        namespace,
                        time_offset=self.scene_manager.time_offset,
                        robot_name=robot_name,
                        entities_info=self.entities_info,
                    )
                    setattr(self, f"{robot_name}_sensor_factory", sensor_factory)
                    if robot_config.get("sensors") is not None:
                        for sensor_config in robot_config.get("sensors", {}):
                            sensor_factory.add_sensor(sensor_config)
                    self.all_nodes_to_spin.extend(sensor_factory.all_ros_nodes)

            self.services = GsRosServices(
                self.scene,
                self.ros_service_node,
                self.entities_info,
            )
        else:
            self.scene = None
            gs.logger.warn(
                f"No config file path provided, please provide the path or add the scene, robots, etc as necessary"
            )
        if add_debug_objects:
            # Number of obstacles to create in a ring around the robot
            NUM_CYLINDERS = 8
            NUM_BOXES = 6
            CYLINDER_RING_RADIUS = 3.0
            BOX_RING_RADIUS = 5.0

            for i in range(NUM_CYLINDERS):
                angle = 2 * np.pi * i / NUM_CYLINDERS
                x = CYLINDER_RING_RADIUS * np.cos(angle)
                y = CYLINDER_RING_RADIUS * np.sin(angle)
                self.scene.add_entity(
                    gs.morphs.Cylinder(
                        height=1.5,
                        radius=0.3,
                        pos=(x, y, 0.75),
                        fixed=True,
                    )
                )

            for i in range(NUM_BOXES):
                angle = 2 * np.pi * i / NUM_BOXES + np.pi / 6
                x = BOX_RING_RADIUS * np.cos(angle)
                y = BOX_RING_RADIUS * np.sin(angle)
                self.scene.add_entity(
                    gs.morphs.Box(
                        size=(0.5, 0.5, 2.0 * (i + 1) / NUM_BOXES),
                        pos=(x, y, 1.0),
                        fixed=False,
                    )
                )

    def build(self):
        """Build the Genesis scene and start the ROS clock."""
        self.sim.build_scene(self.parent_config["scene"])
        for _ in range(20):
            self.scene.step()
        self.sim.start_clock(self.ros_clock_node, self.scene_manager.time_offset)

    def step(self):
        """Step the simulation and spin ROS nodes."""
        if self.scene_manager is not None and self.scene_manager.PENDING_REST:
            self.scene_manager.scene.destroy()
            gs.logger.critical("simulation terminated")
            raise KeyboardInterrupt
        if (
            self.scene_manager is not None
            and self.scene_manager.current_state_code == self.scene_manager.PLAYING
        ):
            self.scene.step()
        else:
            self.scene.step()
        if self.scene_manager is not None:
            self.scene_manager.latest_timestamp = get_current_timestamp(
                self.scene_manager.scene, self.scene_manager.time_offset
            )
        if rclpy.ok():
            for ros_node in self.all_nodes_to_spin:
                rclpy.spin_once(ros_node, timeout_sec=0)
            if hasattr(self, "simulation_interface"):
                rclpy.spin_once(self.simulation_interface, timeout_sec=0)

    def initialise_robots(self):
        """Dynamically initialize robot control and sensors for newly spawned robots."""
        if self.entities_info is not None:
            for robot_name, robot_entry in self.entities_info.items():
                if robot_entry.get("initialisation_pending", False):
                    namespace = robot_entry["namespace"]
                    if robot_entry.get("robot_options") is None:
                        gs.logger.critical(
                            f"Robot {robot_name} has no robot options and the subscirbers, publishers for this cant be added"
                        )
                        continue
                    setattr(
                        self,
                        f"{robot_name}_robot_control",
                        GsRosRobotControl(
                            self.scene,
                            self.ros_control_node,
                            robot_entry.get("robot_options").get("control", {}),
                            robot_name=robot_name,
                            entities_info=self.entities_info,
                            time_offset=self.scene_manager.time_offset,
                        ),
                    )
                    sensor_factory = GsRosSensors(
                        self.scene,
                        namespace,
                        time_offset=self.scene_manager.time_offset,
                        robot_name=robot_name,
                        entities_info=self.entities_info,
                    )
                    setattr(self, f"{robot_name}_sensor_factory", sensor_factory)
                    if robot_entry.get("sensor_options") is not None:
                        for sensor_config in robot_entry.get("sensor_options").get(
                            "sensors", []
                        ):
                            sensor_factory.add_sensor(sensor_config)
                        self.all_nodes_to_spin.extend(sensor_factory.all_ros_nodes)
                    else:
                        gs.logger.critical(
                            f"Robot {robot_name} has no sensor options, sensor data publishers cannot be added"
                        )
                    robot_entry["initialisation_pending"] = False

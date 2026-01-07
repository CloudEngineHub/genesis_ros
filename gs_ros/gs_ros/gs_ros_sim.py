import genesis as gs
import numpy as np
from rosgraph_msgs.msg import Clock
from .gs_ros_utils import (
    make_morph,
    make_material,
    make_surface,
    make_terrain,
    make_primitive,
    calculate_bounds,
)


class GsRosSim:
    """Handles core Genesis simulation operations like building scenes and spawning entities."""

    def __init__(self, scene, scene_config):
        """Initialize the simulator wrapper with scene reference and configuration."""
        self.scene = scene
        self.scene_config = scene_config
        self.STOP_SIMULATOR = False

    def build_scene(self, scene_config):
        """Construct the physical scene based on simulation parameters."""
        self.scene.build(scene_config["n_envs"])

    def spawn_from_config(self, entity_config, entity_name, entities_info):
        """Create and add an entity (robot, object, or world) to the simulation."""
        if entity_config.get("world_type") is not None:
            if entity_config.get("world_type") in ["plane", "terrain", "file"]:
                terrain = make_terrain(entity_config)
                bounds = calculate_bounds(entity_config.get("world_type"), terrain)
                world_entity = self.scene.add_entity(terrain)
                entities_info[entity_name] = {
                    "world_name": entity_name,
                    "world_bounds": bounds,
                    "world_attr": world_entity,
                }
                return world_entity
            elif entity_config.get("world_type") == "file":
                gs.logger.warn(
                    "when loading the world from a file make sure that world is fixed"
                )
        if entity_config.get("type") in ["box", "sphere", "cylinder"]:
            morph = make_primitive(
                entity_config.get("type"), entity_config.get("morph")
            )
        else:
            morph = make_morph(entity_config.get("morph"))
        material = make_material(entity_config.get("material"))
        surface = make_surface(entity_config.get("surface"))
        visualize_contact = entity_config.get("visualize_contact", None)
        entity = self.scene.add_entity(
            morph=morph,
            material=material,
            surface=surface,
            visualize_contact=visualize_contact,
        )
        if entities_info is not None:
            if entity_config.get("world_type") is not None:
                entities_info[entity_name] = {
                    "world_name": entity_name,
                    "world_attr": entity,
                    "world_resource": morph["file"],
                }
            else:
                entities_info[entity_name] = {}
                entities_info[entity_name]["entity_morph"] = morph
                entities_info[entity_name]["initial_pos"] = entity_config["morph"].get(
                    "pos", [0, 0, 0]
                )
                entities_info[entity_name]["initial_quat"] = entity_config["morph"].get(
                    "quat", [1, 0, 0, 0]
                )
                entities_info[entity_name]["resource"] = entity_config["morph"].get(
                    "file", None
                )
                entities_info[entity_name]["namespace"] = entity_config.get(
                    "namespace", None
                )
                entities_info[entity_name]["entity_attr"] = entity
                entities_info[entity_name]["initialisation_pending"] = False
                entities_info[entity_name]["description"] = entity_config.get(
                    "description", ""
                )
                entities_info[entity_name]["tags"] = entity_config.get("tags", [])
                entities_info[entity_name]["category"] = entity_config.get(
                    "category", 1
                )
        return entity

    def start_clock(self, ros_node, time_offset=0):
        """Start a ROS 2 clock publisher synchronized with simulation time."""

        def timer_callback(clock_publisher):
            time = self.scene.cur_t - time_offset
            msg = Clock()
            msg.clock.sec = int(time)
            msg.clock.nanosec = int((time - msg.clock.sec) * 10e8)
            clock_publisher.publish(msg)

        self.pub = ros_node.create_publisher(Clock, "/clock", 50)
        # Timer fires every 0.005 seconds
        self.timer = ros_node.create_timer(0.005, lambda: timer_callback(self.pub))

import genesis as gs
import numpy as np
from rosgraph_msgs.msg import Clock
from .gs_ros_utils import (
    make_morph,
    make_material,
    make_surface)

class GsRosSim:
    def __init__(self,scene,scene_config):
        self.scene=scene
        self.scene_config=scene_config
        self.STOP_SIMULATOR=False
        # self.start_simulation()

    def build_scene(self,scene_config):
        # self.scene._visualizer.build()
        self.scene.build(scene_config["n_envs"])

    # def start_simulation(self,ros_executor):
    #     while not self.STOP_SIMULATOR:
    #         self.scene.step()
    #         ros_executor.spin_once()
            
    def add_world(self,world_config):
        if world_config["plane"] is True:
            self.scene.add_entity(gs.morphs.Plane())
        elif world_config.get("generated",None) is not None:
            terrain_config=world_config.get("generated",None)
            self.scene.add_entity(
                morph=gs.morphs.Terrain(
                    pos = terrain_config.get("pos", (0.0, 0.0, 0.0)),
                    euler = terrain_config.get("euler", (0.0, 0.0, 0.0)),
                    quat = terrain_config.get("quat", None),
                    visualization = terrain_config.get("visualization", True),
                    collision = terrain_config.get("collision", True),
                    requires_jac_and_IK = terrain_config.get("requires_jac_and_IK", False),
                    randomize = terrain_config.get("randomize", False),
                    n_subterrains = terrain_config.get("n_subterrains", (3, 3)),
                    subterrain_size = terrain_config.get("subterrain_size", (12.0, 12.0)),
                    horizontal_scale = terrain_config.get("horizontal_scale", 0.25),
                    vertical_scale = terrain_config.get("vertical_scale", 0.005),
                    subterrain_types = terrain_config.get("subterrain_types", 'flat_terrain'),
                    height_field = terrain_config.get("height_field", None)
                ),
            )
        
    def spawn_from_config(self, entity_config):
        return self.scene.add_entity(
            morph=make_morph(entity_config.get("morph")),
            material=make_material(entity_config.get("material")),
            surface=make_surface(entity_config.get("surface")),
            visualize_contact=entity_config.get("visualize_contact", None)
        )

            
    def start_clock(self,ros_node):
        def timer_callback(clock_publisher):
            msg = Clock()
            # print(msg)
            msg.clock.sec=int(self.scene.cur_t)
            msg.clock.nanosec=int((self.scene.cur_t-msg.clock.sec)*10e8)
            # print(msg.clock.sec,"::",msg.clock.nanosec)
            # self.current_time_stamp=msg.clock
            clock_publisher.publish(msg)
        self.pub = ros_node.create_publisher(Clock, '/clock', 50)
        # Timer fires every 0.5 seconds
        self.timer = ros_node.create_timer(0.02,  lambda: timer_callback(self.pub))

        # if morph_config is not None:
        #     for joint_name in morph_config["dof_names"]:
        #         for joint in robot.joints:
        #             if joint.name ==joint_name:
        #                 motor_dofs.append(joint.dof_idx_local)
        #     return motor_dofs
        # return None



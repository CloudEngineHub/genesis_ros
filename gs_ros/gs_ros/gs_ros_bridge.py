from .gs_ros_sensor_helper import *
from .gs_ros_robot_control import *
from .gs_ros_sensors import *
from .gs_ros_services import *
from .gs_ros_sim import *
from .gs_ros_utils import make_gs_scene

from rclpy.node import Node
import rclpy

import yaml
import time

class GsRosBridge:
    def __init__(self,file_path,ros_node,
                 ros_clock_node=None,ros_service_node=None):

        self.ros_node=ros_node
        if ros_clock_node is not None:
            self.ros_clock_node=ros_clock_node
        else:
            self.ros_clock_node=ros_node
        if ros_service_node is not None:
            self.ros_service_node=ros_service_node
        else:
            self.ros_service_node=ros_node   
        with open(file_path, 'r') as file:
            self.parent_config=yaml.safe_load(file)
        self.scene= make_gs_scene(scene_config=self.parent_config["scene"])
        self.sim=Gs_Ros_Sim(self.scene,self.parent_config["scene"])
        self.sim.add_world(self.parent_config["world"])
        
        self.robots=[]
        self.objects=[]
        self.all_sensors={}
        self.cameras=[]
        self.all_sensors={}
        self.all_cameras={}
        
        for _, object_config in self.parent_config.get("objects", {}).items():
            object_name=object_config["name"]
            object=self.sim.spawn_from_config(entity_config=object_config)
            self.objects.append((object_name,object.idx,object))
            
        for robot_name, robot_config in self.parent_config.get("robots", {}).items():
            namespace=robot_config.get("namespace","")
            robot=self.sim.spawn_from_config(entity_config=robot_config)
            self.robots.append((robot_name,robot.idx,robot))
            setattr(self,f"{robot_name}_robot_control",Gs_Ros_Robot_Control(self.scene,self.ros_node,robot_config,robot))
            sensor_factory=Gs_Ros_Sensors(self.scene,self.sim,self.ros_node,namespace,robot,self.robots,self.objects)
            setattr(self,f"{robot_name}_sensor_factory",sensor_factory)
            sensors=[]
            for sensor_config in robot_config.get("sensors",{}):
                sensor_type=sensor_factory.add_sensor(sensor_config)
                sensors.append([sensor_type,sensor_config["name"]])
            self.all_cameras[robot_name]=sensor_factory.cameras
            self.all_sensors[robot_name]=sensors
        
        self.services=Gs_Ros_Services(self.scene,self.ros_service_node,self.robots,self.objects,self.all_cameras)
    
    def build(self):
        self.sim.build_scene(self.parent_config["scene"])
        self.sim.start_clock(self.ros_clock_node)
        
    def step(self):
        if rclpy.ok():
            rclpy.spin_once(self.ros_node)
            self.scene.step()
            
                    
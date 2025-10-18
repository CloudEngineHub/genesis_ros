from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from .gs_ros_utils import (
    get_current_timestamp,
    get_joint_names,
    get_dofs_idx)

import genesis as gs

class GsRosRobotControl:
    def __init__(self,scene,ros_node,robot_config,robot):
        gs.logger.info("starting robot control interfaces")
        self.scene=scene
        self.ros_node=ros_node
        self.robot_config=robot_config
        self.namespace=robot_config.get("namespace","robot")
        self.robot=robot
        self.joint_names=[joint.name for joint in self.robot.joints]
        self.motor_dofs=get_dofs_idx(robot,joint_names=self.joint_names)

        self.dof_properties_set=False
        self.publish_joint_states()
        self.setup_control_subscriber()
        self.setup_joint_state_target_subscriber()
    
    def set_dofs_properties(self):
        joint_properties=self.robot_config.get("joint_properties",None)
        if joint_properties is not None:
            if any('kp' in joint_cfg for joint_cfg in joint_properties.values()):
                self._set_dofs_kp()
            if any('kv' in joint_cfg for joint_cfg in joint_properties.values()):
                self._set_dofs_kv()
            if any('stiffness' in joint_cfg for joint_cfg in joint_properties.values()):
                self._set_dofs_stiffness()
            if any('armature' in joint_cfg for joint_cfg in joint_properties.values()):
                self._set_dofs_armature()
            if any('damping' in joint_cfg for joint_cfg in joint_properties.values()):
                self._set_dofs_damping()
            if any('force_range' in joint_cfg for joint_cfg in joint_properties.values()):
                self._set_dofs_force_range()

    def _set_dofs_kp(self):
        joint_properties=self.robot_config.get("joint_properties",None)
        for joint_name,joint_cfg in joint_properties.items():
            joint_idx=get_dofs_idx(self.robot,joint_names=[joint_name])
            self.robot.set_dofs_kp([joint_cfg.get('kp',1)],joint_idx)

    def _set_dofs_kv(self):
        joint_properties=self.robot_config.get("joint_properties",None)
        for joint_name,joint_cfg in joint_properties.items():
            joint_idx=get_dofs_idx(self.robot,joint_names=[joint_name])
            self.robot.set_dofs_kv([joint_cfg.get('kv',1)],joint_idx)

    def _set_dofs_stiffness(self):
        joint_properties=self.robot_config.get("joint_properties",None)
        for joint_name,joint_cfg in joint_properties.items():
            joint_idx=get_dofs_idx(self.robot,joint_names=[joint_name])
            self.robot.set_dofs_stiffness([joint_cfg.get('stiffness',1)],joint_idx)

    def _set_dofs_armature(self):
        joint_properties=self.robot_config.get("joint_properties",None)
        for joint_name,joint_cfg in joint_properties.items():
            joint_idx=get_dofs_idx(self.robot,joint_names=[joint_name])
            self.robot.set_dofs_armature([joint_cfg.get('armature',0)],joint_idx)

    def _set_dofs_damping(self):
        joint_properties=self.robot_config.get("joint_properties",None)
        for joint_name,joint_cfg in joint_properties.items():
            joint_idx=get_dofs_idx(self.robot,joint_names=[joint_name])
            self.robot.set_dofs_damping([joint_cfg.get('damping',0)],joint_idx)

    def _set_dofs_force_range(self):
        joint_properties=self.robot_config.get("joint_properties",None)
        for joint_name,joint_cfg in joint_properties.items():
            joint_idx=get_dofs_idx(self.robot,joint_names=[joint_name])
            self.robot.set_dofs_force_range([joint_cfg.get('force_range',[-1,1])[0]],[joint_cfg.get('force_range',[-1,1])[1]],joint_idx)

    def publish_joint_states(self):
        gs.logger.info("Joint state Publisher started")
        def timer_callback(js_publisher):
            dof_names,dof_idx_local=get_joint_names(self.robot)
            joint_qpos=self.robot.get_dofs_position(dof_idx_local).detach().cpu().numpy().tolist()
            joint_qvel=self.robot.get_dofs_velocity(dof_idx_local).detach().cpu().numpy().tolist()
            joint_qforce=self.robot.get_dofs_control_force(dof_idx_local).detach().cpu().numpy().tolist()
            joint_state_msg=JointState()
            joint_state_msg.header.stamp=get_current_timestamp(self.scene)
            joint_state_msg.name=dof_names
            joint_state_msg.position=joint_qpos[0]
            joint_state_msg.velocity=joint_qvel[0]
            joint_state_msg.effort=joint_qforce[0]
            js_publisher.publish(joint_state_msg)
        self.joint_state_publisher = self.ros_node.create_publisher(JointState, f'{self.namespace}/joint_states', 50)
        self.timer = self.ros_node.create_timer(0.02,  lambda: timer_callback(self.joint_state_publisher))

    def set_dof_qpos(self,pos):
        self.robot.set_qpos(pos,self.motor_dofs)

    def _control_dof(self, target_point,motor_dofs):
        if motor_dofs is None:
            motor_dofs=self.motor_dofs
        if len(target_point.positions) ==motor_dofs:
            self.robot.control_dofs_position(target_point.positions,motor_dofs)
        elif len(target_point.velocities) ==motor_dofs:
            self.robot.control_dofs_velocity(target_point.velocities,motor_dofs)
        elif len(target_point.efforts) ==motor_dofs:
            self.robot.control_dofs_effort(target_point.efforts,motor_dofs)
        else:
            print("Invalid control Type specified")
            raise NotImplementedError

    def setup_control_subscriber(self):
        gs.logger.info("control command subscriber started")
        def control_callback(self,msg):
            motor_dofs=get_dofs_idx(msg.joint_names)
            for point in msg.points:
                self._control_dof(point,motor_dofs)
        control_sub =self.ros_node.create_subscription(JointTrajectory,
                                                            f'{self.namespace}/{self.robot_config["control_topic"]}',
                                                            control_callback,
                                                            10)
        setattr(self,f'{self.namespace}_control_subscriber',control_sub)

    def _control_dofs_pos(self, target_qpos,motor_dofs=None):
        if len(target_qpos)>0 and len(motor_dofs)>0:
            if motor_dofs is None:
                motor_dofs=self.motor_dofs
            self.robot.control_dofs_position(target_qpos,motor_dofs)
        
    def _control_dofs_vel(self, target_qpos,motor_dofs=None):
        if len(target_qpos)>0 and len(motor_dofs)>0:
            if motor_dofs is None:
                motor_dofs=self.motor_dofs
            self.robot.control_dofs_velocity(target_qpos,motor_dofs)
        
    def _control_dofs_eff(self, target_qpos,motor_dofs=None):
        if len(target_qpos)>0 and len(motor_dofs)>0:
            if motor_dofs is None:
                motor_dofs=self.motor_dofs
            self.robot.control_dofs_force(target_qpos,motor_dofs)
        
    def setup_joint_state_target_subscriber(self):
        gs.logger.info("joint state target subscriber started")
        def joint_state_target_callback(msg):
            motor_dofs=get_dofs_idx(self.robot,msg.name)
            dof_idx_table={}
            for k,motor_dof in enumerate(motor_dofs):
                dof_idx_table[msg.name[k]]=motor_dof
            valid=True
            joint_properties=dict(sorted(self.robot_config.get("joint_properties",None).items()))
            pos_i,vel_i,eff_i=0,0,0
            pos_vals,pos_dofs=[],[]
            vel_vals,vel_dofs=[],[]
            eff_vals,eff_dofs=[],[]
            if self.scene.is_built and not self.dof_properties_set:
                self.set_dofs_properties()
                self.dof_properties_set=True
            for joint,joint_cfg in joint_properties.items():
                if joint_cfg.get("command","").lower() =='position':
                    pos_vals.append(msg.position[pos_i])
                    pos_dofs.append(dof_idx_table[joint])
                    pos_i+=1
                elif joint_cfg.get("command","").lower() =='velocity':
                    vel_vals.append(msg.velocity[vel_i])
                    vel_dofs.append(dof_idx_table[joint])
                    vel_i+=1
                elif joint_cfg.get("command","").lower() =='effort':
                    eff_vals.append(msg.effort[eff_i])
                    eff_dofs.append(dof_idx_table[joint])
                    eff_i+=1
                else:
                    print("Invalid joint command type")
                    valid=False
            if valid:
                self._control_dofs_pos(pos_vals,pos_dofs)
                self._control_dofs_vel(vel_vals,vel_dofs)
                self._control_dofs_pos(eff_vals,eff_dofs)
                    
        joint_state_target_subscriber=self.ros_node.create_subscription(JointState,
                                                                    f'{self.namespace}/{self.robot_config["joint_state_target_topic"]}',
                                                                    joint_state_target_callback,
                                                                    self.robot_config["joint_state_target_topic_frequency"])
        setattr(self,
                f'{self.robot}_joint_state_target_subscriber',
                joint_state_target_subscriber)
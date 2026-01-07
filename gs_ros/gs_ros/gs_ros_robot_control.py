from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from .gs_ros_utils import get_current_timestamp, get_joint_names, get_dofs_idx

import genesis as gs


class GsRosRobotControl:
    """Manages robot joint states and control commands via ROS 2 topics."""

    def __init__(
        self,
        scene,
        ros_node,
        robot_config,
        entities_info=None,
        robot_name=None,
        time_offset=0,
    ):
        """Initialize the robot control interface, setup publishers and subscribers."""
        gs.logger.info("starting robot control interfaces")
        self.scene = scene
        self.ros_node = ros_node
        self.entities_info = entities_info
        self.robot_name = robot_name
        self.time_offset = time_offset
        self.robot_config = robot_config
        self.namespace = robot_config.get("namespace", "robot")
        self.robot = entities_info[robot_name]["entity_attr"]
        self.joint_names = [joint.name for joint in self.robot.joints]
        self.motor_dofs = get_dofs_idx(self.robot, joint_names=self.joint_names)

        self.register_robot_options()

        self.dof_properties_set = False
        self.setup_joint_states_publisher()
        self.setup_control_subscriber()
        self.setup_joint_commands_subscriber()

    def register_robot_options(self):
        """Register robot configuration in entities_info"""
        if self.entities_info is None or self.robot_name is None:
            return

        if self.robot_name not in self.entities_info.keys():
            gs.logger.warn(
                f"Robot '{self.robot_name}' not found in entities_info, cannot register robot config."
            )
            return

        entity_entry = self.entities_info[self.robot_name]
        entity_entry["robot_options"] = self.robot_config

    def set_dofs_properties(self):
        """Apply physics properties (KP, KV, etc.) to the robot's degrees of freedom."""
        joint_properties = self.robot_config.get("joint_properties", None)
        if joint_properties is not None:
            if any("kp" in joint_cfg for joint_cfg in joint_properties.values()):
                self._set_dofs_kp()
            if any("kv" in joint_cfg for joint_cfg in joint_properties.values()):
                self._set_dofs_kv()
            if any("stiffness" in joint_cfg for joint_cfg in joint_properties.values()):
                self._set_dofs_stiffness()
            if any("armature" in joint_cfg for joint_cfg in joint_properties.values()):
                self._set_dofs_armature()
            if any("damping" in joint_cfg for joint_cfg in joint_properties.values()):
                self._set_dofs_damping()
            if any(
                "force_range" in joint_cfg for joint_cfg in joint_properties.values()
            ):
                self._set_dofs_force_range()

    def _set_dofs_kp(self):
        joint_properties = self.robot_config.get("joint_properties", None)
        for joint_name, joint_cfg in joint_properties.items():
            joint_idx = get_dofs_idx(self.robot, joint_names=[joint_name])
            self.robot.set_dofs_kp([joint_cfg.get("kp", 1)], joint_idx)

    def _set_dofs_kv(self):
        joint_properties = self.robot_config.get("joint_properties", None)
        for joint_name, joint_cfg in joint_properties.items():
            joint_idx = get_dofs_idx(self.robot, joint_names=[joint_name])
            self.robot.set_dofs_kv([joint_cfg.get("kv", 1)], joint_idx)

    def _set_dofs_stiffness(self):
        joint_properties = self.robot_config.get("joint_properties", None)
        for joint_name, joint_cfg in joint_properties.items():
            joint_idx = get_dofs_idx(self.robot, joint_names=[joint_name])
            self.robot.set_dofs_stiffness([joint_cfg.get("stiffness", 1)], joint_idx)

    def _set_dofs_armature(self):
        joint_properties = self.robot_config.get("joint_properties", None)
        for joint_name, joint_cfg in joint_properties.items():
            joint_idx = get_dofs_idx(self.robot, joint_names=[joint_name])
            self.robot.set_dofs_armature([joint_cfg.get("armature", 0)], joint_idx)

    def _set_dofs_damping(self):
        joint_properties = self.robot_config.get("joint_properties", None)
        for joint_name, joint_cfg in joint_properties.items():
            joint_idx = get_dofs_idx(self.robot, joint_names=[joint_name])
            self.robot.set_dofs_damping([joint_cfg.get("damping", 0)], joint_idx)

    def _set_dofs_force_range(self):
        joint_properties = self.robot_config.get("joint_properties", None)
        for joint_name, joint_cfg in joint_properties.items():
            joint_idx = get_dofs_idx(self.robot, joint_names=[joint_name])
            self.robot.set_dofs_force_range(
                [joint_cfg.get("force_range", [-1, 1])[0]],
                [joint_cfg.get("force_range", [-1, 1])[1]],
                joint_idx,
            )

    def setup_joint_states_publisher(self):
        """Initialize and start the joint state publisher timer."""
        gs.logger.info("Joint state Publisher started")

        def timer_callback(js_publisher):
            dof_names, dof_idx_local = get_joint_names(self.robot)
            joint_qpos = (
                self.robot.get_dofs_position(dof_idx_local)
                .detach()
                .cpu()
                .numpy()
                .tolist()
            )
            joint_qvel = (
                self.robot.get_dofs_velocity(dof_idx_local)
                .detach()
                .cpu()
                .numpy()
                .tolist()
            )
            joint_qforce = (
                self.robot.get_dofs_control_force(dof_idx_local)
                .detach()
                .cpu()
                .numpy()
                .tolist()
            )
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = get_current_timestamp(
                self.scene, time_offset=self.time_offset
            )
            joint_state_msg.name = dof_names
            joint_state_msg.position = joint_qpos[0]
            joint_state_msg.velocity = joint_qvel[0]
            joint_state_msg.effort = joint_qforce[0]
            js_publisher.publish(joint_state_msg)

        self.joint_state_publisher = self.ros_node.create_publisher(
            JointState,
            f'{self.namespace}/{self.robot_config.get("joint_states_topic", "joint_states")}',
            10,
        )
        self.timer = self.ros_node.create_timer(
            1.0 / self.robot_config.get("joint_states_topic_frequency", 50.0),
            lambda: timer_callback(self.joint_state_publisher),
        )

    def _trajectory_point_controller(self, point, joint_properties, dof_idx_table):
        valid = True
        pos_i, vel_i, eff_i = 0, 0, 0
        pos_vals, pos_dofs = [], []
        vel_vals, vel_dofs = [], []
        eff_vals, eff_dofs = [], []
        for joint, joint_cfg in joint_properties.items():
            if joint_cfg.get("command", "").lower() == "position":
                pos_vals.append(point.position[pos_i])
                pos_dofs.append(dof_idx_table[joint])
                pos_i += 1
            elif joint_cfg.get("command", "").lower() == "velocity":
                vel_vals.append(point.velocity[vel_i])
                vel_dofs.append(dof_idx_table[joint])
                vel_i += 1
            elif joint_cfg.get("command", "").lower() == "effort":
                eff_vals.append(msg.effort[eff_i])
                eff_dofs.append(dof_idx_table[joint])
                eff_i += 1
            else:
                gs.logger.error(f"Invalid joint command type for {joint} joint")
                valid = False
        if valid:
            self._control_dofs_pos(pos_vals, pos_dofs)
            self._control_dofs_vel(vel_vals, vel_dofs)
            self._control_dofs_pos(eff_vals, eff_dofs)

    def setup_control_subscriber(self):
        """Initialize the joint trajectory (control) subscriber."""
        gs.logger.info("control command subscriber started")

        def joint_control_callback(msg):
            motor_dofs = get_dofs_idx(self.robot, msg.joint_names)
            dof_idx_table = {}
            for k, motor_dof in enumerate(motor_dofs):
                dof_idx_table[msg.joint_names[k]] = motor_dof
            joint_properties = dict(
                sorted(self.robot_config.get("joint_properties", None).items())
            )
            if self.scene.is_built and not self.dof_properties_set:
                self.set_dofs_properties()
            for point in msg.points:
                self._trajectory_point_controller(
                    point, joint_properties, dof_idx_table
                )

        control_topic = self.robot_config.get(
            "joint_control_topic",
            self.robot_config.get("control_topic", "joint_control"),
        )
        control_sub = self.ros_node.create_subscription(
            JointTrajectory,
            f"{self.namespace}/{control_topic}",
            joint_control_callback,
            10,
        )
        setattr(self, f"{self.namespace}_control_subscriber", control_sub)

    def _control_dofs_pos(self, target_qpos, motor_dofs=None):
        if len(target_qpos) > 0 and len(motor_dofs) > 0:
            if motor_dofs is None:
                motor_dofs = self.motor_dofs
            self.robot.control_dofs_position(target_qpos, motor_dofs)

    def _control_dofs_vel(self, target_qpos, motor_dofs=None):
        if len(target_qpos) > 0 and len(motor_dofs) > 0:
            if motor_dofs is None:
                motor_dofs = self.motor_dofs
            self.robot.control_dofs_velocity(target_qpos, motor_dofs)

    def _control_dofs_eff(self, target_qpos, motor_dofs=None):
        if len(target_qpos) > 0 and len(motor_dofs) > 0:
            if motor_dofs is None:
                motor_dofs = self.motor_dofs
            self.robot.control_dofs_force(target_qpos, motor_dofs)

    def setup_joint_commands_subscriber(self):
        """Initialize the joint command (direct state) subscriber."""
        gs.logger.info("joint commands subscriber started")

        def joint_commands_callback(msg):
            # check and set the joint dof physics properties such as kp.kd, armanture, damping etc
            if self.scene.is_built and not self.dof_properties_set:
                self.set_dofs_properties()
                self.dof_properties_set = True
            motor_dofs = get_dofs_idx(self.robot, msg.name)
            dof_idx_table = {}
            for k, motor_dof in enumerate(motor_dofs):
                dof_idx_table[msg.name[k]] = motor_dof

            valid = True
            joint_properties = dict(
                sorted(self.robot_config.get("joint_properties", None).items())
            )
            pos_i, vel_i, eff_i = 0, 0, 0
            pos_vals, pos_dofs = [], []
            vel_vals, vel_dofs = [], []
            eff_vals, eff_dofs = [], []
            for joint, joint_cfg in joint_properties.items():
                if joint_cfg.get("command", "").lower() == "position":
                    pos_vals.append(msg.position[pos_i])
                    pos_dofs.append(dof_idx_table[joint])
                    pos_i += 1
                elif joint_cfg.get("command", "").lower() == "velocity":
                    vel_vals.append(msg.velocity[vel_i])
                    vel_dofs.append(dof_idx_table[joint])
                    vel_i += 1
                elif joint_cfg.get("command", "").lower() == "effort":
                    eff_vals.append(msg.effort[eff_i])
                    eff_dofs.append(dof_idx_table[joint])
                    eff_i += 1
                else:
                    print("Invalid joint command type")
                    valid = False
            if valid:
                self._control_dofs_pos(pos_vals, pos_dofs)
                self._control_dofs_vel(vel_vals, vel_dofs)
                self._control_dofs_pos(eff_vals, eff_dofs)

        joint_commands_subscriber = self.ros_node.create_subscription(
            JointState,
            f'{self.namespace}/{self.robot_config.get("joint_commands_topic", "joint_commands")}',
            joint_commands_callback,
            int(self.robot_config.get("joint_commands_topic_frequency", 50)),
        )
        setattr(
            self, f"{self.robot}_joint_commands_subscriber", joint_commands_subscriber
        )

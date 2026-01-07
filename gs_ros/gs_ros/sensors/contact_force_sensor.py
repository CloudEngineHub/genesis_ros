import genesis as gs
from geometry_msgs.msg import Vector3
from gs_ros_interfaces.msg import ContactForce
from .base_sensor import BaseSensor
from ..gs_ros_utils import create_qos_profile


class ContactForceSensor(BaseSensor):
    """ROS 2 sensor to measure and publish contact forces acting on a specific robot link."""

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
        """Instantiate the Genesis contact force sensor and configure its ROS 2 publisher."""
        gs.logger.info("contact force Sensor created")

        frequency = self.ros_options.get("frequency", 1.0)
        topic = self.ros_options.get("topic")

        def timer_callback(contact_force_pub, link_name):
            if self.scene.is_built:
                data = contact_force_sensor.read().detach().cpu().tolist()[0]
                contact_msg = ContactForce()
                contact_msg.contact_force = Vector3(x=data[0], y=data[1], z=data[2])
                contact_msg.link_name = link_name
                contact_force_pub.publish(contact_msg)

        entity_idx = self.robot.idx
        link_name = self.rigid_options.get("link")
        link = self.robot.get_link(link_name)
        if link is None:
            raise ValueError(f"Link '{link_name}' not found in entity robot")
        link_idx_local = link.idx_local

        pos_offset = self.rigid_options.get("pos_offset", (0, 0, 0))
        euler_offset = self.rigid_options.get("euler_offset", (0, 0, 0))
        draw_debug = self.general_options.get("draw_debug", False)

        contact_force_sensor = self.scene.add_sensor(
            gs.sensors.ContactForce(
                entity_idx=entity_idx,
                link_idx_local=link_idx_local,
                draw_debug=draw_debug,
                pos_offset=pos_offset,
                euler_offset=euler_offset,
            )
        )

        self.sensor_object = contact_force_sensor

        contact_force_sensor_qos_profile = create_qos_profile(
            self.ros_options.get("qos_history"),
            self.ros_options.get("qos_depth"),
            self.ros_options.get("qos_reliability"),
            self.ros_options.get("qos_durability"),
        )
        contact_force_pub = self.node.create_publisher(
            ContactForce, f"{self.namespace}/{topic}", contact_force_sensor_qos_profile
        )
        self.sensor_publishers = [contact_force_pub]

        timer = self.node.create_timer(
            1 / frequency, lambda: timer_callback(contact_force_pub, link_name)
        )
        setattr(self, f"{self.sensor_name}_contact_timer", timer)
        self.register_sensor(self.sensor_object, self.sensor_publishers)
        return self.sensor_object, self.sensor_publishers

import genesis as gs
from gs_ros_interfaces.msg import Contact
from .base_sensor import BaseSensor
from ..gs_ros_utils import create_qos_profile


class ContactSensor(BaseSensor):
    """ROS 2 sensor to detect and publish contact events for a specific robot link."""

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
        """Instantiate the Genesis contact sensor and configure its ROS 2 publisher."""
        gs.logger.info("contact Sensor created")

        frequency = self.ros_options.get("frequency", 1.0)
        topic = self.ros_options.get("topic")

        def timer_callback(contact_pub, link_name):
            if contact_sensor.is_built:
                data = contact_sensor.read().detach().cpu().tolist()[0][0]
                contact_msg = Contact()
                contact_msg.link_name = link_name
                contact_msg.in_contact = data
                contact_pub.publish(contact_msg)

        entity_idx = self.robot.idx
        link_name = self.rigid_options.get("link")
        link = self.robot.get_link(link_name)
        if link is None:
            raise ValueError(f"Link '{link_name}' not found in entity robot")
        link_idx_local = link.idx_local

        pos_offset = self.rigid_options.get("pos_offset", (0, 0, 0))
        euler_offset = self.rigid_options.get("euler_offset", (0, 0, 0))
        draw_debug = self.general_options.get("draw_debug", False)

        contact_sensor = self.scene.add_sensor(
            gs.sensors.Contact(
                entity_idx=entity_idx,
                link_idx_local=link_idx_local,
                draw_debug=draw_debug,
                pos_offset=pos_offset,
                euler_offset=euler_offset,
            )
        )

        self.sensor_object = contact_sensor

        contact_sensor_qos_profile = create_qos_profile(
            self.ros_options.get("qos_history"),
            self.ros_options.get("qos_depth"),
            self.ros_options.get("qos_reliability"),
            self.ros_options.get("qos_durability"),
        )
        contact_pub = self.node.create_publisher(
            Contact, f"{self.namespace}/{topic}", contact_sensor_qos_profile
        )
        self.sensor_publishers = [contact_pub]

        timer = self.node.create_timer(
            1 / frequency, lambda: timer_callback(contact_pub, link_name)
        )
        setattr(self, f"{self.sensor_name}_contact_timer", timer)
        self.register_sensor(self.sensor_object, self.sensor_publishers)
        return self.sensor_object, self.sensor_publishers

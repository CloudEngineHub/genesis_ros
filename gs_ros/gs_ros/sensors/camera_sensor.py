import genesis as gs
import numpy as np
from sensor_msgs.msg import Image
from genesis.utils.geom import quat_to_R, euler_to_quat
from .base_sensor import BaseSensor
from ..gs_ros_utils import create_qos_profile, get_current_timestamp
from .sensor_helper import add_sensor_noise


class CameraSensor(BaseSensor):
    """Implementation of a ROS 2 camera sensor that renders and publishes images from the simulation."""

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
        """Create the Genesis camera object and setup the image publishing timer."""
        gs.logger.info("Camera Sensor created")

        frame_id = self.ros_options.get("frame_id", "")
        frequency = self.ros_options.get("frequency", 1.0)

        add_noise = self.sensor_config.get("add_noise", False)
        noise_mean = self.sensor_config.get("noise_mean", 0.0)
        noise_std = self.sensor_config.get("noise_std", 0.0)

        def timer_callback(image_publishers, camera_types, link, T, add_noise):
            if self.scene.is_built:
                assert cam._is_built, f"CAMERA with id{cam.id} not Built"
                if cam._attached_link is None:
                    cam.attach(link, T)
                cam.move_to_attach()
                for image_publisher, render_type_str in zip(
                    image_publishers, camera_types
                ):
                    msg = Image()
                    if render_type_str == "rgb":
                        rendered_image = cam.render(rgb=True)
                        rendered_image_idx = 0
                        if add_noise:
                            rendered_image = add_sensor_noise(
                                rendered_image[rendered_image_idx],
                                noise_mean,
                                noise_std,
                            )
                        else:
                            rendered_image = rendered_image[rendered_image_idx]
                        msg = self.bridge.cv2_to_imgmsg(rendered_image, encoding="rgb8")
                        msg.header.frame_id = frame_id
                        msg.header.stamp = get_current_timestamp(
                            self.scene, time_offset=self.time_offset
                        )
                        image_publisher.publish(msg)
                    elif render_type_str == "depth":
                        rendered_image = cam.render(rgb=False, depth=True)
                        rendered_image_idx = 1
                        if add_noise:
                            rendered_image = add_sensor_noise(
                                rendered_image[rendered_image_idx],
                                noise_mean,
                                noise_std,
                            )
                        else:
                            rendered_image = rendered_image[rendered_image_idx]
                        msg = self.bridge.cv2_to_imgmsg(
                            rendered_image, encoding="32FC1"
                        )
                        msg.header.frame_id = frame_id
                        msg.header.stamp = get_current_timestamp(
                            self.scene, time_offset=self.time_offset
                        )
                        image_publisher.publish(msg)
                    elif render_type_str == "segmentation":
                        rendered_image = cam.render(rgb=False, segmentation=True)
                        rendered_image_idx = 2
                        if add_noise:
                            rendered_image = add_sensor_noise(
                                rendered_image[rendered_image_idx].astype(np.int16),
                                noise_mean,
                                noise_std,
                            )
                        else:
                            rendered_image = rendered_image[rendered_image_idx].astype(
                                np.int16
                            )
                        msg = self.bridge.cv2_to_imgmsg(
                            rendered_image, encoding="16SC1"
                        )
                        msg.header.frame_id = frame_id
                        msg.header.stamp = get_current_timestamp(
                            self.scene, time_offset=self.time_offset
                        )
                        image_publisher.publish(msg)
                    elif render_type_str == "normal":
                        rendered_image = cam.render(rgb=False, normal=True)
                        rendered_image_idx = 3
                        if add_noise:
                            rendered_image = add_sensor_noise(
                                rendered_image[rendered_image_idx],
                                noise_mean,
                                noise_std,
                            )
                        else:
                            rendered_image = rendered_image[rendered_image_idx]
                        msg = self.bridge.cv2_to_imgmsg(rendered_image, encoding="rgb8")
                        msg.header.frame_id = frame_id
                        msg.header.stamp = get_current_timestamp(
                            self.scene, time_offset=self.time_offset
                        )
                        image_publisher.publish(msg)

        cam = gs.vis.camera.Camera(
            visualizer=self.scene.visualizer,
            model=self.sensor_config.get("model", "pinhole"),
            res=self.sensor_config.get("res", (320, 320)),
            up=self.sensor_config.get("up", (0.0, 0.0, 1.0)),
            fov=self.sensor_config.get("fov", 30),
            aperture=self.sensor_config.get("aperture", 2.8),
            focus_dist=self.sensor_config.get("focus_dist", None),
            GUI=self.sensor_config.get("gui", False),
            spp=self.sensor_config.get("spp", 256),
            denoise=self.sensor_config.get("denoise", True),
            near=self.sensor_config.get("near", 0.05),
            far=self.sensor_config.get("far", 100.0),
        )

        self.sensor_object = cam

        link_name = self.rigid_options.get("link")
        link = self.robot.get_link(link_name)
        if link is None:
            raise ValueError(f"Link '{link_name}' not found in entity robot")

        pos = self.rigid_options.get("pos_offset")
        euler = self.rigid_options.get("euler_offset")
        if pos is None or euler is None:
            raise ValueError(
                "pos_offset and euler_offset must be provided in rigid_options"
            )

        T = np.eye(4)
        T[:3, :3] = quat_to_R(euler_to_quat(np.array(euler)))
        T[:3, 3] = pos

        self.scene._visualizer._cameras.append(cam)
        qos_profile = create_qos_profile(
            self.ros_options.get("qos_history"),
            self.ros_options.get("qos_depth"),
            self.ros_options.get("qos_reliability"),
            self.ros_options.get("qos_durability"),
        )

        camera_types = self.sensor_config.get("camera_types", ["rgb"])
        for cam_id, camera_type in enumerate(camera_types):
            cam_topic = f"{self.namespace}/{camera_type}"
            pub = self.node.create_publisher(Image, cam_topic, qos_profile)
            self.sensor_publishers.append(pub)

        timer = self.node.create_timer(
            1 / frequency,
            lambda: timer_callback(
                self.sensor_publishers, camera_types, link, T, add_noise
            ),
        )
        setattr(self, f"{self.sensor_name}_cams_timer", timer)
        self.register_sensor(self.sensor_object, self.sensor_publishers)
        return self.sensor_object, self.sensor_publishers

import genesis as gs
from genesis.sensors.raycaster.patterns import DepthCameraPattern, GridPattern, SphericalPattern
from genesis.utils.geom import quat_to_R, euler_to_quat
from sensor_msgs.msg import Image,PointCloud2,Imu,LaserScan
from geometry_msgs.msg import Vector3,Quaternion
from gs_ros_interfaces.msg import Contact,ContactForce
import numpy as np
from rclpy.node import Node

from .gs_ros_sensor_helper import (
    make_cv2_bridge,
    raycaster_to_pcd_msg,
    grid_raycaster_to_pcd_msg,
    raycaster_to_laser_scan_msg,
    add_sensor_noise
)

from .gs_ros_utils import (
    create_qos_profile,
    get_current_timestamp,
    gs_quat_to_ros_quat
)

class GsRosSensors:
    def __init__(self,
                 scene,sim,
                 namespace,robot):
        gs.logger.info("Starting all sensor data publishers")
        self.scene=scene
        self.sim=sim
        self.bridge=make_cv2_bridge()
        self.namespace=namespace
        self.robot=robot
        self.sensors={}
        self.cameras={}
        self.all_ros_nodes=[]
    
    def add_sensor(self,sensor_config):
        sensor_type=sensor_config.get("sensor_type")
        if sensor_type=='cam':
            node=Node(f'CAM_NODE_{self.namespace}_{sensor_config.get("name")}')
            self.all_ros_nodes.append(node)
            sensor_object,sensor_publisher=self.add_camera(sensor_config,node)
            return {"sensor_objects":sensor_object,"sensor_publishers":sensor_publisher}
        elif sensor_type=='rgbd':
            node=Node(f'RGBD_NODE_{self.namespace}_{sensor_config.get("name")}')
            self.all_ros_nodes.append(node)
            sensor_object,sensor_publisher=self.add_rgbd_camera(sensor_config,node)
            return {"sensor_objects":sensor_object,"sensor_publishers":sensor_publisher}
        elif sensor_type=='grid_lidar':
            node=Node(f'GRID_LIDAR_NODE_{self.namespace}_{sensor_config.get("name")}')
            self.all_ros_nodes.append(node)
            sensor_object,sensor_publisher=self.add_grid_lidar(sensor_config,node)
            return {"sensor_objects":sensor_object,"sensor_publishers":sensor_publisher}
        elif sensor_type=='sectional_lidar':
            node=Node(f'SECTIONAL_LIDAR_NODE_{self.namespace}_{sensor_config.get("name")}')
            self.all_ros_nodes.append(node)
            sensor_object,sensor_publisher=self.add_sectional_lidar(sensor_config,node)
            return {"sensor_objects":sensor_object,"sensor_publishers":sensor_publisher}
        elif sensor_type=='3d_lidar':
            node=Node(f'LIDAR_NODE_{self.namespace}_{sensor_config.get("name")}')
            self.all_ros_nodes.append(node)
            sensor_object,sensor_publisher=self.add_lidar(sensor_config,node)
            return {"sensor_objects":sensor_object,"sensor_publishers":sensor_publisher}
        elif sensor_type=='laser_scan':
            node=Node(f'LASER_SCAN_NODE_{self.namespace}_{sensor_config.get("name")}')
            self.all_ros_nodes.append(node)
            sensor_object,sensor_publisher=self.add_laser_scan(sensor_config,node)
            return {"sensor_objects":sensor_object,"sensor_publishers":sensor_publisher}
        elif sensor_type=='imu':
            node=Node(f'IMU_NODE_{self.namespace}_{sensor_config.get("name")}')
            sensor_object,sensor_publisher=self.add_imu(sensor_config,node)
            return {"sensor_objects":sensor_object,"sensor_publishers":sensor_publisher}
        elif sensor_type=='contact_force':
            node=Node(f'CONTACT_FORCE_NODE_{self.namespace}_{sensor_config.get("name")}')
            sensor_object,sensor_publisher=self.add_contact_force_sensor(sensor_config,node)
            return {"sensor_objects":sensor_object,"sensor_publishers":sensor_publisher}
        elif sensor_type=='contact':
            node=Node(f'CONTACT_NODE_{self.namespace}_{sensor_config.get("name")}')
            sensor_object,sensor_publisher=self.add_contact_sensor(sensor_config,node)
            return {"sensor_objects":sensor_object,"sensor_publishers":sensor_publisher}
        else:
            return None
            
    def add_camera(self,cam_config,node):
        gs.logger.info("Camera Sensor created")
        if cam_config is None:
            raise AttributeError
        def timer_callback(image_publishers,camera_types,T,add_noise):
            if self.scene.is_built:
                assert cam._is_built , f"CAMERA with id{cam.id} not Built"
                if cam._attached_link is None:
                    cam.attach(link,T)
                cam.move_to_attach()
                for image_publisher,render_type_str in zip(image_publishers,camera_types):
                    msg=Image()
                    if render_type_str=="rgb":
                        rendered_image=cam.render(rgb=True)
                        rendered_image_idx=0
                        if add_noise:
                            rendered_image=add_sensor_noise(rendered_image[rendered_image_idx],noise_mean,noise_std)
                        else:
                            rendered_image=rendered_image[rendered_image_idx]
                        msg = self.bridge.cv2_to_imgmsg(rendered_image, encoding="rgb8")
                        msg.header.frame_id=cam_config.get("frame_id","")
                        msg.header.stamp=get_current_timestamp(self.scene)
                        image_publisher.publish(msg)
                    elif render_type_str=="depth":
                        rendered_image=cam.render(rgb=False,depth=True)
                        rendered_image_idx=1
                        if add_noise:
                            rendered_image=add_sensor_noise(rendered_image[rendered_image_idx],noise_mean,noise_std)
                        else:
                            rendered_image=rendered_image[rendered_image_idx]
                        msg = self.bridge.cv2_to_imgmsg(rendered_image, encoding="32FC1")
                        msg.header.frame_id=cam_config.get("frame_id","")
                        msg.header.stamp=get_current_timestamp(self.scene)
                        image_publisher.publish(msg)
                    elif render_type_str=="segmentation":
                        rendered_image=cam.render(rgb=False,segmentation=True)
                        rendered_image_idx=2
                        if add_noise:
                            rendered_image=add_sensor_noise(rendered_image[rendered_image_idx].astype(np.int16),noise_mean,noise_std)
                        else:
                            rendered_image=rendered_image[rendered_image_idx]
                        msg = self.bridge.cv2_to_imgmsg(rendered_image, encoding="16SC1")
                        msg.header.frame_id=cam_config.get("frame_id","")
                        msg.header.stamp=get_current_timestamp(self.scene)
                        image_publisher.publish(msg)
                    elif render_type_str=="normal":
                        rendered_image=cam.render(rgb=False,normal=True)
                        rendered_image_idx=3
                        if add_noise:
                            rendered_image=add_sensor_noise(rendered_image[rendered_image_idx],noise_mean,noise_std)
                        else:
                            rendered_image=rendered_image[rendered_image_idx]
                        msg = self.bridge.cv2_to_imgmsg(rendered_image, encoding="rgb8")
                        msg.header.frame_id=cam_config.get("frame_id","")
                        msg.header.stamp=get_current_timestamp(self.scene)
                        image_publisher.publish(msg)
            
        cam = gs.vis.camera.Camera(
            visualizer = self.scene.visualizer, 
            model = cam_config.get("model", 'pinhole'),
            res = cam_config.get("res", (320, 320)),
            # pos = cam_config.get("pos", (0.5, 2.5, 3.5)),
            # lookat = cam_config.get("lookat", (0.5, 0.5, 0.5)),
            up = cam_config.get("up", (0.0, 0.0, 1.0)),
            fov = cam_config.get("fov", 30),
            aperture = cam_config.get("aperture", 2.8),
            focus_dist = cam_config.get("focus_dist", None),
            GUI = cam_config.get("GUI", False),
            spp = cam_config.get("spp", 256),
            denoise = cam_config.get("denoise", True),
            near = cam_config.get("near", 0.05),
            far = cam_config.get("far", 100.0),
            )
        # setattr(self,cam_config.get("name"),cam)
        self.cameras[cam_config.get("name")]=[cam,cam._idx]
        self.sensors[cam_config.get("name")]="CAM"
        add_noise=cam_config.get("add_noise",False)
        noise_mean=cam_config.get("noise_mean",0.0)
        noise_std=cam_config.get("noise_std",0.0)

        link_name = cam_config.get("link")

        if self.robot.get_link(link_name) is None:
            raise ValueError(f"Link '{link_name}' not found in entity robot")
        
        link=self.robot.get_link(link_name)
        
        if cam_config.get('Transform',None) is None:
            pos = cam_config.get("pos", None)
            euler = cam_config.get("euler", None)
            if pos is None or euler is None:
                raise ValueError("either the transformation matrix or the euler must be provided")
            else:                
                T=np.eye(4)
                T[:3,:3]=quat_to_R(euler_to_quat(np.array(euler)))
                T[:3,3]=pos
        else:
            T=np.array(cam_config.get('Transform',None))
            
        self.scene._visualizer._cameras.append(cam)
        qos_profile=create_qos_profile(
            cam_config.get("QOS_history"),
            cam_config.get("QOS_depth"),
            cam_config.get("QOS_reliability"),
            cam_config.get("QOS_durability")
        )
        self.camera_pubs=[]
        camera_types = cam_config.get("camera_types", ["rgb"])
        for cam_id,camera_type in enumerate(camera_types):
            if camera_type in camera_type:
                if cam_config.get('topics') is None or len(cam_config.get('topics'))!=len(camera_types):
                    cam_topic=f"{self.namespace}/{camera_type}_topic"            
                else:
                    cam_topic=f"{self.namespace}/{cam_config.get('topics')[cam_id]}"
                pub= node.create_publisher(Image, cam_topic, qos_profile)
                self.camera_pubs.append(pub)
            elif 'depth' in camera_type:            
                if cam_config.get('topics') is None or len(cam_config.get('topics'))!=len(camera_types):
                    cam_topic=f"{self.namespace}/{camera_type}_topic"            
                else:
                    cam_topic=f"{self.namespace}/{cam_config.get('topics')[cam_id]}"
                pub= node.create_publisher(Image, cam_topic, qos_profile)
                self.camera_pubs.append(pub)
            elif 'segmentation' in camera_type:            
                if cam_config.get('topics') is None or len(cam_config.get('topics'))!=len(camera_types):
                    cam_topic=f"{self.namespace}/{camera_type}_topic"            
                else:
                    cam_topic=f"{self.namespace}/{cam_config.get('topics')[cam_id]}"
                pub= node.create_publisher(Image, cam_topic, qos_profile)
                self.camera_pubs.append(pub)
            elif 'normal' in camera_type:            
                if cam_config.get('topics') is None or len(cam_config.get('topics'))!=len(camera_types):
                    cam_topic=f"{self.namespace}/{camera_type}_topic"            
                else:
                    cam_topic=f"{self.namespace}/{cam_config.get('topics')[cam_id]}"
                pub= node.create_publisher(Image, cam_topic, qos_profile)
                self.camera_pubs.append(pub)
            else:
                raise NotImplementedError(f"Unsupported camera type: {camera_type}")
        timer = node.create_timer(1/cam_config.get('frequency', 1.0),  lambda: timer_callback(self.camera_pubs,camera_types,T,add_noise))
        setattr(self,f'{cam_config.get("name")}_cams_timer',timer)
        return cam,[pub]
        
    def add_rgbd_camera(self, cam_config,node):
        gs.logger.info("RGBD Sensor created")
        rgb_cam_config = cam_config.get("rgb", None)
        sectional_lidar_config = cam_config.get("depth", None)
        
        if rgb_cam_config is None or sectional_lidar_config is None:
            raise AttributeError("RGB or Sectional Lidar configuration missing.")
        
        def timer_callback(rgb_publisher, pcd_publisher,add_noise):
            if self.scene.is_built:
                assert rgb_cam._is_built, f"CAMERA with id {rgb_cam.id} not Built"
                
                if rgb_cam._attached_link is None:
                    rgb_cam.attach(link, T)
                
                rgb_cam.move_to_attach()
                rgb_image = rgb_cam.render(rgb=True)
                if add_noise:
                    rgb_image = add_sensor_noise(rgb_image[0], rgb_noise_mean, rgb_noise_std)
                else:
                    rgb_image=rgb_image[0]

                # Use sectional lidar for depth info (this is the main change)
                pcd_msg = raycaster_to_pcd_msg(sectional_lidar,
                                                stamp=get_current_timestamp(self.scene),
                                                frame_id=cam_config.get("frame_id"),
                                                add_noise=add_noise,
                                                noise_mean=depth_noise_mean,
                                                noise_std=depth_noise_std)
                pcd_publisher.publish(pcd_msg)
                
                rgb_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding="rgb8")
                rgb_msg.header.frame_id = cam_config.get("frame_id")
                rgb_msg.header.stamp = get_current_timestamp(self.scene)
                rgb_publisher.publish(rgb_msg)

        # Initialize RGB camera
        rgb_cam = gs.vis.camera.Camera(
            visualizer=self.scene.visualizer, 
            model=rgb_cam_config.get("model", 'pinhole'),
            res=rgb_cam_config.get("res", (320, 320)),
            up=rgb_cam_config.get("up", (0.0, 0.0, 1.0)),
            fov=rgb_cam_config.get("fov", 30),
            aperture=rgb_cam_config.get("aperture", 2.8),
            focus_dist=rgb_cam_config.get("focus_dist", None),
            GUI=rgb_cam_config.get("GUI", False),
            spp=rgb_cam_config.get("spp", 256),
            denoise=rgb_cam_config.get("denoise", True),
            near=rgb_cam_config.get("near", 0.05),
            far=rgb_cam_config.get("far", 100.0),
        )

        # Setup Sectional Lidar (replacing depth_cam)
        # Directly create the sectional lidar sensor here
        entity_idx = self.robot.idx
        link_name = sectional_lidar_config.get("link", None)
        link = self.robot.get_link(link_name)
        if link is None:
            raise ValueError(f"Link '{link_name}' not found in entity robot")
        link_idx_local = link.idx_local

        near = sectional_lidar_config.get("near", 0.05)
        far = sectional_lidar_config.get("far", 100.0)
        pos_offset = sectional_lidar_config.get("pos_offset", (0, 0, 0))
        euler_offset = sectional_lidar_config.get("euler_offset", (0, 0, 0))

        resolution = sectional_lidar_config.get("resolution", [420, 360])

        fx = sectional_lidar_config.get("fx", None)
        fy = sectional_lidar_config.get("fy", None)

        cx = sectional_lidar_config.get("cx", None)
        cy = sectional_lidar_config.get("cy", None)

        fov_horizontal = sectional_lidar_config.get("fov_horizontal", 90.0)
        fov_vertical = sectional_lidar_config.get("fov_vertical", None)
        draw_points = sectional_lidar_config.get("draw_points", False)

        if draw_points:
            draw_point_radius = sectional_lidar_config.get("draw_point_radius", 0.02)
            ray_start_color = sectional_lidar_config.get("ray_start_color", (0.5, 0.5, 1.0, 1.0))
            ray_hit_color = sectional_lidar_config.get("ray_hit_color", (1.0, 0.5, 0.5, 1.0))
        else:
            draw_point_radius = 0.0
            ray_start_color = (0.0, 0.0, 0.0, 0.0)
            ray_hit_color = (0.0, 0.0, 0.0, 0.0)

        return_points_in_world_frame = sectional_lidar_config.get("return_points_in_world_frame", False)
        try:
            pattern_cfg = DepthCameraPattern(
                res=resolution,
                fx=fx,
                fy=fy,
                cx=cx,
                cy=cy,
                fov_horizontal=fov_horizontal,
                fov_vertical=fov_vertical,
            )
        except TypeError:
            pattern_cfg = DepthCameraPattern(
                Width=resolution[0],
                Height=resolution[1],
                fx=fx,
                fy=fy,
                cx=cx,
                cy=cy,
                fov_horizontal=fov_horizontal,
                fov_vertical=fov_vertical,
            )

        sectional_lidar_options = gs.sensors.Lidar(
            pattern=pattern_cfg,
            entity_idx=entity_idx,
            link_idx_local=link_idx_local,
            return_world_frame=return_points_in_world_frame,
            pos_offset=pos_offset,
            euler_offset=euler_offset,
            min_range=near,
            max_range=far,
            draw_debug=draw_points,
            debug_sphere_radius=draw_point_radius,
            debug_ray_start_color=ray_start_color,
            debug_ray_hit_color=ray_hit_color
        )

        # Add sectional lidar sensor to the scene
        sectional_lidar=self.scene.add_sensor(sectional_lidar_options)

        # Configure noise for the RGB and lidar
        add_noise=rgb_cam_config.get("add_noise",False)
        rgb_noise_mean = rgb_cam_config.get("noise_mean", 0.0)
        rgb_noise_std = rgb_cam_config.get("noise_std", 0.0)
        depth_noise_mean = sectional_lidar_config.get("noise_mean", 0.0)
        depth_noise_std = sectional_lidar_config.get("noise_std", 0.0)

        link_name = cam_config.get("link")
        if self.robot.get_link(link_name) is None:
            raise ValueError(f"Link '{link_name}' not found in entity robot")
        
        link = self.robot.get_link(link_name)

        if cam_config.get('Transform', None) is None:
            pos = cam_config.get("pos", None)
            euler = cam_config.get("euler", None)
            if pos is None or euler is None:
                raise ValueError("either the transformation matrix or the euler must be provided")
            else:
                T = np.eye(4)
                T[:3, :3] = quat_to_R(euler_to_quat(np.array(euler)))
                T[:3, 3] = pos
        else:
            T = np.array(cam_config.get('Transform', None))

        self.scene._visualizer._cameras.append(rgb_cam)
        self.cameras[rgb_cam_config.get("name")] = [rgb_cam, rgb_cam._idx]
        self.sensors[rgb_cam_config.get("name")] = "RGBD"

        rgb_qos_profile = create_qos_profile(
            rgb_cam_config.get("QOS_history"),
            rgb_cam_config.get("QOS_depth"),
            rgb_cam_config.get("QOS_reliability"),
            rgb_cam_config.get("QOS_durability")
        )

        sectional_lidar_qos_profile = create_qos_profile(
            sectional_lidar_config.get("QOS_history"),
            sectional_lidar_config.get("QOS_depth"),
            sectional_lidar_config.get("QOS_reliability"),
            sectional_lidar_config.get("QOS_durability")
        )

        rgb_pub = node.create_publisher(
            Image, 
            f"{self.namespace}/{rgb_cam_config.get('topic')}", 
            rgb_qos_profile
        )
        sectional_lidar_pub = node.create_publisher(
            PointCloud2, 
            f"{self.namespace}/{sectional_lidar_config.get('topic')}", 
            sectional_lidar_qos_profile
        )
        
        setattr(self, f'{rgb_cam_config.get("name")}_rgb_publisher', rgb_pub)
        setattr(self, f'{rgb_cam_config.get("name")}_pcd_publisher', sectional_lidar_pub)

        timer = node.create_timer(
            1 / rgb_cam_config.get('frequency', 1.0), 
            lambda: timer_callback(rgb_pub, sectional_lidar_pub,add_noise)
        )
        setattr(self, f'{rgb_cam_config.get("name")}_rgbd_timer', timer)
        return [rgb_cam, sectional_lidar], [rgb_pub, sectional_lidar_pub]
    
    
    def add_grid_lidar(self,grid_lidar_config,node):
        gs.logger.info("grid_lidar Sensor created")
        def timer_callback(pcd_publisher,add_noise):
            if self.scene.is_built:
                assert grid_lidar._is_built , f"grid liadr sensor is not built"
                self.scene.draw_debug_points(grid_lidar.read()[0].detach().cpu().numpy())
                pcd_msg=grid_raycaster_to_pcd_msg(grid_lidar,
                                                stamp=get_current_timestamp(self.scene),
                                                frame_id=grid_lidar_config.get("frame_id"),
                                                add_noise=add_noise,
                                                noise_mean=noise_mean,
                                                noise_std=noise_std)
                pcd_publisher.publish(pcd_msg)
        
        entity_idx=self.robot.idx
        
        link_name=grid_lidar_config.get("link",None)
        link=self.robot.get_link(link_name)
        if link is None:
            raise ValueError(f"Link '{link_name}' not found in entity robot")
        link_idx_local=link.idx_local
         
        near=grid_lidar_config.get("near",0.05)
        far=grid_lidar_config.get("far",100.0)
        pos_offset=grid_lidar_config.get("pos_offset",(0,0,0))
        euler_offset=grid_lidar_config.get("euler_offset",(0,0,0))

        resolution=grid_lidar_config.get("resolution",0.5)
        grid_size=grid_lidar_config.get("grid_size",(1,1))
        ray_direction_euler=grid_lidar_config.get("ray_direction_euler",(0,0,90))
        draw_points=grid_lidar_config.get("draw_points",False)
        
        if draw_points:
            draw_point_radius=grid_lidar_config.get("draw_point_radius",0.02)
            ray_start_color=grid_lidar_config.get("ray_start_color",(0.5, 0.5, 1.0, 1.0))
            ray_hit_color=grid_lidar_config.get("ray_hit_color",(1.0, 0.5, 0.5, 1.0))
        else:
            draw_point_radius=0.0
            ray_start_color=(0.0, 0.0, 0.0, 0.0)
            ray_hit_color=(0.0, 0.0, 0.0, 0.0)
        return_points_in_world_frame=grid_lidar_config.get("return_points_in_world_frame",False)
        # if draw_points:
        
        pattern_cfg = GridPattern(resolution=resolution,size=grid_size,direction=ray_direction_euler)
        grid_lidar=self.scene.add_sensor(gs.sensors.Lidar(pattern=pattern_cfg,entity_idx=entity_idx,link_idx_local=link_idx_local,return_world_frame=return_points_in_world_frame,
                                                          pos_offset=pos_offset,euler_offset=euler_offset,min_range=near,max_range=far,draw_debug=draw_points,
                                                          debug_sphere_radius=draw_point_radius,debug_ray_start_color=ray_start_color,debug_ray_hit_color=ray_hit_color))
        
        add_noise=grid_lidar_config.get("add_noise",0.0)    
        noise_mean=grid_lidar_config.get("noise_mean",0.0)
        noise_std=grid_lidar_config.get("noise_std",0.0)
        
        self.sensors[grid_lidar_config.get("name")]="SECLIDAR"    
        grid_lidar_qos_profile=create_qos_profile(
            grid_lidar_config.get("QOS_history"),
            grid_lidar_config.get("QOS_depth"),
            grid_lidar_config.get("QOS_reliability"),
            grid_lidar_config.get("QOS_durability")
        )
        grid_lidar_pub= node.create_publisher(PointCloud2, f"{self.namespace}/{grid_lidar_config.get('topic')}", grid_lidar_qos_profile)
        setattr(self,f'{grid_lidar_config.get("name")}_pcd_publisher',grid_lidar_pub)
        timer = node.create_timer(1/grid_lidar_config.get('frequency', 1.0), 
                                           lambda: timer_callback(grid_lidar_pub,add_noise))
        setattr(self,f'{grid_lidar_config.get("name")}_grid_lidar_timer',timer)
        return grid_lidar,grid_lidar_pub 
        
    def add_sectional_lidar(self,sectional_lidar_config,node):
        gs.logger.info("sectional_lidar Sensor created")
        def timer_callback(pcd_publisher,add_noise):
            if self.scene.is_built:
                assert sectional_lidar._is_built , f"sectional lidar sensor is not built"
                    
                pcd_msg=raycaster_to_pcd_msg(sectional_lidar,
                                                        stamp=get_current_timestamp(self.scene),
                                                        frame_id=sectional_lidar_config.get("frame_id"),
                                                        add_noise=add_noise,
                                                        noise_mean=noise_mean,
                                                        noise_std=noise_std)
                pcd_publisher.publish(pcd_msg)
        
        entity_idx=self.robot.idx
        
        link_name=sectional_lidar_config.get("link",None)
        link=self.robot.get_link(link_name)
        if link is None:
            raise ValueError(f"Link '{link_name}' not found in entity robot")
        link_idx_local=link.idx_local
         
        near=sectional_lidar_config.get("near",0.05)
        far=sectional_lidar_config.get("far",100.0)
        pos_offset=sectional_lidar_config.get("pos_offset",(0,0,0))
        euler_offset=sectional_lidar_config.get("euler_offset",(0,0,0))

        resolution=sectional_lidar_config.get("resolution",[420,360])

        fx = sectional_lidar_config.get("fx", None)
        fy = sectional_lidar_config.get("fy", None)

        cx = sectional_lidar_config.get("cx", None)
        cy = sectional_lidar_config.get("cy", None)

        fov_horizontal = sectional_lidar_config.get("fov_horizontal", 90.0)
        fov_vertical = sectional_lidar_config.get("fov_vertical", None)
        draw_points=sectional_lidar_config.get("draw_points",False)
        
        if draw_points:
            draw_point_radius=sectional_lidar_config.get("draw_point_radius",0.02)
            ray_start_color=sectional_lidar_config.get("ray_start_color",(0.5, 0.5, 1.0, 1.0))
            ray_hit_color=sectional_lidar_config.get("ray_hit_color",(1.0, 0.5, 0.5, 1.0))
        else:
            draw_point_radius=0.0
            ray_start_color=(0.0, 0.0, 0.0, 0.0)
            ray_hit_color=(0.0, 0.0, 0.0, 0.0)
        return_points_in_world_frame=sectional_lidar_config.get("return_points_in_world_frame",False)
        
        try:
            pattern_cfg = DepthCameraPattern(
                res=resolution,
                fx=fx,
                fy=fy,
                cx=cx,
                cy=cy,
                fov_horizontal=fov_horizontal,
                fov_vertical=fov_vertical,
            )
        except TypeError:
            pattern_cfg = DepthCameraPattern(
                Width=resolution[0],
                Height=resolution[1],
                fx=fx,
                fy=fy,
                cx=cx,
                cy=cy,
                fov_horizontal=fov_horizontal,
                fov_vertical=fov_vertical,
            )

        sectional_lidar=self.scene.add_sensor(gs.sensors.Lidar(pattern=pattern_cfg,entity_idx=entity_idx,link_idx_local=link_idx_local,return_world_frame=return_points_in_world_frame,
                                                          pos_offset=pos_offset,euler_offset=euler_offset,min_range=near,max_range=far,draw_debug=draw_points,
                                                          debug_sphere_radius=draw_point_radius,debug_ray_start_color=ray_start_color,debug_ray_hit_color=ray_hit_color))
            
        add_noise=sectional_lidar_config.get("add_noise",False)
        noise_mean=sectional_lidar_config.get("noise_mean",0.0)
        noise_std=sectional_lidar_config.get("noise_std",0.0)
        
        self.sensors[sectional_lidar_config.get("name")]="SECLIDAR"    
        sectional_lidar_qos_profile=create_qos_profile(
            sectional_lidar_config.get("QOS_history"),
            sectional_lidar_config.get("QOS_depth"),
            sectional_lidar_config.get("QOS_reliability"),
            sectional_lidar_config.get("QOS_durability")
        )
        sectional_lidar_pub= node.create_publisher(PointCloud2, f"{self.namespace}/{sectional_lidar_config.get('topic')}", sectional_lidar_qos_profile)
        setattr(self,f'{sectional_lidar_config.get("name")}_pcd_publisher',sectional_lidar_pub)
        timer = node.create_timer(1/sectional_lidar_config.get('frequency', 1.0), 
                                           lambda: timer_callback(sectional_lidar_pub,add_noise))
        setattr(self,f'{sectional_lidar_config.get("name")}_sectional_lidar_timer',timer)
        return sectional_lidar,sectional_lidar_pub 
        
    def add_lidar(self,lidar_config,node):
        gs.logger.info("lidar Sensor created")
        def timer_callback(pcd_publisher,add_noise):
            if self.scene.is_built:
                assert lidar._is_built , f"sectional lidar sensor is not built"
                    
                pcd_msg=raycaster_to_pcd_msg(lidar,
                                                stamp=get_current_timestamp(self.scene),
                                                frame_id=lidar_config.get("frame_id"),
                                                add_noise=add_noise,
                                                noise_mean=noise_mean,
                                                noise_std=noise_std)
                pcd_publisher.publish(pcd_msg)
        
        
        # entity_name=lidar_config.get("entity_name",None)
        entity_idx=self.robot.idx
        
        link_name=lidar_config.get("link",None)
        link=self.robot.get_link(link_name)
        if link is None:
            raise ValueError(f"Link '{link_name}' not found in entity robot")
        link_idx_local=link.idx_local
         
        near=lidar_config.get("near",0.05)
        far=lidar_config.get("far",100.0)
        pos_offset=lidar_config.get("pos_offset",(0,0,0))
        euler_offset=lidar_config.get("euler_offset",(0,0,0))

        fov = lidar_config.get("fov", (360.0, 30.0))
        n_points = lidar_config.get("n_points", (64, 128))

        draw_points=lidar_config.get("draw_points",False)
        
        if draw_points:
            draw_point_radius=lidar_config.get("draw_point_radius",0.02)
            ray_start_color=lidar_config.get("ray_start_color",(0.5, 0.5, 1.0, 1.0))
            ray_hit_color=lidar_config.get("ray_hit_color",(1.0, 0.5, 0.5, 1.0))
        else:
            draw_point_radius=0.0
            ray_start_color=(0.0, 0.0, 0.0, 0.0)
            ray_hit_color=(0.0, 0.0, 0.0, 0.0)
        return_points_in_world_frame=lidar_config.get("return_points_in_world_frame",False)
        # if draw_points:
        
        pattern_cfg = SphericalPattern(
            fov=fov,
            n_points=n_points,
        )

        lidar=self.scene.add_sensor(gs.sensors.Lidar(pattern=pattern_cfg,entity_idx=entity_idx,link_idx_local=link_idx_local,return_world_frame=return_points_in_world_frame,
                                                          pos_offset=pos_offset,euler_offset=euler_offset,min_range=near,max_range=far,draw_debug=draw_points,
                                                          debug_sphere_radius=draw_point_radius,debug_ray_start_color=ray_start_color,debug_ray_hit_color=ray_hit_color))
        
        add_noise=lidar_config.get("add_noise",False)    
        noise_mean=lidar_config.get("noise_mean",0.0)
        noise_std=lidar_config.get("noise_std",0.0)
        
        self.sensors[lidar_config.get("name")]="LIDAR"    
        lidar_qos_profile=create_qos_profile(
            lidar_config.get("QOS_history"),
            lidar_config.get("QOS_depth"),
            lidar_config.get("QOS_reliability"),
            lidar_config.get("QOS_durability")
        )
        lidar_pub= node.create_publisher(PointCloud2, f"{self.namespace}/{lidar_config.get('topic')}", lidar_qos_profile)
        setattr(self,f'{lidar_config.get("name")}_pcd_publisher',lidar_pub)
        timer = node.create_timer(1/lidar_config.get('frequency', 1.0), 
                                           lambda: timer_callback(lidar_pub,add_noise))
        setattr(self,f'{lidar_config.get("name")}_lidar_timer',timer)
        return lidar,lidar_pub 
               
    def add_laser_scan(self,laser_scan_config,node):
        gs.logger.info("laser_scan Sensor created")
        def timer_callback(laser_scan_publisher,angle_min,angle_max,resolution,near,far,add_noise):
            if self.scene.is_built:
                assert laser_scan._is_built , f"laser_scan sensor is not built"
                    
                laser_scan_msg=raycaster_to_laser_scan_msg(laser_scan,
                                                        stamp=get_current_timestamp(self.scene),
                                                        frame_id=laser_scan_config.get("frame_id"),
                                                        angle_min=angle_min,angle_max=angle_max,
                                                        resolution=resolution,near=near,far=far,
                                                        add_noise=add_noise,
                                                        noise_mean=noise_mean,
                                                        noise_std=noise_std)
                laser_scan_publisher.publish(laser_scan_msg)
        
        
        # entity_name=laser_scan_config.get("entity_name",None)
        entity_idx=self.robot.idx
        
        link_name=laser_scan_config.get("link",None)
        link=self.robot.get_link(link_name)
        if link is None:
            raise ValueError(f"Link '{link_name}' not found in entity robot")
        link_idx_local=link.idx_local
         
        near=laser_scan_config.get("near",0.05)
        far=laser_scan_config.get("far",100.0)
        pos_offset=laser_scan_config.get("pos_offset",(0,0,0))
        euler_offset=laser_scan_config.get("euler_offset",(0,0,0))

        fov = laser_scan_config.get("fov", (360.0, 1.0))
        resolution=laser_scan_config.get("resolution", 720)
        n_points = (resolution,1)
        

        draw_points=laser_scan_config.get("draw_points",False)
        
        if draw_points:
            draw_point_radius=laser_scan_config.get("draw_point_radius",0.02)
            ray_start_color=laser_scan_config.get("ray_start_color",(0.5, 0.5, 1.0, 1.0))
            ray_hit_color=laser_scan_config.get("ray_hit_color",(1.0, 0.5, 0.5, 1.0))
        else:
            draw_point_radius=0.0
            ray_start_color=(0.0, 0.0, 0.0, 0.0)
            ray_hit_color=(0.0, 0.0, 0.0, 0.0)
        return_points_in_world_frame=laser_scan_config.get("return_points_in_world_frame",False)
        # if draw_points:
        
        pattern_cfg = SphericalPattern(
            fov=fov,
            n_points=n_points,
        )

        laser_scan=self.scene.add_sensor(gs.sensors.Lidar(pattern=pattern_cfg,entity_idx=entity_idx,link_idx_local=link_idx_local,return_world_frame=return_points_in_world_frame,
                                                          pos_offset=pos_offset,euler_offset=euler_offset,min_range=near,max_range=far,draw_debug=draw_points,
                                                          debug_sphere_radius=draw_point_radius,debug_ray_start_color=ray_start_color,debug_ray_hit_color=ray_hit_color))
        
        add_noise=laser_scan_config.get("add_noise",False)    
        noise_mean=laser_scan_config.get("noise_mean",0.0)
        noise_std=laser_scan_config.get("noise_std",0.0)
        
        self.sensors[laser_scan_config.get("name")]="LIDAR"    
        laser_scan_qos_profile=create_qos_profile(
            laser_scan_config.get("QOS_history"),
            laser_scan_config.get("QOS_depth"),
            laser_scan_config.get("QOS_reliability"),
            laser_scan_config.get("QOS_durability")
        )
        laser_scan_pub=node.create_publisher(LaserScan,f"{self.namespace}/{laser_scan_config.get('topic')}",laser_scan_qos_profile)
        setattr(self,f'{laser_scan_config.get("name")}_pcd_publisher',laser_scan_pub)
        timer = node.create_timer(1/laser_scan_config.get('frequency', 1.0), 
                                           lambda: timer_callback(laser_scan_pub,-fov[0]/2,fov[0]/2,resolution,near,far,add_noise))
        setattr(self,f'{laser_scan_config.get("name")}_laser_scan_timer',timer)
        return laser_scan,laser_scan_pub

    def add_imu(self,imu_config,node):
        gs.logger.info("imu Sensor created")
        def timer_callback(imu_pub,robot):
            if self.scene.is_built:
                assert imu._is_built , f"imu sensor is not built"
                data = imu.read()
                imu_msg=Imu()
                imu_msg.header.frame_id=imu_config.get("frame_id")
                imu_msg.header.stamp=get_current_timestamp(self.scene)
                quat=gs_quat_to_ros_quat(robot.get_quat().detach().cpu().numpy().tolist()[0])
                imu_msg.orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
                ang_vel=data.ang_vel.detach().cpu().numpy().tolist()[0]
                imu_msg.angular_velocity=Vector3(x=ang_vel[0],y=ang_vel[1],z=ang_vel[2])
                lin_acc=data.lin_acc.detach().cpu().numpy().tolist()[0]
                imu_msg.linear_acceleration=Vector3(x=lin_acc[0],y=lin_acc[1],z=lin_acc[2])
                imu_pub.publish(imu_msg)
        
        self.sensors[imu_config.get("name")]="IMU"
        entity_idx=self.robot.idx
        
        link_name=imu_config.get("link",None)
        link=self.robot.get_link(link_name)
        if link is None:
            raise ValueError(f"Link '{link_name}' not found in entity robot")
        link_idx_local=link.idx_local

        pos_offset=imu_config.get("pos_offset",(0,0,0))
        euler_offset=imu_config.get("euler_offset",(0,0,0))
        draw_debug=imu_config.get("draw_debug",False)

        imu=self.scene.add_sensor(
            gs.sensors.IMU(
                entity_idx=entity_idx,
                link_idx_local=link_idx_local,
                pos_offset=pos_offset,
                euler_offset=euler_offset,
                draw_debug=draw_debug,
                acc_resolution = imu_config.get("acc_resolution", 0.0),
                acc_axes_skew = imu_config.get("acc_axes_skew", 0.0),
                acc_bias = imu_config.get("acc_bias", (0.0, 0.0, 0.0)),
                acc_noise = imu_config.get("acc_noise", (0.0, 0.0, 0.0)),
                acc_random_walk = imu_config.get("acc_random_walk", (0.0, 0.0, 0.0)),

                gyro_resolution = imu_config.get("gyro_resolution", 0.0),
                gyro_axes_skew = imu_config.get("gyro_axes_skew", 0.0),
                gyro_bias = imu_config.get("gyro_bias", (0.0, 0.0, 0.0)),
                gyro_noise = imu_config.get("gyro_noise", (0.0, 0.0, 0.0)),
                gyro_random_walk = imu_config.get("gyro_random_walk", (0.0, 0.0, 0.0)),

                debug_acc_color = imu_config.get("debug_acc_color", (0.0, 1.0, 1.0, 0.5)),
                debug_acc_scale = imu_config.get("debug_acc_scale", 0.01),
                debug_gyro_color = imu_config.get("debug_gyro_color", (1.0, 1.0, 0.0, 0.5)),
                debug_gyro_scale = imu_config.get("debug_gyro_scale", 0.01),
            )
        )
            
        imu_qos_profile=create_qos_profile(
            imu_config.get("QOS_history"),
            imu_config.get("QOS_depth"),
            imu_config.get("QOS_reliability"),
            imu_config.get("QOS_durability")
        )
        imu_pub=node.create_publisher(Imu,f'{self.namespace}/{imu_config.get("topic")}',imu_qos_profile)
        setattr(self,f'{imu_config.get("name")}_imu_publisher',imu_pub)
        timer = node.create_timer(1/imu_config.get('frequency', 1.0), lambda: timer_callback(imu_pub,self.robot))
        setattr(self,f'{imu_config.get("name")}_imu_timer',timer)
        return imu,imu_pub  
        
    def add_contact_force_sensor(self,contact_force_sensor_config,node):
        gs.logger.info("contact force Sensor created")
        def timer_callback(contact_force_pub,link_name):
            if self.scene.is_built:
                data=contact_force_sensor.read().detach().cpu().tolist()[0]
                contact_msg=ContactForce()
                contact_msg.contact_force=Vector3(x=data[0],y=data[1],z=data[2])
                contact_msg.link_name=link_name
                contact_force_pub.publish(contact_msg)
        self.sensors[contact_force_sensor_config.get("name")]="CONTACT_FORCE"
        entity_idx=self.robot.idx

        link_name=contact_force_sensor_config.get("link",None)
        link=self.robot.get_link(link_name)
        if link is None:
            raise ValueError(f"Link '{link_name}' not found in entity robot")
        link_idx_local=link.idx_local

        pos_offset=contact_force_sensor_config.get("pos_offset",(0,0,0))
        euler_offset=contact_force_sensor_config.get("euler_offset",(0,0,0))
        draw_debug=contact_force_sensor_config.get("draw_debug",False)
        
        contact_force_sensor_options = gs.sensors.ContactForce(
                entity_idx=entity_idx,
                link_idx_local=link_idx_local,
                draw_debug=draw_debug,
                pos_offset=pos_offset,
                euler_offset=euler_offset,
            )
        contact_force_sensor=self.scene.add_sensor(
            contact_force_sensor_options
            )
        contact_force_sensor_qos_profile=create_qos_profile(
            contact_force_sensor_config.get("QOS_history"),
            contact_force_sensor_config.get("QOS_depth"),
            contact_force_sensor_config.get("QOS_reliability"),
            contact_force_sensor_config.get("QOS_durability")
        )
        topic=contact_force_sensor_config.get("topic")
        contact_force_pub=node.create_publisher(ContactForce,f"{self.namespace}/{topic}",contact_force_sensor_qos_profile)
        setattr(self,f'{contact_force_sensor_config.get("name")}_contact_publisher',contact_force_pub)
        timer=node.create_timer(1/contact_force_sensor_config.get('frequency', 1.0),lambda:timer_callback(contact_force_pub,link_name))
        setattr(self,f'{contact_force_sensor_config.get("name")}_contact_timer',timer)
        return contact_force_sensor,contact_force_pub

    def add_contact_sensor(self,contact_sensor_config,node):
        gs.logger.info("contact Sensor created")
        def timer_callback(contact_pub,link_name):
            if contact_sensor.is_built:
                data=contact_sensor.read().detach().cpu().tolist()[0][0]
                contact_msg=Contact()
                contact_msg.link_name=link_name
                contact_msg.in_contact=data
                contact_pub.publish(contact_msg)
        self.sensors[contact_sensor_config.get("name")]="CONTACT"
        entity_idx=self.robot.idx

        link_name=contact_sensor_config.get("link",None)
        link=self.robot.get_link(link_name)
        if link is None:
            raise ValueError(f"Link '{link_name}' not found in entity robot")
        link_idx_local=link.idx_local

        pos_offset=contact_sensor_config.get("pos_offset",(0,0,0))
        euler_offset=contact_sensor_config.get("euler_offset",(0,0,0))
        draw_debug=contact_sensor_config.get("draw_debug",False)
        
        contact_sensor_options = gs.sensors.Contact(
                entity_idx=entity_idx,
                link_idx_local=link_idx_local,
                draw_debug=draw_debug,
                pos_offset=pos_offset,
                euler_offset=euler_offset,
            )
        contact_sensor=self.scene.add_sensor(
            contact_sensor_options
            )
        contact_sensor_qos_profile=create_qos_profile(
            contact_sensor_config.get("QOS_history"),
            contact_sensor_config.get("QOS_depth"),
            contact_sensor_config.get("QOS_reliability"),
            contact_sensor_config.get("QOS_durability")
        )
        topic=contact_sensor_config.get("topic")
        contact_pub=node.create_publisher(Contact,f"{self.namespace}/{topic}",contact_sensor_qos_profile)
        setattr(self,f'{contact_sensor_config.get("name")}_contact_publisher',contact_pub)
        timer=node.create_timer(1/contact_sensor_config.get('frequency', 1.0),lambda:timer_callback(contact_pub,link_name))
        setattr(self,f'{contact_sensor_config.get("name")}_contact_timer',timer)
        return contact_sensor,contact_pub

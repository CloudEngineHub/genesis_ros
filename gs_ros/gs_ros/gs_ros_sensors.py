import genesis as gs
from sensor_msgs.msg import Image,PointCloud2,Imu,LaserScan
from geometry_msgs.msg import Vector3,Quaternion
from gs_ros_interfaces.msg import Contacts,Contact
import numpy as np
import math
from .gs_ros_sensor_helper import (
    make_cv2_bridge,
    lidar_cams_to_pcd_msg,
    sectional_lidar_cam_to_pcd_msg,
    lidar_cams_to_laser_scan_msg,
    add_noise)

from .gs_ros_utils import (
    create_qos_profile,
    get_entity,
    get_current_timestamp,
    get_camera_transformation_matrices,
    gs_quat_to_ros_quat
)

class Gs_Ros_Sensors:
    def __init__(self,
                 scene,sim,
                 ros_node,
                 namespace,robot,
                 robots,objects):
        self.scene=scene
        self.sim=sim
        self.ros_node=ros_node
        self.bridge=make_cv2_bridge()
        self.namespace=namespace
        self.robots=robots
        self.objects=objects
        self.robot=robot
        self.sensors={}
        self.cameras={}
    
    def add_sensor(self,sensor_config):
        sensor_type=sensor_config.get("sensor_type")
        if sensor_type=='cam':
            self.add_camera(sensor_config)
            return sensor_type
        elif sensor_type=='rgbd':
            self.add_rgbd_camera(sensor_config)
            return sensor_type
        elif sensor_type=='sectional_lidar':
            self.add_sectional_lidar(sensor_config)
            return sensor_type
        elif sensor_type=='3d_lidar':
            self.add_lidar(sensor_config)
            return sensor_type
        elif sensor_type=='laser_scan':
            self.add_laser_scan(sensor_config)
            return sensor_type
        elif sensor_type=='imu':
            self.add_imu(sensor_config)
            return sensor_type
        elif sensor_type=='contact':
            self.add_contact_sensor(sensor_config)
            return sensor_type
        else:
            return None
            
    def add_camera(self,cam_config):
        if cam_config is None:
            raise AttributeError
        def timer_callback(image_publishers,camera_types):
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
                        rendered_image=add_noise(rendered_image[rendered_image_idx],noise_mean,noise_std)
                        msg = self.bridge.cv2_to_imgmsg(rendered_image, encoding="rgb8")
                        msg.header.frame_id=cam_config.get("frame_id","")
                        msg.header.stamp=get_current_timestamp(self.scene)
                        image_publisher.publish(msg)
                    elif render_type_str=="depth":
                        rendered_image=cam.render(rgb=False,depth=True)
                        rendered_image_idx=1
                        rendered_image=add_noise(rendered_image[rendered_image_idx],noise_mean,noise_std)
                        msg = self.bridge.cv2_to_imgmsg(rendered_image, encoding="32FC1")
                        msg.header.frame_id=cam_config.get("frame_id","")
                        msg.header.stamp=get_current_timestamp(self.scene)
                        image_publisher.publish(msg)
                    elif render_type_str=="segmentation":
                        rendered_image=cam.render(rgb=False,segmentation=True)
                        rendered_image_idx=2
                        rendered_image=add_noise(rendered_image[rendered_image_idx].astype(np.int16),noise_mean,noise_std)
                        msg = self.bridge.cv2_to_imgmsg(rendered_image, encoding="16SC1")
                        msg.header.frame_id=cam_config.get("frame_id","")
                        msg.header.stamp=get_current_timestamp(self.scene)
                        image_publisher.publish(msg)
                    elif render_type_str=="normal":
                        rendered_image=cam.render(rgb=False,normal=True)
                        rendered_image_idx=3
                        rendered_image=add_noise(rendered_image[rendered_image_idx],noise_mean,noise_std)
                        msg = self.bridge.cv2_to_imgmsg(rendered_image, encoding="rgb8")
                        msg.header.frame_id=cam_config.get("frame_id","")
                        msg.header.stamp=get_current_timestamp(self.scene)
                        image_publisher.publish(msg)
            
        cam = gs.vis.camera.Camera(
            visualizer = self.scene.visualizer, 
            model = cam_config.get("model", 'pinhole'),
            res = cam_config.get("res", (320, 320)),
            pos = cam_config.get("pos", (0.5, 2.5, 3.5)),
            lookat = cam_config.get("lookat", (0.5, 0.5, 0.5)),
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
        noise_mean=cam_config.get("noise_mean",0.0)
        noise_std=cam_config.get("noise_std",0.0)

        link_name = cam_config.get("link")

        if self.robot.get_link(link_name) is None:
            raise ValueError(f"Link '{link_name}' not found in entity robot")
        
        link=self.robot.get_link(link_name)
        
        if cam_config.get('Transform',None) is not None:
            T=np.array(cam_config.get('Transform',None))
        self.scene._visualizer._cameras.append(cam)
        # rgb=True, depth=False, segmentation=False, colorize_seg=False, normal=False
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
                pub= self.ros_node.create_publisher(Image, cam_topic, qos_profile)
                self.camera_pubs.append(pub)
            elif 'depth' in camera_type:            
                if cam_config.get('topics') is None or len(cam_config.get('topics'))!=len(camera_types):
                    cam_topic=f"{self.namespace}/{camera_type}_topic"            
                else:
                    cam_topic=f"{self.namespace}/{cam_config.get('topics')[cam_id]}"
                pub= self.ros_node.create_publisher(Image, cam_topic, qos_profile)
                self.camera_pubs.append(pub)
            elif 'segmentation' in camera_type:            
                if cam_config.get('topics') is None or len(cam_config.get('topics'))!=len(camera_types):
                    cam_topic=f"{self.namespace}/{camera_type}_topic"            
                else:
                    cam_topic=f"{self.namespace}/{cam_config.get('topics')[cam_id]}"
                pub= self.ros_node.create_publisher(Image, cam_topic, qos_profile)
                self.camera_pubs.append(pub)
            elif 'normal' in camera_type:            
                if cam_config.get('topics') is None or len(cam_config.get('topics'))!=len(camera_types):
                    cam_topic=f"{self.namespace}/{camera_type}_topic"            
                else:
                    cam_topic=f"{self.namespace}/{cam_config.get('topics')[cam_id]}"
                pub= self.ros_node.create_publisher(Image, cam_topic, qos_profile)
                self.camera_pubs.append(pub)
            else:
                raise NotImplementedError(f"Unsupported camera type: {camera_type}")
        timer = self.ros_node.create_timer(1/cam_config.get('frequency', 1.0),  lambda: timer_callback(self.camera_pubs,camera_types))
        setattr(self,f'{cam_config.get("name")}_cams_timer',timer)
        
    def add_rgbd_camera(self,cam_config):
        rgb_cam_config=cam_config.get("rgb",None)
        depth_cam_config=cam_config.get("depth",None)
        if rgb_cam_config is None or depth_cam_config is None:
            raise AttributeError
        def timer_callback(rgb_publisher,pcd_publisher,
                           T_MAT_X,T_MATS_Z,T_MAT_TILT,
                           near, far):
            if self.scene.is_built:
                assert rgb_cam._is_built , f"CAMERA with id{rgb_cam.id} not Built"
                assert depth_cam._is_built , f"CAMERA with id{rgb_cam.id} not Built"
                if rgb_cam._attached_link is None:
                    rgb_cam.attach(link,T)
                if depth_cam._attached_link is None:
                    depth_cam.attach(link,T)
                rgb_cam.move_to_attach()
                depth_cam.move_to_attach()
                rgb_image=rgb_cam.render(rgb=True)
                rgb_image=add_noise(rgb_image[0],rgb_noise_mean, rgb_noise_std)
                pcd_msg= sectional_lidar_cam_to_pcd_msg([depth_cam],T_MAT_X,T_MATS_Z,T_MAT_TILT,
                                                                        stamp=get_current_timestamp(self.scene),
                                                                        frame_id=cam_config.get("frame_id"),
                                                                        tilt_angle=tilt_angle,
                                                                        noise_mean=depth_noise_mean,
                                                                        noise_std=depth_noise_std,
                                                                        near=near,far=far)
                pcd_publisher.publish(pcd_msg)
                rgb_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding="rgb8")
                rgb_msg.header.frame_id=cam_config.get("frame_id")
                rgb_msg.header.stamp=get_current_timestamp(self.scene)
                rgb_publisher.publish(rgb_msg)
            
        rgb_cam = gs.vis.camera.Camera(
            visualizer = self.scene.visualizer, 
            model = rgb_cam_config.get("model", 'pinhole'),
            res = rgb_cam_config.get("res", (320, 320)),
            pos = rgb_cam_config.get("pos", (0.5, 2.5, 3.5)),
            lookat = rgb_cam_config.get("lookat", (0.5, 0.5, 0.5)),
            up = rgb_cam_config.get("up", (0.0, 0.0, 1.0)),
            fov = rgb_cam_config.get("fov", 30),
            aperture = rgb_cam_config.get("aperture", 2.8),
            focus_dist = rgb_cam_config.get("focus_dist", None),
            GUI = rgb_cam_config.get("GUI", False),
            spp = rgb_cam_config.get("spp", 256),
            denoise = rgb_cam_config.get("denoise", True),
            near = rgb_cam_config.get("near", 0.05),
            far = rgb_cam_config.get("far", 100.0),
            )
        depth_near = depth_cam_config.get("near", 0.05)
        depth_far = depth_cam_config.get("far", 100.0)
        depth_cam=gs.vis.camera.Camera(
            visualizer = self.scene.visualizer,
            model = depth_cam_config.get("model", 'pinhole'),
            res = depth_cam_config.get("res", (320, 320)),
            pos = depth_cam_config.get("pos", (0.5, 2.5, 3.5)),
            lookat = depth_cam_config.get("lookat", (0.5, 0.5, 0.5)),
            up = depth_cam_config.get("up", (0.0, 0.0, 1.0)),
            fov = depth_cam_config.get("vfov", 30),
            aperture = depth_cam_config.get("aperture", 2.8),
            focus_dist = depth_cam_config.get("focus_dist", None),
            GUI = depth_cam_config.get("GUI", False),
            spp = depth_cam_config.get("spp", 256),
            denoise = depth_cam_config.get("denoise", True),
            near = depth_near,
            far = depth_far
            )
        tilt_angle=cam_config.get("tilt_angle")
        rgb_noise_mean=rgb_cam_config.get("noise_mean",0.0)
        rgb_noise_std=rgb_cam_config.get("noise_std",0.0)
        depth_noise_mean=depth_cam_config.get("noise_mean",0.0)
        depth_noise_std=depth_cam_config.get("noise_std",0.0)
        
        link_name = cam_config.get("link")

        if self.robot.get_link(link_name) is None:
            raise ValueError(f"Link '{link_name}' not found in entity robot")
        
        link=self.robot.get_link(link_name)
        
        if cam_config.get('Transform',None) is not None:
            T=np.array(cam_config.get('Transform',None))
            
        self.scene._visualizer._cameras.append(depth_cam)
        self.scene._visualizer._cameras.append(rgb_cam)
        T_MAT_X,T_MATS_Z,T_MAT_TILT=get_camera_transformation_matrices(n_cameras=1,
                                                                h_fov=None,
                                                                tilt_angle=tilt_angle)
        self.cameras[rgb_cam_config.get("name")]=[rgb_cam,rgb_cam._idx]
        self.cameras[depth_cam_config.get("name")]=[depth_cam,depth_cam._idx]
        self.sensors[rgb_cam_config.get("name")]="RGBD"
        
        rgb_qos_profile=create_qos_profile(
            rgb_cam_config.get("QOS_history"),
            rgb_cam_config.get("QOS_depth"),
            rgb_cam_config.get("QOS_reliability"),
            rgb_cam_config.get("QOS_durability")
        )
        depth_qos_profile=create_qos_profile(
            depth_cam_config.get("QOS_history"),
            depth_cam_config.get("QOS_depth"),
            depth_cam_config.get("QOS_reliability"),
            depth_cam_config.get("QOS_durability")
        )    
        rgb_pub= self.ros_node.create_publisher(Image, f"{self.namespace}/{rgb_cam_config.get('topic')}", rgb_qos_profile)
        depth_pub= self.ros_node.create_publisher(PointCloud2, f"{self.namespace}/{depth_cam_config.get('topic')}", depth_qos_profile)
        setattr(self,f'{rgb_cam_config.get("name")}_rgb_publisher',rgb_pub)
        setattr(self,f'{rgb_cam_config.get("name")}_pcd_publisher',depth_pub)
        timer = self.ros_node.create_timer(1/rgb_cam_config.get('frequency', 1.0), 
                                           lambda: timer_callback(rgb_pub,depth_pub,T_MAT_X,T_MATS_Z,T_MAT_TILT,depth_near,depth_far))
        setattr(self,f'{rgb_cam_config.get("name")}_rgbd_timer',timer)    
    
    def add_sectional_lidar(self,sectional_lidar_config):
        def timer_callback(pcd_publisher,T_MAT_X,T_MATS_Z,T_MAT_TILT,near,far):
            if self.scene.is_built:
                assert sectional_lidar._is_built , f"CAMERA with id{sectional_lidar.id} not Built"
                if sectional_lidar._attached_link is None:
                    sectional_lidar.attach(link,T)
                sectional_lidar.move_to_attach()
                    
                pcd_msg=sectional_lidar_cam_to_pcd_msg([sectional_lidar],T_MAT_X,T_MATS_Z,T_MAT_TILT,
                                                                        stamp=get_current_timestamp(self.scene),
                                                                        frame_id=sectional_lidar_config.get("frame_id"),
                                                                        tilt_angle=tilt_angle,
                                                                        noise_mean=noise_mean,
                                                                        noise_std=noise_std,
                                                                        near=near,far=far)
                pcd_publisher.publish(pcd_msg)
        
        near=sectional_lidar_config.get("near",0.05)
        far=sectional_lidar_config.get("far",100.0)
        sectional_lidar=gs.vis.camera.Camera(
            visualizer = self.scene.visualizer,
            model = sectional_lidar_config.get("model", 'pinhole'),
            res = sectional_lidar_config.get("res", (320, 320)),
            pos = sectional_lidar_config.get("pos", (0.5, 2.5, 3.5)),
            lookat = sectional_lidar_config.get("lookat", (0.5, 0.5, 0.5)),
            up = sectional_lidar_config.get("up", (0.0, 0.0, 1.0)),
            fov = sectional_lidar_config.get("fov", 30),
            aperture = sectional_lidar_config.get("aperture", 2.8),
            focus_dist = sectional_lidar_config.get("focus_dist", None),
            GUI = sectional_lidar_config.get("GUI", False),
            spp = sectional_lidar_config.get("spp", 256),
            denoise = sectional_lidar_config.get("denoise", True),
            near = near,
            far = far,
            transform = sectional_lidar_config.get("transform", None))
        tilt_angle=sectional_lidar_config.get("tilt_angle")
        
        link_name = sectional_lidar_config.get("link")

        if self.robot.get_link(link_name) is None:
            raise ValueError(f"Link '{link_name}' not found in entity robot")
        
        link=self.robot.get_link(link_name)
        
        if sectional_lidar_config.get('Transform',None) is not None:
            T=np.array(sectional_lidar_config.get('Transform',None))
            
        self.scene._visualizer._cameras.append(sectional_lidar)
        noise_mean=sectional_lidar_config.get("noise_mean",0.0)
        noise_std=sectional_lidar_config.get("noise_std",0.0)
        T_MAT_X,T_MATS_Z,T_MAT_TILT=get_camera_transformation_matrices(n_cameras=1,
                                                        h_fov=None,
                                                        tilt_angle=tilt_angle)
        self.cameras[sectional_lidar_config.get("name")]=[sectional_lidar,sectional_lidar._idx]
        self.sensors[sectional_lidar_config.get("name")]="SECLIDAR"    
        sectional_lidar_qos_profile=create_qos_profile(
            sectional_lidar_config.get("QOS_history"),
            sectional_lidar_config.get("QOS_depth"),
            sectional_lidar_config.get("QOS_reliability"),
            sectional_lidar_config.get("QOS_durability")
        )
        sectional_lidar_pub= self.ros_node.create_publisher(PointCloud2, f"{self.namespace}/{sectional_lidar_config.get('topic')}", sectional_lidar_qos_profile)
        setattr(self,f'{sectional_lidar_config.get("name")}_pcd_publisher',sectional_lidar_pub)
        timer = self.ros_node.create_timer(1/sectional_lidar_config.get('frequency', 1.0), 
                                           lambda: timer_callback(sectional_lidar_pub,T_MAT_X,T_MATS_Z,T_MAT_TILT,near,far))
        setattr(self,f'{sectional_lidar_config.get("name")}_rgbd_timer',timer) 
            
    def add_lidar(self,lidar_config):
        def timer_callback(pcd_pub,lidar_cams,
                           T_MAT_X,T_MATS_Z,T_MAT_TILT,
                           near,far):
            if self.scene.is_built:
                pcd_msg=lidar_cams_to_pcd_msg(lidar_cams,link,T,T_MAT_X,T_MATS_Z,T_MAT_TILT,
                                                    stamp=get_current_timestamp(self.scene),
                                                    frame_id=lidar_config.get("frame_id"),
                                                    hfov=total_h_fov_angle_deg,
                                                    near=near,far=far,
                                                    noise_mean=noise_mean,
                                                    noise_std=noise_std,)
                pcd_pub.publish(pcd_msg)
        
        tilt_angle=lidar_config.get("tilt_angle")
        h_min_angle=abs(lidar_config.get("horizontal_min_angle"))
        h_max_angle=lidar_config.get("horizontal_max_angle")
        total_h_fov_angle=h_min_angle+h_max_angle
        total_h_fov_angle_deg=np.rad2deg(total_h_fov_angle)
        n_cam=math.ceil(total_h_fov_angle_deg/22.5)
        h_sample=lidar_config.get("h_samples")
        v_samples=lidar_config.get("v_samples")
        per_cam_resolution=(h_sample//n_cam,v_samples)
        model="pinhole"
        near=lidar_config.get("near")
        far=lidar_config.get("far")
        
        noise_mean=lidar_config.get("noise_mean",0.0)
        noise_std=lidar_config.get("noise_std",0.0)
        
        link_name = lidar_config.get("link")
        if self.robot.get_link(link_name) is None:
            raise ValueError(f"Link '{link_name}' not found in entity robot")
        
        link=self.robot.get_link(link_name)
        if lidar_config.get('Transform',None) is not None:
            T=np.array(lidar_config.get('Transform',None))
        
        self.sensors[lidar_config.get("name")]="3DLIDAR"    
        lidar_cams=[]
        for _ in range(n_cam):
            cam = gs.vis.camera.Camera(self.scene.visualizer,res=per_cam_resolution,
                                        model=model,up=(0.0, 0.0, 1.0),
                                        fov=22.5,near=near,far=far)
            self.scene._visualizer._cameras.append(cam)
            lidar_cams.append(cam)
        T_MAT_X,T_MATS_Z,T_MAT_TILT=get_camera_transformation_matrices(n_cameras=n_cam,
                                                                       h_fov=total_h_fov_angle_deg,
                                                                       tilt_angle=tilt_angle)
        lidar_qos_profile=create_qos_profile(
            lidar_config.get("QOS_history"),
            lidar_config.get("QOS_depth"),
            lidar_config.get("QOS_reliability"),
            lidar_config.get("QOS_durability")
        )
        topic=lidar_config.get('topic')
        lidar_pub=self.ros_node.create_publisher(PointCloud2,f"{self.namespace}/{topic}",lidar_qos_profile)
        setattr(self,f'{lidar_config.get("name")}_lidar_publisher',lidar_pub)
        timer=self.ros_node.create_timer(1/lidar_config.get('frequency', 1.0),
                                         lambda: timer_callback(lidar_pub,lidar_cams,T_MAT_X,T_MATS_Z,T_MAT_TILT,near,far))
        setattr(self,f'{lidar_config.get("name")}_lidar_timer',timer)       

    def add_laser_scan(self,laser_scan_config):
        def timer_callabck(laser_scan_pub,lidar_cams,
                           T_MAT_X,T_MATS_Z,
                           near,far):
            if self.scene.is_built:
                laser_scan_msg=lidar_cams_to_laser_scan_msg(lidar_cams,link,T,T_MAT_X,T_MATS_Z,
                                                            stamp=get_current_timestamp(self.scene),
                                                            near=near,far=far,
                                                            frame_id=laser_scan_config.get("frame_id"),
                                                            hfov=total_h_fov_angle_deg,
                                                            noise_mean=noise_mean,noise_std=noise_std)
                laser_scan_pub.publish(laser_scan_msg)
            
        h_min_angle=abs(laser_scan_config.get("horizontal_min_angle"))
        h_max_angle=laser_scan_config.get("horizontal_max_angle")
        total_h_fov_angle=h_min_angle+h_max_angle
        total_h_fov_angle_deg=math.ceil(np.rad2deg(total_h_fov_angle))
        n_cam=math.ceil(math.ceil(total_h_fov_angle_deg)/22.5)
        h_sample=laser_scan_config.get("h_samples")
        v_samples=1
        model='pinhole'
        per_cam_resolution=(h_sample//n_cam,v_samples)
        
        near=laser_scan_config.get("range_min")
        far=laser_scan_config.get("range_max")
        
        noise_mean=laser_scan_config.get("noise_mean",0.0)
        noise_std=laser_scan_config.get("noise_std",0.0)
        
        link_name = laser_scan_config.get("link")
        if self.robot.get_link(link_name) is None:
            raise ValueError(f"Link '{link_name}' not found in entity robot")
        
        link=self.robot.get_link(link_name)
        if laser_scan_config.get('Transform',None) is not None:
            T=np.array(laser_scan_config.get('Transform',None))
    
        self.sensors[laser_scan_config.get("name")]="LASERSCAN"
        lidar_cams=[]
        for i in range(n_cam):
            cam = gs.vis.camera.Camera(self.scene.visualizer,res=per_cam_resolution,
                                        model=model,
                                        fov=80,GUI=True,
                                        near=near,far=far)
            self.scene._visualizer._cameras.append(cam)
            lidar_cams.append(cam)
        T_MAT_X,T_MATS_Z,_=get_camera_transformation_matrices(n_cameras=n_cam,
                                                                h_fov=total_h_fov_angle_deg,
                                                                tilt_angle=None)
                
        laser_scan_qos_profile=create_qos_profile(
            laser_scan_config.get("QOS_history"),
            laser_scan_config.get("QOS_depth"),
            laser_scan_config.get("QOS_reliability"),
            laser_scan_config.get("QOS_durability")
        )
        laser_scan_pub=self.ros_node.create_publisher(LaserScan,f"{self.namespace}/{laser_scan_config.get('topic')}",laser_scan_qos_profile)
        setattr(self,f'{laser_scan_config.get("name")}_laser_scan_publisher',laser_scan_pub)
        timer=self.ros_node.create_timer(1/laser_scan_config.get("frequency", 1.0), lambda:timer_callabck(laser_scan_pub,lidar_cams,T_MAT_X,T_MATS_Z,near, far))
        setattr(self,f'{laser_scan_config.get("name")}_laser_scan_timer',timer)

    def add_imu(self,imu_config):
        def timer_callback(imu_pub,robot):
            if self.scene.is_built:
                imu_msg=Imu()
                imu_msg.header.frame_id=imu_config.get("frame_id")
                imu_msg.header.stamp=get_current_timestamp(self.scene)
                quat=gs_quat_to_ros_quat(robot.get_quat().detach().cpu().numpy().tolist()[0])
                imu_msg.orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
                ang_vel=robot.get_links_ang(robot.get_link("base_link").idx_local).detach().cpu().numpy().tolist()[0]
                imu_msg.angular_velocity=Vector3(x=ang_vel[0][0],y=ang_vel[0][1],z=ang_vel[0][2])
                lin_acc=robot.get_links_acc(robot.get_link("base_link").idx_local).detach().cpu().numpy().tolist()[0]
                imu_msg.linear_acceleration=Vector3(x=lin_acc[0][0],y=lin_acc[0][1],z=lin_acc[0][1])
                imu_pub.publish(imu_msg)
        
        self.sensors[imu_config.get("name")]="IMU"    
        imu_qos_profile=create_qos_profile(
            imu_config.get("QOS_history"),
            imu_config.get("QOS_depth"),
            imu_config.get("QOS_reliability"),
            imu_config.get("QOS_durability")
        )
        imu_pub=self.ros_node.create_publisher(Imu,f'{self.namespace}/{imu_config.get("topic")}',imu_qos_profile)
        setattr(self,f'{imu_config.get("name")}_imu_publisher',imu_pub)
        timer = self.ros_node.create_timer(1/imu_config.get('frequency', 1.0), lambda: timer_callback(imu_pub,self.robot))
        setattr(self,f'{imu_config.get("name")}_imu_timer',timer)  
        
    def add_contact_sensor(self,contact_sensor_config):
        def timer_callback(contacts_pub):
            if self.scene.is_built:
                if contact_sensor_config.get("target_object_name") is None:
                    target_entity=None
                elif get_entity(self.objects,contact_sensor_config.get("target_object_name")) is not None:
                    target_entity=get_entity(self.objects,contact_sensor_config.get("target_object_name"))
                elif get_entity(self.robots,contact_sensor_config.get("target_object_name")) is not None:
                    target_entity=get_entity(self.robots,contact_sensor_config.get("target_object_name"))
                else:
                    target_entity=None
                contacts=self.robot.get_contacts(with_entity=target_entity,
                                                exclude_self_contact=contact_sensor_config.get("exclude_self_contact",None))
                contacts_msg=Contacts()
                contact_msgs_list=[]
                for contact in contacts:
                    contact_msg=Contact()
                    contact_msg.collision_1=self.scene.rigid_solver.links[contact["link_a"]].name
                    contact_msg.collision_1=self.scene.rigid_solver.links[contact["link_b"]].name
                    contact_msg.position=contact.position
                    contact_msg.force_1=contact.force_a
                    contact_msg.force_2=contact.force_b
                    contact_msgs_list.append(contact_msg)
                contacts_msg.contacts=contact_msgs_list
                contacts_pub.publish(contacts_msg)
        self.sensors[contact_sensor_config.get("name")]="CONTACT"
        contact_sensor_qos_profile=create_qos_profile(
            contact_sensor_config.get("QOS_history"),
            contact_sensor_config.get("QOS_depth"),
            contact_sensor_config.get("QOS_reliability"),
            contact_sensor_config.get("QOS_durability")
        )
        topic=contact_sensor_config.get("topic")
        contacts_pub=self.ros_node.create_publisher(Contacts,f"{self.namespace}/{topic}",contact_sensor_qos_profile)
        setattr(self,f'{contact_sensor_config.get("name")}_contact_publisher',contacts_pub)
        timer=self.ros_node.create_timer(1/contact_sensor_config.get('frequency', 1.0),lambda:timer_callback(contacts_pub))
        setattr(self,f'{contact_sensor_config.get("name")}_contact_timer',timer)

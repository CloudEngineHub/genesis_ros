from sensor_msgs.msg import PointCloud2,PointField,LaserScan
import numpy as np
from cv_bridge import CvBridge
from .gs_ros_utils import rotate_around_line_local_x
import math

def make_cv2_bridge():
        return CvBridge()

def add_noise(arr, mean=0.0, std=1.0):
    noise = np.random.normal(loc=mean, scale=std, size=arr.shape)
    noisy = arr.astype(float) + noise
    return noisy.astype(arr.dtype)  # round/truncate implicitly
    
def points_to_pcd_msg(input_pc,stamp,frame_id):
        type_mappings = [
            (PointField.INT8, np.dtype('int8')), (PointField.UINT8, np.dtype('uint8')),
            (PointField.INT16, np.dtype('int16')), (PointField.UINT16, np.dtype('uint16')),
            (PointField.INT32, np.dtype('int32')), (PointField.UINT32, np.dtype('uint32')),
            (PointField.FLOAT32, np.dtype('float32')), (PointField.FLOAT64, np.dtype('float64'))
        ]
        nptype_to_pftype = {nptype: pftype for pftype, nptype in type_mappings}
        arr = input_pc.astype(np.float32)
        cloud_arr = np.core.records.fromarrays(arr.T, names='x,y,z', formats='f4,f4,f4')
        cloud_arr = np.atleast_2d(cloud_arr)

        # Create PointCloud2 message
        msg = PointCloud2()
        msg.height = cloud_arr.shape[0]
        msg.width = cloud_arr.shape[1]
        msg.is_bigendian = False
        msg.point_step = cloud_arr.dtype.itemsize
        msg.row_step = msg.point_step * cloud_arr.shape[1]
        msg.is_dense = all(np.isfinite(cloud_arr[name]).all() for name in cloud_arr.dtype.names)
        msg.data = cloud_arr.tobytes()

        if stamp is not None:
            msg.header.stamp = stamp
        if frame_id is not None:
            msg.header.frame_id = frame_id

        # Define PointFields
        msg.fields = []
        for name in cloud_arr.dtype.names:
            np_type, offset = cloud_arr.dtype.fields[name]
            pf = PointField()
            pf.name = name
            if np_type.subdtype:
                item_type, shape = np_type.subdtype
                pf.count = int(np.prod(shape))
                np_type = item_type
            else:
                pf.count = 1
            pf.datatype = nptype_to_pftype[np_type]
            pf.offset = offset
            msg.fields.append(pf)

        return msg
        
def lidar_cams_to_pcd_msg(cameras,link,link_offset_T,
                            T_MAT_X,T_MAT_Z,T_MAT_TILT, 
                            stamp, frame_id,hfov,near,far,
                            noise_mean=0.0,noise_std=0.0,):
        """
        Renders point clouds from multiple cameras, merges them, applies transforms,
        and returns a PointCloud2 message.
        """
        agg_pc = None
        camera_count = len(cameras)
        if camera_count >0:
            step=math.ceil(hfov)/(camera_count)
        for j, cam in enumerate(cameras):
            assert cam.is_built, f"CAMERA with id{cam.id} not Built"
            if cam._attached_link is None:
                cam.attach(link,link_offset_T)
            cam.move_to_attach()
            angle_rad = np.deg2rad(j * step)
            direction = np.array([np.cos(angle_rad), np.sin(angle_rad), 0.0])
            lookat = cam.pos + direction
            cam.set_pose(lookat=lookat)
            pc, _ = cam.render_pointcloud(world_frame=False)  # (H, W, 3)
            pc_mask = (
                (pc[:, :, 0] < near) |
                (pc[:, :, 0] > far-0.1) |
                (pc[:, :, 1] < near) |
                (pc[:, :, 1] > far-0.1) |
                (pc[:, :, 2] < near) |
                (pc[:, :, 2] > far-0.1)  
            )
            pc=pc[~pc_mask]
            
            pc = pc @ T_MAT_X.T @ T_MAT_Z[j].T @ T_MAT_TILT

            if agg_pc is None:
                agg_pc = pc
            else:
                agg_pc = np.concatenate([agg_pc, pc],axis=0)
        
        agg_pc=add_noise(agg_pc,noise_mean,noise_std)
        return points_to_pcd_msg(agg_pc,stamp=stamp,frame_id=frame_id)
            

def sectional_lidar_cam_to_pcd_msg(cameras,T_MAT_X,T_MAT_Z,T_MAT_TILT, 
                                        stamp, frame_id,
                                        near, far, tilt_angle=0,                                 
                                        noise_mean=0.0,noise_std=0.0):
    cam=cameras[0]
    cam.move_to_attach()
    lookat=rotate_around_line_local_x(cam.pos, cam.lookat,-tilt_angle)
    cam.set_pose(lookat=lookat)
        
    pc, _ = cam.render_pointcloud(world_frame=False)  # (H, W, 3)

    pc_mask = (
        (pc[:, :, 1] < near) |
        (pc[:, :, 1] > far) |
        (pc[:, :, 0] < near) |
        (pc[:, :, 0] > far) |
        (pc[:, :, 2] < near) |
        (pc[:, :, 2] > far)  
    )
    # pc[pc_mask] = np.array([0, 0, 0])
    pc=pc[~pc_mask]
    
    #modify
    pc = pc @ T_MAT_X.T @ T_MAT_Z[0].T @T_MAT_TILT
    
    pc=add_noise(pc,noise_mean,noise_std)

    return points_to_pcd_msg(pc,stamp=stamp,frame_id=frame_id)

def lidar_cams_to_laser_scan_msg(cameras,link,link_offset_T,
                                 T_MAT_X,T_MAT_Z,
                                stamp, frame_id,hfov,
                                near,far,
                                noise_mean=0.0,noise_std=0.0):
        """
        Renders point clouds from multiple cameras, merges them, applies transforms,
        and returns a PointCloud2 message.
        """
        agg_pc = None
        camera_count = len(cameras)
        if camera_count >0:
            step=hfov/camera_count
        step=hfov/(camera_count)
        for j, cam in enumerate(cameras):
            assert cam.is_built, f"CAMERA with id{cam.id} not Built"
            if cam._attached_link is None:
                    cam.attach(link,link_offset_T)
            cam.move_to_attach()
            angle_rad = np.deg2rad(j * step)
            direction = np.array([np.cos(angle_rad), np.sin(angle_rad), 0.0])
            lookat = cam.pos + direction
            cam.set_pose(lookat=lookat)

            pc, _ = cam.render_pointcloud(world_frame=False)  # (H, W, 3)
            
            pc_mask = (
                (pc[:, :, 0] < near) |
                (pc[:, :, 0] > far-0.1) |
                (pc[:, :, 1] < near) |
                (pc[:, :, 1] > far-0.1) |
                (pc[:, :, 2] < near) |
                (pc[:, :, 2] > far-0.1)  
            )
            # pc=pc[~pc_mask]
            pc[~pc_mask]=np.array([0,0,0])
            
            pc = pc @ T_MAT_X.T @ T_MAT_Z[j].T

            if agg_pc is None:
                agg_pc = pc
            else:
                agg_pc = np.concatenate([agg_pc, pc],axis=1)
            
        # Create PointCloud2 message
        agg_pc=add_noise(agg_pc,noise_mean,noise_std)
        distance= np.hypot(agg_pc[0][:,0],agg_pc[0][:,1]) 
        # distance
        msg=LaserScan()
        msg.angle_min=-np.deg2rad(hfov/2)
        msg.angle_max=np.deg2rad(hfov/2)
        msg.range_min=near
        msg.range_max=far
        msg.angle_increment=hfov/agg_pc.shape[1]

        msg.ranges=distance.tolist()
        if stamp is not None:
            msg.header.stamp = stamp
        if frame_id is not None:
            msg.header.frame_id = frame_id

        return msg
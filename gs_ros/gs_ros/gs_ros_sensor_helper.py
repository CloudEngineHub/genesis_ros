from sensor_msgs.msg import PointCloud2,PointField,LaserScan
import numpy as np
from cv_bridge import CvBridge

def make_cv2_bridge():
        return CvBridge()

def add_sensor_noise(arr, mean=0.0, std=1.0):
    noise = np.random.normal(loc=mean, scale=std, size=arr.shape)
    noisy = arr.astype(float) + noise
    return noisy.astype(arr.dtype)  # round/truncate implicitly
    
def raycaster_to_pcd_msg(raycaster, stamp, frame_id,add_noise=False,noise_mean=0.0,noise_std=0.0):
    if add_noise:
        return points_to_pcd_msg(add_sensor_noise(raycaster.read()[0].detach().cpu().numpy(),noise_mean,noise_std),
                                stamp=stamp,frame_id=frame_id)
    else:
        return points_to_pcd_msg(raycaster.read()[0].detach().cpu().numpy(),
                        stamp=stamp,frame_id=frame_id)

def grid_raycaster_to_pcd_msg(raycaster, stamp, frame_id,add_noise=False,noise_mean=0.0,noise_std=0.0):
    if add_noise:
        return points_to_pcd_msg(add_sensor_noise(raycaster.read()[0].detach().cpu().numpy(),noise_mean,noise_std),
                                stamp=stamp,frame_id=frame_id)
    else:
        return points_to_pcd_msg(raycaster.read()[0].detach().cpu().numpy(),
                        stamp=stamp,frame_id=frame_id)
        
def raycaster_to_laser_scan_msg(raycaster, stamp, frame_id,
                               angle_min,angle_max,
                               resolution,near,far,
                               add_noise=False,
                               noise_mean=0.0,noise_std=0.0,):
    if add_noise:
        data=add_sensor_noise(raycaster.read()[1][0].detach().cpu().numpy(),noise_mean,noise_std)
    else:
        data=raycaster.read()[1][0].detach().cpu().numpy()

    msg=LaserScan()
    msg.angle_min=np.deg2rad(angle_min)
    msg.angle_max=np.deg2rad(angle_max)
    msg.range_min=near
    msg.range_max=far
    msg.angle_increment=(abs(msg.angle_max)+abs(msg.angle_min))/resolution

    ranges=data.squeeze(1).tolist()
    msg.ranges=ranges
    if stamp is not None:
        msg.header.stamp = stamp
    if frame_id is not None:
        msg.header.frame_id = frame_id

    return msg
    
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
        
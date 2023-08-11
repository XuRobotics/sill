import numpy as np
from scipy.spatial.transform import Rotation
import rosbag
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
import argparse
from laserscan import LaserScan


def ros_pose_to_numpy(msg):
    R = Rotation.from_quat([msg.rotation.x,
                            msg.rotation.y,
                            msg.rotation.z,
                            msg.rotation.w])
    T = np.array([msg.translation.x,
                  msg.translation.y,
                  msg.translation.z])
    return {'R': R, 'T': T}


class DataLoader:
    def __init__(self, bagpath, spacing_sec=1):
        self.bag_ = rosbag.Bag(bagpath, 'r')
        self.spacing_sec_ = spacing_sec

    def __iter__(self):
        # reset iterator
        self.bag_iter_ = self.bag_.read_messages()
        self.frame_pose_ = None
        self.last_stamp_ = 0
        return self

    def __next__(self):
        stamp = 0
        data_sent = False
        # TODO: THIS CAN BE THE BIGGEST SOURCE OF ERROR
        # TODO: Maybe make this a argparser argument
        # Currently msg should be perfectly synced given a big enough time_sync_error
        # If only relevant topics are recorded then sync is perfect
        # TODO: CHECK THIS PROPERLY AGAIN
        time_sync_error = 1e5
        data = [None, None]
        time_stamp_data = [None, None, None]
        # this assumes that all messages with the same stamp arrive sequentially
        # before any messages of the next timestamp
        while True:
            if data_sent:
                # reset buffers
                data= [None, None]
                self.frame_pose_ = None
                data_sent = False
            topic, msg, t = self.bag_iter_.__next__()
            # 2.3e6 nsec sync error between os_node and faster_lio
            if topic == '/tf':
                for trans in msg.transforms:
                    if trans.header.stamp.to_nsec() > stamp - time_sync_error and \
                       trans.header.frame_id == 'camera_init' and \
                       trans.child_frame_id == 'body':
                        self.frame_pose_ = ros_pose_to_numpy(trans.transform)
                        time_stamp_data[0] = trans.header.stamp.to_sec()
            else:
                if msg.header.stamp.to_nsec() > self.last_stamp_ + self.spacing_sec_*1e9:
                    if msg.header.stamp.to_nsec() > stamp + time_sync_error:
                        # reset
                        stamp = msg.header.stamp.to_nsec()
                        data = [None, None]

                if abs(msg.header.stamp.to_nsec() - stamp) < time_sync_error:
                    if topic == '/cloud_registered_body':
                        # initially organized as x,y,z,space,intensity,space,...
                        # The last channel is point_step of cloud / 4 (cause float32 has 4 bytes).
                        pc = np.frombuffer(
                            msg.data, dtype=np.float32).reshape(-1, int(msg.point_step/4)).copy()
                        pc[:, 3] = pc[:, 4]
                        time_stamp_data[1] = msg.header.stamp.to_sec()

                        # TODO: Hardcoding the shape and fov
                        laser_scan_obj  = LaserScan(project=True, H=128, W=2048, fov_up=45.0, fov_down=-45.0)

                        laser_scan_obj.open_scan(points_xyz=pc[:, :3].copy(), points_intensity=pc[:, 3].copy())
                        organized_pc = np.dstack((laser_scan_obj.proj_xyz, laser_scan_obj.proj_remission[:,:,None]))

                        data[0] = organized_pc.reshape(-1, 4)
                        data[1] = organized_pc
                        time_stamp_data[2] = msg.header.stamp.to_sec()

                    # if topic == '/os_node/points':

                    #     pc = np.frombuffer(
                    #         msg.data, dtype=np.float32).reshape(-1, int(msg.point_step/4)).copy()
                    #     pc[:, 3] = pc[:, 4]
                    #     pc_reshaped = np.zeros((128,2048, 4), dtype=np.float32)
                    #     pc_reshaped[:,:,0] = pc[:,0].reshape(128,2048)
                    #     pc_reshaped[:,:,1] = pc[:,1].reshape(128,2048)
                    #     pc_reshaped[:,:,2] = pc[:,2].reshape(128,2048)
                    #     pc_reshaped[:,:,3] = pc[:,3].reshape(128,2048)
                    #     data[1] = pc_reshaped
                    #     time_stamp_data[2] = msg.header.stamp.to_sec()

                if all(d is not None for d in data) and self.frame_pose_ is not None:
                    # we have all elements
                    self.last_stamp_ = stamp
                    data_sent = True
                    print("-------------------")
                    print("Cloud Stamp / Size: ", time_stamp_data[1],data[0].shape)
                    print("-------------------")
                    return [data[0], data[1], self.frame_pose_, stamp]

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    # test
    ap = argparse.ArgumentParser()
    ap.add_argument('-d', '--dataset',
                    default="/home/ankit/Work/sloam_utils/bags/sill_test_bags/sill_test_os0_all_clouds.bag", help='path to dataset')
    args = vars(ap.parse_args())
    # d = DataLoader(args['dataset']).__iter__()
    # counter = 0
    # while True:
    #     print(counter)
    #     pc, pose, img, info = d.__next__()
    #     print(info.header.stamp.to_sec())
    #     counter += 1

    for count, (topic, msg, t) in enumerate(rosbag.Bag(args['dataset'], 'r').read_messages()):
        if topic == '/tf':
            for trans in msg.transforms:
                if trans.header.frame_id == 'camera_init' and \
                        trans.child_frame_id == 'body':
                    print(topic, t.to_sec())
        else:            
            print(topic, t.to_sec())
        if count == 50:
            break

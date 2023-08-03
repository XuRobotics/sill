import numpy as np
from scipy.spatial.transform import Rotation
import rosbag
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
import argparse


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
        time_sync_error = 2.3e6
        data = [None, None, None]
        # this assumes that all messages with the same stamp arrive sequentially
        # before any messages of the next timestamp
        while True:
            topic, msg, t = self.bag_iter_.__next__()
            #2.3e6 nsec sync error between os_node and faster_lio
            if topic == '/tf':
                for trans in msg.transforms:
                    if trans.header.stamp.to_nsec() > stamp - time_sync_error and \
                       trans.header.frame_id == 'camera_init' and \
                       trans.child_frame_id == 'body':
                        self.frame_pose_ = ros_pose_to_numpy(trans.transform)
            else:
                if msg.header.stamp.to_nsec() > self.last_stamp_ + self.spacing_sec_*1e9:
                    if msg.header.stamp.to_nsec() > stamp + time_sync_error:
                        # reset
                        stamp = msg.header.stamp.to_nsec()
                        data = [None, None, None]

                if abs(msg.header.stamp.to_nsec() - stamp) < time_sync_error:
                    if topic == '/cloud_registered_body':
                        # initially organized as x,y,z,space,intensity,space,...
                        pc = np.frombuffer(
                            msg.data, dtype=np.float32).reshape(-1, 8).copy()
                        pc[:, 3] = pc[:, 4]
                        data[0] = pc[:, :4]
                    elif topic == '/quadrotor/image':
                        data[1] = np.frombuffer(
                            msg.data, dtype=np.float32).reshape(64, 1024, -1)
                    elif topic == '/quadrotor/camera_info':
                        data[2] = msg

                    if all(d is not None for d in data) and self.frame_pose_ is not None:
                        # we have all elements
                        self.last_stamp_ = stamp
                        return [data[0], self.frame_pose_, data[1], data[2]]

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    # test
    ap = argparse.ArgumentParser()
    ap.add_argument('-d', '--dataset', default="/home/ankit/Work/sloam_utils/bags/sill_test.bag", help='path to dataset')
    args = vars(ap.parse_args())
    d = DataLoader(args['dataset']).__iter__()
    counter = 0
    while True:
        print(counter)
        pc, pose, img, info = d.__next__()
        print(info.header.stamp.to_sec())
        counter += 1

    # for count, (topic, msg, t) in enumerate(rosbag.Bag(args['dataset'], 'r').read_messages()):
    #     print(t.to_sec(), topic, count)
    #     if count==50:
    #         break

import rosbag
import argparse
import numpy as np


class ExamineRosTopic():
    def __init__(self, bag_path, topic_name1=None, topic_name2=None):

        self.bag_topic1_ = rosbag.Bag(
            bag_path).read_messages(topics=[topic_name1])

        self.msg_count_ = 200
        self.pc_intensity_mean_ = np.array([])
        self.pc_intensity_min = np.array([])
        self.pc_intensity_max = np.array([])
        self.pc_intensity_99th = np.array([])

    def topic_playback(self):

        print("-----------------------------------")
        print("Providing stats over {} messages".format(self.msg_count_))

        for _ in range(self.msg_count_):
            try:
                topic, msg, t = next(self.bag_topic1_)
            except:
                print("Used up all messages in the bag")
                break

            pc = np.frombuffer(
                msg.data, dtype=np.float32).reshape(-1, int(msg.point_step/4)).copy()
            # Replacing space with intensity channel
            pc[:, 3] = pc[:, 4]
            data = pc[:, 3]
            self.pc_intensity_mean_ = np.append(
                self.pc_intensity_mean_, np.mean(data))
            self.pc_intensity_min = np.append(
                self.pc_intensity_min, np.min(data))
            self.pc_intensity_max = np.append(
                self.pc_intensity_max, np.max(data))
            self.pc_intensity_99th = np.append(
                self.pc_intensity_99th, np.percentile(data, 99))
        
        # Print the results
        print("99th Percentile Intensity is: ", np.mean(self.pc_intensity_99th))
        print("Play around with this number in the params.yaml file for good point cloud visuals in tool")
        print("Lower the number, brighter the point cloud, and vice versa")
        print("-----------------------------------")


if __name__ == '__main__':

    ap = argparse.ArgumentParser()
    ap.add_argument("-b", "--bag", default="/home/ankit/Work/sloam_utils/bags/sill_test_bags/sill_test_os0_all_clouds.bag",
                    help="path to input bag file")
    ap.add_argument("-t", "--topic", default="/cloud_registered_body",
                    help="name of topic to examine")
    args = vars(ap.parse_args())

    ex = ExamineRosTopic(args["bag"], args["topic"])
    ex.topic_playback()

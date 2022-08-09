#!/usr/bin/python
# coding:utf-8

from ntpath import join
import time
import csv
import rospy
import tf
from datetime import datetime
from visualization_msgs.msg import MarkerArray

class HumanPose:
    def __init__(self):
        self.joints = []
        self.time = 0
    def get_string_list(self):
        human_string_list = []
        human_string_list.append(self.time)
        for joint_num in range(32):
            human_string_list += self.joints[joint_num].get_string()
        return human_string_list

class JointPose:
    def __init__(self):
        self.name = ""
        self.position = []
        self.orientation = []
    def __repr__(self):
        return "\nJointPose: \n Name:  %s\n Position: %s\n Orientation: %s\n" % (self.name,self.position,self.orientation)
    def get_string(self):
        joint_string_list = []
        joint_string_list.append(self.name)
        joint_string_list.append(self.position[0])
        joint_string_list.append(self.position[1])
        joint_string_list.append(self.position[2])
        joint_string_list.append(self.orientation[0])
        joint_string_list.append(self.orientation[1])
        joint_string_list.append(self.orientation[2])
        joint_string_list.append(self.orientation[3])
        return joint_string_list

class PoseSaver:
    def __init__(self):
        human_pose_topic = rospy.get_param('~human_pose_topic', '/body_tracking_data')
        self.listener = tf.TransformListener()
        human_pose_sub = rospy.Subscriber(human_pose_topic, MarkerArray, self.get_pose)
        self.start_flag = False
        self.human_poses = []

    def __del__(self):
        
        if len(self.human_poses)>0:
            print("output_csv")
            self.output_csv()

    def get_pose(self, msg):
        self.human_pose = []
        try:
            (trans,rot) = self.listener.lookupTransform('/rgb_camera_link', '/checker_board', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return False
        
        joints = []
        for joint_num in range(32):
            frame_name = "1"+str(joint_num).zfill(2)
            try:
                (trans,rot) = self.listener.lookupTransform(frame_name, '/checker_board', rospy.Time(0))
                joint_pose = JointPose()
                joint_pose.name = frame_name
                joint_pose.position = [trans[0],trans[1],trans[2]]
                joint_pose.orientation = [rot[0],rot[1],rot[2],rot[3]]
                joints.append(joint_pose)
                print(joint_pose)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                return False

        if not self.start_flag: 
            self.start_flag = True
            self.start_time = time.time()

        human_pose = HumanPose()
        human_pose.time = time.time() - self.start_time 
        human_pose.joints = joints
        self.human_poses.append(human_pose)

    def output_csv(self):
        filename = datetime.now().strftime("%Y_%m%d_%H%M_%S")
        total_filename = '/pose_data/' + filename + ".csv"
        with open(total_filename, 'w') as f:
            writer = csv.writer(f)
            for human_pose_index in range(len(self.human_poses)):
                writer.writerow(self.human_poses[human_pose_index].get_string_list())
            f.close()

if __name__ == '__main__':
    rospy.init_node("pose_saver")
    pose_saver = PoseSaver()
    rospy.spin()
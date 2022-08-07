#!/usr/bin/python
# coding:utf-8

import rospy
import tf
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped

class TfPublisher:
    def __init__(self):
        human_pose_topic = rospy.get_param('~human_pose_topic', '/body_tracking_data')
        board_pose_topic = rospy.get_param('~board_pose_topic', '/objectdetection_pose')
        human_pose_sub = rospy.Subscriber(human_pose_topic, MarkerArray, self.human_pose_cb)
        board_pose_sub = rospy.Subscriber(board_pose_topic, PoseStamped, self.board_pose_cb)
        self.br = tf.TransformBroadcaster()
    def human_pose_cb(self, msg):
        for marker in msg.markers:
            self.br.sendTransform((marker.pose.position.x,marker.pose.position.y,marker.pose.position.z),
                     (marker.pose.orientation.x,marker.pose.orientation.y,marker.pose.orientation.z,marker.pose.orientation.w),
                     rospy.Time.now(),
                     str(marker.id),
                     marker.header.frame_id)
    def board_pose_cb(self, msg):
        self.br.sendTransform((msg.pose.position.x,msg.pose.position.y,msg.pose.position.z),
                     (msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w),
                     rospy.Time.now(),
                     "/checker_board",
                     msg.header.frame_id)
if __name__ == '__main__':
    rospy.init_node("tf_publisher")
    tf_publisher = TfPublisher()
    rospy.spin()
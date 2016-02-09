#!/usr/bin/env python

import roslib; roslib.load_manifest('pr2_delivery')
import rospy
import actionlib
import geometry_msgs.msg
import tf.transformations
from pr2_delivery.msg import DeliverAction, DeliverGoal

def pose_from_xytheta(x, y, theta):
    pose = geometry_msgs.msg.PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = '/map'
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0
    quat = tf.transformations.quaternion_from_euler(0, 0, theta)
    pose.pose.orientation = geometry_msgs.msg.Quaternion(*quat)
    return pose

if __name__ == '__main__':
    rospy.loginfo("starting...")
    rospy.init_node('deliver_test_client')
    client = actionlib.SimpleActionClient('deliver', DeliverAction)
    rospy.loginfo("waiting for action server...")
    client.wait_for_server()

    goal = DeliverGoal()
    # Fill in goal here.
    goal.get_object_pose = pose_from_xytheta( 12.7, 13.1, 1.52 )
    goal.give_object_pose = pose_from_xytheta( 13.4, 9.2, -1.52 )
    goal.return_home_pose = pose_from_xytheta( 12.7, 12.2, 0 )

    rospy.loginfo("sending goal.")
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(50.0))
    rospy.loginfo("got action finished.")
    rospy.sleep(50)

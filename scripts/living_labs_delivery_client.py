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
    rospy.loginfo("waiting for delivery action server ...")
    client.wait_for_server()

    goal = DeliverGoal()
  # <param name="/amcl/initial_pose_x" value="2.00" />
  # <param name="/amcl/initial_pose_y" value="11.60" />
    # Fill in goal here.
    goal.get_object_pose = pose_from_xytheta( 4.0, 9., -0.6 ) #
    goal.give_object_pose = pose_from_xytheta( 4.7, 12.5, 1.5 ) # x was 3.3
    goal.return_home_pose = pose_from_xytheta( 2., 11.6, 0.0 ) # 2.4; 8.2, 0

    rospy.loginfo("Sending delivery goal.")
    client.send_goal(goal)
    rospy.loginfo("Waiting for results ...")
    # client.wait_for_result(rospy.Duration.from_sec(50.0))
    client.wait_for_result()

    rospy.loginfo(client.get_result())
    if (client.get_state()):
        print("Results: Success")
    else:
        print("Results: Failed!")
    rospy.loginfo("Delivery action finished.")
    # rospy.sleep(50)
    rospy.loginfo("Done.")

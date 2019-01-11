# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

PACKAGE = 'pr2_delivery'

import os
import roslib
roslib.load_manifest(PACKAGE)
import subprocess
import rospy
import actionlib
import geometry_msgs.msg
import pr2_common_action_msgs.msg
from pr2_delivery.msg import DeliverAction, DeliverGoal
from pr2_head.srv import Query
from ArmMover import ArmMover
from pr2_gripper_sensor_msgs.msg import PR2GripperEventDetectorAction, PR2GripperEventDetectorGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty


class DeliverServer:

    tucked_with_object_pose = [-0.0175476818422744, 1.1720448201611564,
                               -1.3268105758514066, -1.288722079574422,
                               -31.28968470078213, -2.0089650532319836,
                               -5.841424529413016]
    tuck_approach_pose = [0.039, 1.1072, 0.0, -2.067, -1.231, -1.998, 0.369]
    accept_object_pose = [-0.07666010001780543, 0.1622352230632809,
                          -0.31320771836735584, - 1.374860652847621,
                          -3.1324415863359545, -1.078194355846691,
                          1.857217828689617]
    tucked_with_object_approach_pose = [-0.01829384139848722, 0.6712428753827848,
                                        -1.3264898661986668,
                                        -0.6078654239376914,
                                        0.601472182148825, -1.3278329090728338,
                                        -5.83346239703479]

    wiggle_acceleration = 2


    def __init__(self):
        self.tuck_arm_client = actionlib.SimpleActionClient(
            "tuck_arms", pr2_common_action_msgs.msg.TuckArmsAction)
        self.gripper_wiggle_detector_client = actionlib.SimpleActionClient(
            'r_gripper_sensor_controller/event_detector', PR2GripperEventDetectorAction)
        self.move_base_client = actionlib.SimpleActionClient(
            'move_base', MoveBaseAction)
        self.clear_costmaps_client = rospy.ServiceProxy(
            '/move_base_node/clear_costmaps', Empty)
        self.fetch = rospy.Service('fetch', Empty, self.fetch_request)

        # Load phrases
        self.lang = rospy.get_param('~lang', 'en')
        self.person_with_laserscanner = 'Michael'
        self.person_needing_laserscanner = 'Yordan'

        self.pr2_go = False
        self.start_delivery_phrase = rospy.get_param(
            '~start_delivery_phrase', 'I believe {} needs a laser scanner.'.format(self.person_needing_laserscanner))
        self.request_item_phrase = rospy.get_param('~request_item_phrase',
                                                   '{}, do you have a laser scanner for me?.'.format(self.person_with_laserscanner))
        self.item_received_phrase = rospy.get_param('~item_received_phrase',
                                                    'Thank you. I will give this to {}.'.format(self.person_needing_laserscanner))
        self.give_item_phrase = rospy.get_param('~give_item_phrase',
                                                'Here is your laser scanner, {}.'.format(self.person_needing_laserscanner))
        self.item_delivered_phrase = rospy.get_param('~item_delivered_phrase',
                                                     'You\'re welcome. Good luck with the assembly.')
        self.home_phrase = rospy.get_param('~home_phrase',
                                           'Thanks!.')
        self.wiggle_acceleration = rospy.get_param('~wiggle_acceleration', 2)

        self.arm_mover = ArmMover()

        rospy.loginfo("Waiting for tuck_arms action server")
        self.tuck_arm_client.wait_for_server(rospy.Duration(10.0))
        rospy.loginfo("Waiting for wiggle_detector action server")
        self.gripper_wiggle_detector_client.wait_for_server(
            rospy.Duration(10.0))
        rospy.loginfo("Waiting for move_base action server")
        self.move_base_client.wait_for_server(rospy.Duration(10.0))
        rospy.loginfo("Waiting for clear costmaps service")
        rospy.wait_for_service('/move_base_node/clear_costmaps')

        self.server = actionlib.SimpleActionServer(
            'deliver', DeliverAction, self.execute, False)
        self.server.start()
        self.say("PR2 Delivery Ready")

        rospy.loginfo("Ready")

    def execute(self, goal):
        """This is the main sequence of the delivery action."""
        try:
            self.tuck_arms()
            self.wait_for_signal()
            rospy.loginfo("*****************************")
            # rospy.sleep(10)
            self.say(self.start_delivery_phrase)
            self.navigate_to(goal.get_object_pose)
            self.say(self.request_item_phrase)
            self.get_object()
            self.say(self.item_received_phrase)

            self.deliver_signal()
            self.navigate_to(goal.give_object_pose)
            self.say(self.give_item_phrase)
            self.give_object()
            self.say(self.item_delivered_phrase)
            self.tuck_arms()
            self.navigate_to(goal.return_home_pose)
            self.say(self.home_phrase)
            self.server.set_succeeded()
            self.pr2_go = False
        except Exception, error:
            rospy.logerr("Delivery failed: %s" % error)
            # sys.exit(0)

    def fetch_request(self, req):
        """Fetch service signal to know when PR2 may go fetch"""
        rospy.loginfo("FETCHING!")
        self.pr2_go = True
        return []

    def deliver_signal(self):
        try:
            os.system("bash -i -c 'export ROS_MASTER_URI=\"http://jazz:11311\" && rosservice call /deliver \"{}\"'")
        except rospy.ServiceException, error:
            rospy.loginfo("Service call failed: %s" % error)

    def wait_for_signal(self):
        rate = rospy.Rate(10)  # 10 Hz
        rospy.loginfo("waiting for fetch signal...")
        while not rospy.is_shutdown() and not self.pr2_go:
            rate.sleep()

    def say(self, thing_to_say):
        rospy.loginfo("saying '%s'" % thing_to_say)
        try:
            speak = rospy.ServiceProxy('/pr2_head/say', Query)
            resp1 = speak(thing_to_say)
            return resp1.success
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def clear_costmaps(self):
        rospy.loginfo("clearing costmaps")
        try:
            self.clear_costmaps_client()
        except rospy.ServiceException, error:
            rospy.loginfo("Service call failed: %s" % error)

    def tuck_arms(self):
        rospy.loginfo("tucking arms")
        goal = pr2_common_action_msgs.msg.TuckArmsGoal()
        goal.tuck_left = True
        goal.tuck_right = True
        self.tuck_arm_client.send_goal_and_wait(
            goal, rospy.Duration(30.0), rospy.Duration(5.0))
        # self.tuck_arm_client.cancel_goal()

    def navigate_to(self, nav_goal_pose):
        self.clear_costmaps()
        rospy.loginfo("navigating to %f %f" % (nav_goal_pose.pose.position.x,
                                               nav_goal_pose.pose.position.y))
        goal = MoveBaseGoal()
        goal.target_pose = nav_goal_pose
        rospy.loginfo("navigating to state before")
        rospy.loginfo(self.move_base_client.get_state())
        self.move_base_client.send_goal_and_wait(
            goal, rospy.Duration(60*5), rospy.Duration(5))
        rospy.loginfo("navigating to state after")
        rospy.loginfo(self.move_base_client.get_state())
        self.move_base_client.cancel_goal()
        rospy.loginfo("navigating to state after cancel")
        rospy.loginfo(self.move_base_client.get_state())

    def wait_for_gripper_wiggle(self, accel):
        """Waits for one year.  accel is in m/s^2, normal values are 6 and 10"""
        # contents of function ported from pr2_props
        goal = PR2GripperEventDetectorGoal()
        # use just acceleration as our contact signal
        goal.command.trigger_conditions = 4
        goal.command.acceleration_trigger_magnitude = accel
        goal.command.slip_trigger_magnitude = 0.008  # slip gain
        self.gripper_wiggle_detector_client.send_goal_and_wait(
            goal, rospy.Duration(3600), rospy.Duration(5))
        self.gripper_wiggle_detector_client.cancel_goal()

    def get_object(self):
        rospy.loginfo("getting object")
        # - move right arm to accept-object pose
        self.arm_mover.go('r', self.tuck_approach_pose, 1)
        self.arm_mover.go('r', self.accept_object_pose, 2)

        object_gripped = False
        while not object_gripped:
            # - open gripper all the way
            self.arm_mover.open_right()
            rospy.sleep(2)
            # - wait for externally-applied hand motion detected (ala "fist-pump" demo)
            self.wait_for_gripper_wiggle(self.wiggle_acceleration)  # m/s^2
            # - close gripper all the way
            self.arm_mover.close_right()
            # - if gripper closes all the way, no object is gripped
            gripper_pos = self.arm_mover.joints['r_gripper_joint'].position
            if gripper_pos > 0.01:
                rospy.loginfo("gripper has object: %f", gripper_pos)
                object_gripped = True
            else:
                rospy.loginfo("gripper does not have object: %f", gripper_pos)

        self.arm_mover.go('r', self.tucked_with_object_approach_pose, 2)
        self.arm_mover.go('r', self.tucked_with_object_pose, 1)

    def give_object(self):
        rospy.loginfo("giving object")
        # move out to tucked-with-object approach pose for right arm
        self.arm_mover.go('r', self.tucked_with_object_approach_pose, 2)
        # - move right arm to give-object pose
        # give-object pose is the same as accept-object pose.
        self.arm_mover.go('r', self.accept_object_pose, 2)
        # - let arm motion settle
        rospy.sleep(2)
        # - wait for externally-applied hand motion detected (ala "fist-pump" demo)
        self.wait_for_gripper_wiggle(self.wiggle_acceleration)  # m/s^2
        # - open gripper
        self.arm_mover.open_right()
        rospy.sleep(1)

# Question: what if sequence is pre-empted while robot is holding object?
# - maybe set the object on the floor?
# - v1 is certainly just "do nothing" so robot is still holding the object.

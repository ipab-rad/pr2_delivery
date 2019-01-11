#!/usr/bin/env python

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

from pr2_picknplace_msgs.msg import PickPlaceAction, PickPlaceGoal

from pr2_delivery.ArmMover import ArmMover
from pr2_head_msgs.msg import LookAt
from pr2_head.srv import Query
# from ArmMover import ArmMover
from pr2_gripper_sensor_msgs.msg import PR2GripperEventDetectorAction, PR2GripperEventDetectorGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty
# TODO write a separate pr2_delivery srv (YORDAN)
from pr2_head.srv import Query

import tf
import numpy as np
from math import cos, sin


class ObjectGatherServer:

    def __init__(self):
        self.tuck_arm_client = actionlib.SimpleActionClient(
            "tuck_arms", pr2_common_action_msgs.msg.TuckArmsAction)

        self.r_gripper_wiggle_detector_client = actionlib.SimpleActionClient(
            'r_gripper_sensor_controller/event_detector', PR2GripperEventDetectorAction)

        self.l_gripper_wiggle_detector_client = actionlib.SimpleActionClient(
            'l_gripper_sensor_controller/event_detector', PR2GripperEventDetectorAction)

        self.move_base_client = actionlib.SimpleActionClient(
            'move_base', MoveBaseAction)
        self.clear_costmaps_client = rospy.ServiceProxy(
            '/move_base_node/clear_costmaps', Empty)
        self.fetch = rospy.Service('fetch', Query, self.execute)

        self.listener = tf.TransformListener()

        # self.arm_mover = ArmMover()

        # Head gaze control
        self.look_pub = rospy.Publisher('pr2_head/target_object', LookAt, queue_size=10)

        # Arms picknplace action clients
        self.left_arm_client = actionlib.SimpleActionClient(
            '/pr2_picknplace_left/pr2_picknplace', PickPlaceAction)
        self.right_arm_client = actionlib.SimpleActionClient(
            '/pr2_picknplace_right/pr2_picknplace', PickPlaceAction)

        self.pr2_go = False

        # format - [frame_id, x, y, z, angle, PickPlace Request]
        self.target_locations = {}
        self.target_locations["home"] =             ['map',         2.1,    8.4,    0,       0,      None]

        self.target_locations["table_1"] =          ['map',         3.6,    10.2,   0,       1.57,   None]
        self.target_locations["table_2"] =          ['map',         3.0,    8.6,    0,       0.01,    None]
        # self.target_locations["interm"]  =          ['map',         2.1,    9.6,    0,       -1.57,  None]
        self.target_locations["table_3"] =          ['map',         3.0,    7.2,    0,       -1.57,  None]
        self.target_locations["dropoff"] =          ['dropoff',     0.0,    0.0,    0.01,     None,   1]

        self.target_locations["object_1"] =         ['object_1',    0.06,   0.0,    -0.03,    None,   0]
        self.target_locations["object_2"] =         ['object_2',    0.07,   0.0,    -0.03,    None,   0]
        self.target_locations["object_3"] =         ['object_3',    0.06,   0.0,    -0.02,    None,   0]
        self.target_locations["object_4"] =         ['object_4',    0.06,   0.0,    -0.032,    None,   0]

        self.target_locations["forwards"] =         ['base_link',   5.0,    0.0,    2.0,     None,   None]
        self.target_locations["down"] =             ['base_link',   1.0,    0.0,    0.3,     None,   None]
        self.target_locations["comfort_right"] =    ['base_link',   0.4,    -0.3,   1.0,     0,      2]
        self.target_locations["comfort_left"] =     ['base_link',   0.4,    0.3,    1.0,     0,      2]
        self.target_locations["give_r"] =           ['base_link',   0.6,    -0.2,  1.0,     0,      2]
        self.target_locations["give_l"] =           ['base_link',   0.6,    0.2,   1.0,     0,      2]
        self.target_locations["safedrop"] =         ['base_link',   0.5,    0,      0.9,     0,      1]


        self.target_locations["go"] =          ['map',         3.0,    8.6,    0,       0.01,    None]
        self.target_locations["turn"] =          ['map',         3.0,    8.6,    0,       3,    None]

        self.wiggle_acceleration = 5

        rospy.loginfo("Waiting for tuck_arms action server")
        self.tuck_arm_client.wait_for_server(rospy.Duration(30.0))
        rospy.loginfo("Waiting for move_base action server")
        self.move_base_client.wait_for_server(rospy.Duration(30.0))

        # Grippers wiggle clients/servers
        rospy.loginfo("Waiting for r_gripper_wiggle_detector_client")
        self.r_gripper_wiggle_detector_client.wait_for_server(rospy.Duration(30.0))
        rospy.loginfo("Waiting for L_gripper_wiggle_detector_client")
        self.l_gripper_wiggle_detector_client.wait_for_server(rospy.Duration(30.0))

        # Arms picknplace action clients/servers
        rospy.loginfo("Waiting for left_arm action server")
        self.left_arm_client.wait_for_server(rospy.Duration(30.0))
        rospy.loginfo("Waiting for right_arm action server")
        self.right_arm_client.wait_for_server(rospy.Duration(30.0))

        rospy.loginfo("Waiting for clear costmaps service")
        rospy.wait_for_service('/move_base_node/clear_costmaps')

        # self.server = actionlib.SimpleActionServer(
        #     'deliver', DeliverAction, self.execute, False)
        # self.server.start()
        # self.say("PR2 Delivery Ready")

        # self.tuck_arms()
        self.prepare_arms()
        rospy.loginfo("READY FOR ACTION")
        self.say("I am ready for action")

    def execute(self, goal):
        """This is the main sequence of the delivery action."""

        action = goal.query
        rospy.loginfo(action)
        if action == "look at me":
            self.look_at_me()
        elif action == "go home":
            self.go_home()
        elif action == "come here":
            self.come_here()
        elif action == "bring knife and fork" or action == "bring fork and knife":
            self.bring_objects("table 2", "object_1", "object_2")
        elif action == "bring banana and apple" or action == "bring apple and banana":
            self.bring_objects("table 3", "object_3", "object_4")
        elif action == "take knife and fork" or action == "take fork and knife":
            self.take_objects("table 2", "object_1", "object_2")
        elif action == "take banana and apple" or action == "take apple and banana":
            self.take_objects("table 3", "object_3", "object_4")
        elif action == "stop":
            rospy.loginfo("STOPPING")
            self.move_base_client.cancel_goal()
        elif action == "go":
            tmp_pos = self.pose_from_xytheta(self.target_locations['go'])
            self.navigate_to(tmp_pos)
        elif action == "turn left":
            # tmp_pos = self.pose_from_xytheta(self.target_locations['turn'])
            # self.navigate_to(tmp_pos)
            self.turn("left")
        elif action == "turn right":
            # tmp_pos = self.pose_from_xytheta(self.target_locations['turn'])
            # self.navigate_to(tmp_pos)
            self.turn("right")

    def go_home(self):
        self.say("I am going home now. Good Bye!")
        # Go Home
        self.turn("left")
        home = self.pose_from_xytheta(self.target_locations['home'])
        self.navigate_to(home)
        self.get_comfy()
        self.look_at("forwards")

    def come_here(self):
        self.say("I am coming to you")
        self.look_at("down")
        table_pos = self.pose_from_xytheta(self.target_locations['table_1'])
        self.navigate_to(table_pos)
        self.get_comfy()
        self.look_at("down")
        self.visual_servo_to("/table_1", table_pos)
        self.say("I am here. How can I help you?")

    def look_at_me(self):
        self.look_at("table_1")
        self.say("How can I help you")


    def bring_objects(self, target_table, object_right, object_left):
        try:
            self.say("I will be right back!")
            self.look_at("down")

            # Fetch objects
            target_table = target_table.replace(" ", "_")
            table_pos = self.pose_from_xytheta(self.target_locations[target_table])
            self.navigate_to(table_pos)
            self.get_comfy()
            self.look_at("down")
            self.visual_servo_to("/"+target_table, table_pos)

            self.pick_place("r", object_right, "pick")
            self.get_comfy()
            self.pick_place("l", object_left, "pick")
            self.get_comfy()

            # Bring object to target location
            self.turn("left")
            table_pos = self.pose_from_xytheta(self.target_locations['table_1'])
            self.navigate_to(table_pos)
            self.look_at("down")
            self.visual_servo_to("/table_1", table_pos)

            # Drop the cube
            self.pick_place("r", "dropoff", "place")
            self.get_comfy()
            self.pick_place("l", "dropoff", "place")
            self.get_comfy()

            self.say("What else can I do for you?")
        except Exception, error:
            rospy.logerr("Delivery failed: %s" % error)

    def take_objects(self, target_table, object_right, object_left):
        try:
            # Get objects 1 and 2
            self.say("I will take away two items.")
            self.look_at("down")
            rospy.sleep(2)

            rospy.loginfo("GIVE ME FIRST ITEM")
            self.say("Can you pass me the first item?")
            self.take_object("r", self.right_arm_client)

            rospy.loginfo("GIVE ME SECOND ITEM")
            self.say("Can you pass me the second item?")
            self.take_object("l", self.left_arm_client)

            self.get_comfy()
            self.say("Ok. I will be right back.")
            target_table = target_table.replace(" ", "_")
            # Take objects 1 and 2 to table 2
            self.turn("left")
            table_pos = self.pose_from_xytheta(self.target_locations[target_table])
            self.navigate_to(table_pos)
            self.get_comfy()
            self.look_at("down")
            self.visual_servo_to("/"+target_table, table_pos)

            self.pick_place("r", object_right, "place")
            self.get_comfy()
            self.pick_place("l", object_left, "place")
            self.get_comfy()

            # Go back to table 1 for objects 3 and 4
            self.turn("left")
            table_pos = self.pose_from_xytheta(self.target_locations['table_1'])
            self.navigate_to(table_pos)
            self.look_at("down")
            self.visual_servo_to("/table_1", table_pos)

            # self.say("Can you pass me the third item?")
            # self.take_object("r", self.right_arm_client)

            # self.say("Can you pass me the fourth item?")
            # self.take_object("l", self.left_arm_client)

            # self.say("I will bring them to table 3")

            # Take objects 3 and 4 to table 3
            # table_3 = self.pose_from_xytheta(self.target_locations['table_3'])
            # self.navigate_to(table_3)
            # self.look_at("down")
            # self.get_comfy()
            # self.visual_servo_to("/table_3", table_3)

            # self.pick_place("r", "object_3", "place")
            # self.get_comfy()
            # self.pick_place("l", "object_4", "place")
            # self.get_comfy()

            # # Go back to table 1
            # table_1 = self.pose_from_xytheta(self.target_locations['table_1'])
            # self.navigate_to(table_1)
            # self.look_at("down")
            # self.visual_servo_to("/table_1", table_1)

            self.say("What else can I do for you?")
        except Exception, error:
            rospy.logerr("Delivery failed: %s" % error)

    def fetch_request(self, req):
        """Fetch service signal to know when PR2 may go fetch"""
        rospy.loginfo("FETCHING!")
        self.pr2_go = True
        return []

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

    def look_at(self, target_object_str):
        target_object = LookAt()
        target_object.frame = self.target_locations[target_object_str][0]
        target_object.follow = False

        target_object.offset = geometry_msgs.msg.Point()
        target_object.offset.x = self.target_locations[target_object_str][1]
        target_object.offset.y = self.target_locations[target_object_str][2]
        target_object.offset.z = self.target_locations[target_object_str][3]

        self.look_pub.publish(target_object)

        # HACK - fix pr2_head looking_at functionality
        rospy.sleep(0.5)
        target_object.frame = "nothing"
        self.look_pub.publish(target_object)

    def tuck_arms(self):
        rospy.loginfo("tucking arms")
        goal = pr2_common_action_msgs.msg.TuckArmsGoal()
        goal.tuck_left = True
        goal.tuck_right = True
        self.tuck_arm_client.send_goal_and_wait(
            goal, rospy.Duration(30.0), rospy.Duration(5.0))

    def untuck_arms(self):
        rospy.loginfo("untucking arms")
        goal = pr2_common_action_msgs.msg.TuckArmsGoal()
        goal.tuck_left = False
        goal.tuck_right = False
        self.tuck_arm_client.send_goal_and_wait(
            goal, rospy.Duration(30.0), rospy.Duration(5.0))

    def prepare_arms(self):
        self.untuck_arms()
        self.get_comfy()

    def get_comfy(self):
        right_goal = self.get_pickplace_goal(self.target_locations["comfort_right"])
        self.right_arm_client.send_goal_and_wait(
            right_goal, rospy.Duration(30), rospy.Duration(5))
        self.right_arm_client.cancel_goal()

        left_goal = self.get_pickplace_goal(self.target_locations["comfort_left"])
        self.left_arm_client.send_goal_and_wait(
            left_goal, rospy.Duration(30), rospy.Duration(5))
        self.left_arm_client.cancel_goal()

    def pick_place(self, arm, object_name, action):
        if arm == "l":
            client = self.left_arm_client
        elif arm == "r":
            client = self.right_arm_client

        goal = self.get_pickplace_goal(self.target_locations[object_name])

        if action == "pick":
            goal.goal.request = 0
        elif action == "place":
            goal.goal.request = 1

        client.send_goal_and_wait(goal, rospy.Duration(30))
        result = client.get_result()
        client.cancel_goal()

        # if it fails to drop an item and is next to human - pass the item; otherwise try to drop it
        # with respect to base_link
        if result.success == False and goal.goal.request == 1:
            if goal.goal.header.frame_id == "dropoff":
                self.say("I can't put down the Item. Can you take it from me?")
                rospy.sleep(2)
                self.give_object(arm, client)
            else:
                goal = self.get_pickplace_goal(self.target_locations["safedrop"])
                client.send_goal_and_wait(goal, rospy.Duration(30))
        # if it fails to pick up an item ask a human to pass it
        elif result.success == False and goal.goal.request == 0:
            self.say("I can't pick up the Item. Can you pass it from me?")
            rospy.sleep(2)
            self.take_object(arm, client)

    def give_object(self, arm, client):
        rospy.loginfo("giving object")
        # move out to tucked-with-object approach pose for right arm
        goal = self.get_pickplace_goal(self.target_locations["give_"+arm])
        # goal.goal.object_pose.orientation.x = 0.5
        # goal.goal.object_pose.orientation.y = -0.5
        # goal.goal.object_pose.orientation.z = 0.5
        # goal.goal.object_pose.orientation.w = 0.5
        goal.goal.object_pose.orientation.x = 0.0
        goal.goal.object_pose.orientation.y = 0.0
        goal.goal.object_pose.orientation.z = 0.0
        goal.goal.object_pose.orientation.w = 1.0
        client.send_goal_and_wait(goal, rospy.Duration(30), rospy.Duration(5))
        # - wait for externally-applied hand motion detected (ala "fist-pump" demo)
        rospy.sleep(1.5)
        self.wait_for_gripper_wiggle(self.wiggle_acceleration, arm)  # m/s^2
        rospy.loginfo("OPEN")
        goal.goal.request = 5
        client.send_goal_and_wait(goal, rospy.Duration(30), rospy.Duration(5))

    def take_object(self, arm, client):
        rospy.loginfo("TAKING OBJECT")
        # move out to tucked-with-object approach pose for right arm
        goal = self.get_pickplace_goal(self.target_locations["give_"+arm])
        # goal.goal.object_pose.orientation.x = 0.5
        # goal.goal.object_pose.orientation.y = -0.5
        # goal.goal.object_pose.orientation.z = 0.5
        # goal.goal.object_pose.orientation.w = 0.5
        goal.goal.object_pose.orientation.x = 0.0
        goal.goal.object_pose.orientation.y = 0.0
        goal.goal.object_pose.orientation.z = 0.0
        goal.goal.object_pose.orientation.w = 1.0
        client.send_goal_and_wait(goal, rospy.Duration(30), rospy.Duration(5))
        rospy.loginfo("OPEN")
        goal.goal.request = 5
        client.send_goal_and_wait(goal, rospy.Duration(30), rospy.Duration(5))

        rospy.sleep(1.5)
        self.wait_for_gripper_wiggle(self.wiggle_acceleration, arm)  # m/s^2
        goal.goal.request = 4
        client.send_goal_and_wait(goal, rospy.Duration(30), rospy.Duration(5))
        rospy.loginfo("CLOSE")

    def wait_for_gripper_wiggle(self, accel, arm):
        """Waits for one year.  accel is in m/s^2, normal values are 6 and 10"""
        # contents of function ported from pr2_props
        goal = PR2GripperEventDetectorGoal()
        # use just acceleration as our contact signal
        # goal.command.trigger_conditions = 4
        goal.command.acceleration_trigger_magnitude = accel
        goal.command.slip_trigger_magnitude = 0.008  # slip gain
        rospy.loginfo("ARM %s", arm)
        if arm == "l":
            self.wiggle_client = self.l_gripper_wiggle_detector_client
        elif arm == "r":
            self.wiggle_client = self.r_gripper_wiggle_detector_client
        rospy.loginfo("WAIT FOR WIGGLE")
        self.wiggle_client.send_goal_and_wait(goal, rospy.Duration(30), rospy.Duration(20))

        rospy.loginfo(self.wiggle_client.get_result())

        rospy.loginfo("WIGGLE DONE")
        self.wiggle_client.cancel_goal()

    def get_pickplace_goal(self, location):
        pose = geometry_msgs.msg.Pose()
        pose.position.x = location[1]
        pose.position.y = location[2]
        pose.position.z = location[3]

        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 1

        result = PickPlaceGoal()
        result.goal.request = location[5]
        result.goal.header.frame_id = location[0]
        result.goal.object_pose = pose
        return result

    def pose_from_xytheta(self, location):
        x = location[1]
        y = location[2]
        theta = location[4]
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = '/map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0
        quat = tf.transformations.quaternion_from_euler(0, 0, theta)
        pose.pose.orientation = geometry_msgs.msg.Quaternion(*quat)
        return pose

    def euler_from_quaternion(self, quat):
        euler = tf.transformations.euler_from_quaternion(quat)
        return euler[2]

    def clear_costmaps(self):
        rospy.loginfo("clearing costmaps")
        try:
            self.clear_costmaps_client()
        except rospy.ServiceException, error:
            rospy.loginfo("Service call failed: %s" % error)

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

    def turn(self, direction):
        rospy.loginfo("Turning {}".format(direction))
        (trans,quat) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        rot = self.euler_from_quaternion(quat)
        rospy.loginfo("Position : {}".format(trans))
        rospy.loginfo("Initial angle: {}".format(rot))
        if direction == "left":
            delta_rot = 1.57
        elif direction == "right":
            delta_rot = -1.57

        rot += delta_rot
        # keep the positional angle between pi and -pi
        if rot > 3.14:
            rot = -3.14 + (rot - 3.14)
        elif rot < -3.14:
            rot = 3.14 - (rot + 3.14)

        rospy.loginfo("Final angle: {}".format(rot))
        goal = self.pose_from_xytheta(['map', trans[0], trans[1], trans[2], rot, None])
        self.navigate_to(goal)

    def visual_servo_to(self, object_goal_frame, nav_goal_pose):

        rospy.loginfo("I AM VISUAL SERVOING")

        thresh_x = 0.6
        thresh_y = 0.2

        theta = self.target_locations[object_goal_frame.replace('/','')][4]

        got_reading = False
        while got_reading == False:
            try:
                (trans,rot) = self.listener.lookupTransform('/base_link', object_goal_frame, rospy.Time(0))
                got_reading = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        dx = round(trans[0], 2)
        dy = round(trans[1], 2)
        (trans,rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        robot_x = round(trans[0], 2)
        robot_y = round(trans[1], 2)

        rospy.loginfo("OLD dX and dY: %f, %f", dx, dy)

        while abs(dx) > thresh_x or abs(dy) > thresh_y:

            refined_goal = nav_goal_pose

            if abs(dx) > thresh_x:
                dx -= thresh_x

            if abs(dy) > thresh_y:
                if dy < 0:
                    dy += thresh_y
                elif dy > 0:
                    dy -= thresh_y

            rospy.loginfo("BEFORE TRANSFORM %f, %f", dx, dy)
            deltas = self.transform(theta, [dx, dy])
            rospy.loginfo("AFTER TRANSFORM %f, %f", deltas[0], deltas[1])
            refined_goal.pose.position.x += deltas[0]
            refined_goal.pose.position.y += deltas[1]
            # rospy.sleep(1)

            self.navigate_to(refined_goal)

            got_reading = False
            while got_reading == False:
                try:
                    (trans,rot) = self.listener.lookupTransform('/base_link', object_goal_frame, rospy.Time(0))
                    got_reading = True
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
            dx = round(trans[0], 2)
            dy = round(trans[1], 2)
            (trans,rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            robot_x = round(trans[0], 2)
            robot_y = round(trans[1], 2)

            rospy.loginfo("OLD dX and dY: %f, %f", dx, dy)

    def transform(self,  theta, inputs):
        R = np.zeros((2,2))
        R[0,0] = cos(theta)
        R[0,1] = -sin(theta)
        R[1,0] = sin(theta)
        R[1,1] = cos(theta)
        result = np.dot(R, inputs)
        return result

if __name__ == '__main__':
    rospy.init_node('object_gather_server')
    server = ObjectGatherServer()
    rospy.spin()

#!/usr/bin/env python
import roslib;
import rospy
import time, sys, math

from ur_msgs.srv import SetPayload, SetIO
from ur_msgs.msg import *
from geometry_msgs.msg import Pose
import actionlib
import tf
import cmd
import yaml
from tf_conversions import posemath
import numpy
from tf import transformations

base_frame = "ur_base_link"
tool_frame = "ee_link"
#base_frame = "ur_base_link"
#tool_frame = "ur_ee_link"


def xyz_to_mat44(pos):
  return transformations.translation_matrix((pos.x, pos.y, pos.z))

def xyzw_to_mat44(ori):
   return transformations.quaternion_matrix((ori.x, ori.y, ori.z, ori.w))


class TestTeacher(cmd.Cmd):

    def __init__(self):
        self.listener = tf.TransformListener()
        self.current_traj = FollowCartesianTrajectoryGoal()
        self.current_traj.header.frame_id = base_frame
        self.client = actionlib.SimpleActionClient('follow_cart_trajectory', FollowCartesianTrajectoryAction)
        self.last_pose = None
        rospy.set_param('prevent_programming', True)
        cmd.Cmd.__init__(self)
        self.teach_mode = True
        self.teach_mode_rel = False

    def getCurrentPose(self):
        trans = [0,0,0]
        rot = [0,0,0,1]
        try:
            (trans,rot) = self.listener.lookupTransform(base_frame, tool_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "Error receiving pose"
            pass
        pose = Pose()
        pose.position.x = trans[0]
        pose.position.y = trans[1]
        pose.position.z = trans[2]
        pose.orientation.x = rot[0]
        pose.orientation.y = rot[1]
        pose.orientation.z = rot[2]
        pose.orientation.w = rot[3]
        return pose

    def do_tm(self, rel):
        if(self.teach_mode):
            print "Deactivating teach mode"
            rospy.set_param('prevent_programming', False)
            self.teach_mode = False
        else:
            print "Activating teach mode"
            rospy.set_param('prevent_programming', True)
            self.teach_mode = True
            if(rel == "rel"):
                self.teach_mode_rel = True
                print "Activate relative motion from here"
                self.last_pose = self.getCurrentPose()
                self.current_traj.header.frame_id = tool_frame
            else:
                print "Current motion will be global"
                self.current_traj.header.frame_id = base_frame

    def do_new(self, arg):
        self.current_traj = FollowCartesianTrajectoryGoal()
        self.current_traj.header.frame_id = base_frame

    def do_l(self, arg):
        print "-- Tool frame =", tool_frame, " ------- Base frame =", self.current_traj.header.frame_id, "--"
        for pose in self.current_traj.poses:
            [rx, ry, rz] = tf.transformations.euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
            print "|", float("{0:.4f}".format(pose.position.x)), "|", float("{0:.4f}".format(pose.position.y)), "|", float("{0:.4f}".format(pose.position.z)), "|", float("{0:.4f}".format(rx)), "|", float("{0:.4f}".format(ry)), "|", float("{0:.4f}".format(rz)), "|"

    def do_tool_frame(self, frame):
        global tool_frame
        tool_frame = frame
        print "Setting current tool frame to", tool_frame

    def do_base_frame(self, frame):
        global base_frame
        base_frame = frame
        print "Setting current base frame to", base_frame

    def do_t(self, arg):
        if(self.teach_mode):
            if(self.teach_mode_rel):
                print "Calculating relative motion"
                f1 = posemath.fromMsg(self.last_pose)
                current_global_pose = self.getCurrentPose()
                f2 = posemath.fromMsg(current_global_pose)
                diff =  f1 * f2.Inverse()
                print diff
                self.current_traj.poses.append(posemath.toMsg(diff))
            else:
                self.current_traj.poses.append(self.getCurrentPose())
        else:
            print "Please activate teach mode first using tm"

    def do_run(self, arg):
        if(not self.teach_mode):
            self.client.wait_for_server()
            self.client.send_goal(self.current_traj)
            print "Sended goal"
            self.client.wait_for_result(rospy.Duration.from_sec(25.0))
            print "Finished motion"
            #rospy.set_param('prevent_programming', True)
        else:
            print "Please deactivate teach mode first using tm"

    def do_save(self, filename):
        if(filename):
            try:
                with open(filename, 'w') as outfile:
                    outfile.write( self.current_traj.__str__() )
            except:
                pass
        else:
            print "Please specify a filename"

    def do_load(self, filename):
        if(filename):
            try:
                genpy.message.fill_message_args( self.current_traj, yaml.load(open(filename, 'r')))
            except:
                print "Error loading file"
        else:
            print "Please specify a filename"

    def do_EOF(self, arg):
        rospy.set_param('prevent_programming', False)
        return True

if __name__ == '__main__':
    rospy.init_node('ur_service_tester', disable_signals=True)
    TestTeacher().cmdloop()

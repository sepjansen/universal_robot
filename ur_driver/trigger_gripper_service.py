#!/usr/bin/env python

from std_srvs.srv import *
from ur_msgs.srv import *
import rospy

class GripperTrigger:
    def __init__(self):
        rospy.init_node('gripper_trigger')
        self.s = rospy.Service('trigger_gripper', Empty, self.handle_gripper_trigger)
        self.gripper_open = True
        self.io_srv = rospy.ServiceProxy('set_io', SetIO)

    def handle_gripper_trigger(self, req):
        if(self.gripper_open):
            r = SetIORequest()
            r.fun = 1
            r.pin = 0
            r.state = 1.0
            self.io_srv(r)
            self.gripper_open = False
        else:
            r = SetIORequest()
            r.fun = 1
            r.pin = 0
            r.state = 0.0
            self.io_srv(r)
            self.gripper_open = True
        rospy.sleep(0.5)
        return EmptyResponse()



if __name__ == "__main__":
    g = GripperTrigger()
    rospy.spin()

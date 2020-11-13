#!/usr/bin/env python

import sys
import rospy
from bingham_registration.srv import *
import timeit

class registration_client:
    def __init__(self):
        rospy.wait_for_service('stereo/bingham_registration')
        bingham_registration = rospy.ServiceProxy('stereo/bingham_registration',
                                                  RegistrationService)

    def update(self):
        pass;

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print usage()
        sys.exit(1)
    print "Requesting %s+%s"%(x, y)
    reg = registration_client();
    # print "%s + %s = %s"%(x, y, add_two_ints_client(x, y))
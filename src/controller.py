#!/usr/bin/env python

import math
import sys

import rospy
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute


if __name__ == '__main__':
    # start node
    rospy.init_node('controller')
    # start services
    rospy.wait_for_service('turtle1/teleport_absolute')
    teleport = rospy.ServiceProxy('turtle1/teleport_absolute', TeleportAbsolute)

    delta = 0.01 # rate at which velocity is updated
    r = rospy.Rate(1/delta) # set rate in hertzs
    t = 0

    # check command line arguments to the program
    if len(sys.argv) == 2: # check if the right number of arguments have been passed
        try:
            T = float(sys.argv[1])
            # make sure time passed in is greater than plotting rate
            if T < delta :
                raise ValueError('Please pick "Time" greater than ' + str(delta) + ' seconds')
        except (ValueError, TypeError), e:
             print 'Error: ', e
             exit(3)
    else:
        # default time
        T = 5 #seconds

    middle = 5.544445  # middle of turtle track

    while not rospy.is_shutdown():
        # set turtle postion using reference trajectory
        x = middle +  3 * math.sin(4*math.pi*t/T)
        y = middle + 3 * math.sin(2*math.pi*t/T)
        teleport(x, y, 0)

        # update time
        t += delta
        if t > T:
            t = 0
        r.sleep()

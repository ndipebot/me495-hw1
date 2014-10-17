#!/usr/bin/env python

import math
import sys
import rospy
from turtlesim.srv import TeleportAbsolute
from geometry_msgs.msg import Twist


if __name__ == '__main__':
    # start node
    rospy.init_node('controller')
    # start services
    rospy.wait_for_service('turtle1/teleport_absolute')
    teleport = rospy.ServiceProxy('turtle1/teleport_absolute', TeleportAbsolute)
    #publish to cmd_velocity
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    #adjust rate at which velocity is updated
    delta = 0.001
    r = rospy.Rate(1/delta) # set rate in hertzs
    t = delta

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

    #starting position
    middle = 5.544445

    #place turtle in the middle and set a starting angle
    theta1 = math.atan2(6*math.pi/T,12*math.pi/T)
    teleport(middle, middle, theta1)

    #define message
    twist = Twist()

    while not rospy.is_shutdown():
        #velocity = sqrt(xdot^2 + ydot^2), expression derived using matlab
        velocity = 6*math.pow(((math.pow(math.pi,2)*math.pow(math.cos((2*math.pi*t)/T),2))/math.pow(T,2) + (4*math.pow(math.pi,2)*math.pow(math.cos((4*math.pi*t)/T),2))/math.pow(T,2)),.5)

        #derivatives of the positions
        xdot = (12*math.pi*math.cos((4*math.pi*t)/T))/T
        ydot = (6*math.pi*math.cos((2*math.pi*t)/T))/T
        #current angle
        theta2 = math.atan2(ydot, xdot)
        #angular speed, omega = d(theta)/dt
        w = (theta2 - theta1)/delta
        #publish speed
        twist.linear.x = velocity
        twist.angular.z = w
        pub.publish(twist)
        #make current speed, previous speed for next loop
        theta1 = theta2

        # update time
        t += delta
        if t > T:
            t = 0
        r.sleep()

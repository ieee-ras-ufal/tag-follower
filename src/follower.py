#!/usr/bin/env python2

import rospy
import numpy
from tf.msg import tfMessage
from Driver import Driver
import time

Ts = 0.1

class Follower:

    def __cb_tag(self, msg):
        tag = numpy.array([msg.transforms.transform.translation.x, msg.transforms.transform.translation.y, msg.transforms.transform.translation.z])
        self.tag_position = self.__to_origin(tag)

    def __to_origin(self, position):
        T_E_0 = numpy.linalg.inv(self.fkm())

        result = numpy.dot(T_E_0, numpy.append(position, 1))

        return result[0:3]

    def position(self, theta_1 = None, theta_2 = None):
        if (theta_1 == None):
            theta_1 = self.theta[0]
        if (theta_2 == None):
            theta_2 = self.theta[1]
        
        kine = self.fkm(theta_1, theta_2)

        return kine[0:3, 3]

    def fkm(self, theta_1 = None, theta_2 = None):
        if (theta_1 == None):
            theta_1 = self.theta[0]
        if (theta_2 == None):
            theta_2 = self.theta[1]

        s1 = numpy.sin(theta_1)
        s2 = numpy.sin(theta_2)
        c1 = numpy.cos(theta_1)
        c2 = numpy.cos(theta_2)

        L1 = self.L[0]
        L2 = self.L[1]
        L3 = self.L[2]
        L4 = self.L[3]

        T_0_E = numpy.array([[c1*s2, -s1, c1*c2, L2*c1 + L3*c1*c2 - L4*c1*s2],
        [s1*s2, c1, s1*c2, L2*s1 + L3*s1*c2 - L4*s1*s2],
        [-c2, 0, s2, L1 + L3*s2 + L4*c2],
        [0, 0, 0, 1]])

        return T_0_E

    def jacobian(self, theta_1 = None, theta_2 = None):
        if (theta_1 == None):
            theta_1 = self.theta[0]
        if (theta_2 == None):
            theta_2 = self.theta[1]

        s1 = numpy.sin(theta_1)
        s2 = numpy.sin(theta_2)
        c1 = numpy.cos(theta_1)
        c2 = numpy.cos(theta_2)

        L2 = self.L[1]
        L3 = self.L[2]
        L4 = self.L[3]

        J = numpy.array([[-L2*s1 - L3*s1*c2 + L4*s1*s2, -L3*c1*s2 - L4*c1*c2],
        [L2*c1 + L3*c1*c2 - L4*c1*s2, -L3*s1*s2 -L4*s1*c2],
        [0, L3*c2 - L4*s2]])

        return J

    def __init__(self):
        rospy.init_node('follower')
        rospy.Subscriber('/ar_pose_marker', tfMessage, self.__cb_tag)
        self.theta = numpy.zeros(2)
        self.L = numpy.array([0.093, 0.025, 0.0257, 0.095])
        self.tag_position = self.position(-numpy.pi/4, -numpy.pi/6)

follower = Follower()

# Driver instance
driver = Driver()

# Registering shutdown hook
def shutdown_hook():
    driver.closeConn()

# TEST
driver.set_theta([numpy.pi/9, numpy.pi/9])
time.sleep(2)

# Main loop
rospy.on_shutdown(shutdown_hook)
rate = rospy.Rate(1/Ts)
while not rospy.is_shutdown():

    # Computing feedback control
    error = follower.tag_position - follower.position()
    dtheta = numpy.dot(numpy.linalg.pinv(follower.jacobian()), error)

    # Euler integration
    theta = follower.theta + dtheta * Ts
    
    ## Apply on servo
    driver.set_theta(theta)

    rate.sleep()

    follower.theta = theta
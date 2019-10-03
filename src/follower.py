#!/usr/bin/env python2

import rospy
import numpy
from tf.msg import tfMessage
from Driver import Driver

Ts = .1

class Follower:

    def __cb_tag(self, msg):
        tag = numpy.array([msg.transforms[0].transform.translation.x, msg.transforms[0].transform.translation.y, msg.transforms[0].transform.translation.z])
        self.tag_position = self.__to_origin(tag)

    def __to_origin(self, position):
        result = numpy.dot(self.fkm(), numpy.append(position, 1))

        return result[0:3]

    def position(self, theta_1 = None, theta_2 = None):
        if (theta_1 == None):
            theta_1 = self.theta[0]
        if (theta_2 == None):
            theta_2 = self.theta[1]

	L1 = self.L[0]
	L2 = self.L[1]
	L3 = self.L[2]
	L4 = self.L[3]

	s1 = numpy.sin(theta_1)
	s2 = numpy.sin(theta_2)
	c1 = numpy.cos(theta_1)
	c2 = numpy.cos(theta_2)
        
        kine = numpy.array([L2*c1 - L3*c1*s2 + L4*c1*c2, L2*s1 - L3*s1*s2 + L4*s1*c2, L1 + L3*c2 + L4*s2])

        return kine

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

        T_0_E = numpy.array([[s1, c1*s2, c1*c2, L2*c1 - L3*c1*s2 + L4*c1*c2],
        [-c1, s1*s2, s1*c2, L2*s1 - L3*s1*s2 + L4*s1*c2],
        [0, -c2, s2, L1 + L3*c2 + L4*s2],
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

        J = numpy.array([[-L2*s1 +  L3*s1*s2 - L4*s1*c2, -L3*c1*c2 - L4*c1*s2],
        [L2*c1 - L3*c1*s2 + L4*c1*c2, -L3*s1*c2 -L4*s1*s2],
        [0, -L3*s2 + L4*c2]])

        return J

    def __init__(self):
        rospy.init_node('follower')
        rospy.Subscriber('/camera_cube_tf', tfMessage, self.__cb_tag)
        self.theta = numpy.zeros(2)
        self.L = numpy.array([0.093, 0.025, 0.0257, 0.095])
	self.tag_position = self.position()

follower = Follower()

# Driver instance
driver = Driver(follower.theta)

# Registering shutdown hook
def shutdown_hook():
    driver.closeConn()

# Main loop
cnt = 0
rospy.on_shutdown(shutdown_hook)
rate = rospy.Rate(1/Ts)
while not rospy.is_shutdown():

    # Computing feedback control
    goal = numpy.copy(follower.tag_position)
    #goal = numpy.array([1, 0.5, 0.5])
    goal[2] -= follower.L[0]
    versor = goal/numpy.linalg.norm(goal)
    setpoint = (follower.L[1] + follower.L[3]) * versor
    setpoint[2] += follower.L[0]
    error = setpoint - follower.position()

    dtheta = numpy.dot(numpy.linalg.pinv(follower.jacobian()), error)

    # Euler integration
    theta = follower.theta + dtheta * Ts
    
    ## Apply on servo
    driver.set_theta(theta)

    rate.sleep()

    follower.theta = theta

    cnt += 1
    if (cnt % 10):
	print(follower.tag_position)
	cnt = 0

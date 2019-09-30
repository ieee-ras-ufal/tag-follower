import pigpio
import numpy

class Driver:    
    BASE_PIN = 17
    TILT_PIN = 27

    base_limits = [1100, 1900]
    tilt_limits = [1300, 2000]

    base_angles = [-45, 45]
    tilt_angles = [-30, 30]

    def __mapping(self, value, anglesLimits, limits):
        span = anglesLimits[1] - anglesLimits[0]
        
        scaled = float(value - anglesLimits[0])/float(span)

        # A little limit to avoid miscalculation
        if scaled > 1:
            scaled = 1
        elif scaled < 0:
            scaled = 0

        return limits[0] + (scaled * (limits[1] - limits[0]))

    def set_theta(self, theta):
        theta1 = self.__mapping(theta[0], base_angles, base_limits)
        theta2 = self.__mapping(theta[1], tilt_angles, tilt_limits)

        self.pi.set_servo_pulsewidth(BASE_PIN, theta1)
        self.pi.set_servo_pulsewidth(TILT_PIN, theta2)

    def __init__(self):
        self.pi = pigpio.pi()
        self.pi.set_mode(BASE_PIN, pigpio.OUTPUT)
        self.pi.set_mode(TILT_PIN, pigpio.OUTPUT)

    def closeConn(self):
        self.pi.stop()
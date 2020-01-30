#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import sys
import numpy as np
from gazebo_swarm.msg import State, Control
import tf
from sensor_msgs.msg import Joy
from simple_pid import PID

class ControlGazebo:
    def __init__(self):

        rospy.init_node('ControlSystem', anonymous=True)

        self.pub = rospy.Publisher('/Skyhunter_Plugin/Control', Control, queue_size=10)
        self.rate = rospy.Rate(1200) # 10hz

        self.subpose = rospy.Subscriber('/Skyhunter_Plugin/State', State, self.PosCallback)

        self.subjoy = rospy.Subscriber("/joy", Joy, self.JoyCallback)


        self.Pos_x = 0
        self.Pos_y = 0
        self.Pos_z = 0

        self.Vel_x = 0
        self.Vel_y = 0
        self.Vel_z = 0

        self.Roll = 0
        self.Pitch = 0
        self.Yaw = 0

        self.p = 0
        self.q = 0
        self.r = 0

        self.pid_motor = PID(1, 0, 0, setpoint=0)
        self.pid_roll = PID(0.008, 0, 0, setpoint=0)
        self.pid_pitch = PID(0.9, 0, 0, setpoint=0)
        self.pid_yaw = PID(1, 0, 0, setpoint=0)
        self.pid_height = PID(0.1,0,0, setpoint=0)

    def JoyCallback(self, data):
        #print data.axes
        pass

    def PosCallback(self, data):

        self.Pos_x = data.position.x
        self.Pos_y = data.position.y
        self.Pos_z = data.position.z

        quaternion = (data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)

        (self.Roll, self.Pitch, self.Yaw) = tf.transformations.euler_from_quaternion(quaternion)

        self.Vel_x = data.linearvel.x
        self.Vel_y = data.linearvel.y
        self.Vel_z = data.linearvel.z

        self.p = data.angularvel.x
        self.q = data.angularvel.y
        self.r = data.angularvel.z

    def saturate(self, input):
        if (input > 1):
            input = 1
        elif (input < -1):
            input = -1
        return input

    def Run(self):
        msg=Control()

        self.rate.sleep()

        msg.throttle = 0.75
        msg.roll = 0
        msg.pitch = 0
        msg.yaw = 0

        self.pid_height.setpoint = -60

        while not rospy.is_shutdown():

            #print "Height = ", self.Pos_z
            print "Roll = ", self.Roll
            #print("Pitch = ", self.Pitch)
            #print("Yaw = ", self.Yaw)

            #print("p = ", self.p)
            #print("q = ", self.q)
            #print("r = ", self.r)

            self.pid_pitch.setpoint = self.pid_height(self.Pos_z)
            msg.pitch = self.saturate(-1*self.pid_pitch(self.Pitch*(180/np.pi)))


            rollcmd = self.saturate(self.pid_roll(self.Roll*(180/np.pi)))
            #print "Cmd value =", rollcmd
            msg.roll = rollcmd

            self.pub.publish(msg)
            self.rate.sleep()

if __name__ == '__main__':
    control = ControlGazebo()
    control.Run()

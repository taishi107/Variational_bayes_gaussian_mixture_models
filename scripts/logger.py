#!/usr/bin/env python
# Copyright 2017 Masahiro Kato
# Copyright 2017 Ryuichi Ueda
# Released under the BSD License.
import signal
import sys

import rospy, rosbag, rosparam
import math, sys, random, datetime
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from raspimouse_ros.msg import LightSensorValues
from variational_bayes_gaussian_mixture_models.msg import LightSensorValues2
class Logger():
    def __init__(self):
        self.sensor_values = LightSensorValues()
        self.pub = rospy.Publisher('read_bagfile',LightSensorValues,queue_size=100)
        rospy.Subscriber('/lightsensors', LightSensorValues, self.sensor_callback)

        self.on = True
        self.bag_open = False

    def sensor_callback(self,messages):
        self.sensor_values = messages

    def output_decision(self):
        if not self.on:
            if self.bag_open:
                print ("----------------------")
                print ("Stop.")
                self.bag.close()
                self.bag_open = False
            return
        else:
            if not self.bag_open:
                filename ='sensor_values.bag'
                #rosparam.set_param("/current_bag_file", filename)
                self.bag = rosbag.Bag(filename, 'w')
                self.bag_open = True
                print ("----------------------")
                print ("Start recoding.")
        s1 = self.sensor_values
        s2 = LightSensorValues()
        
        s2.left_side = s1.left_side
        s2.right_side = s1.right_side
        s2.left_forward = s1.left_forward
        s2.right_forward = s1.right_forward
        self.pub.publish(self.sensor_values)
        self.bag.write('/event', self.sensor_values)
    def run(self):
        rate = rospy.Rate(10)
        data = Twist()
        signal.signal(signal.SIGINT, self.handler)
        while not rospy.is_shutdown():
            self.output_decision()
            rate.sleep()

    def handler(self, signal, frame):
        print('Finish recoding')
        self.bag.close()
        sys.exit(0)

if __name__ == '__main__':
    rospy.init_node('logger')
    Logger().run()

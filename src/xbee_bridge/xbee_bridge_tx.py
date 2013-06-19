#!/usr/bin/python

#####################################################
#                                                   #
#   Copyright (c) 2013, Calder Phillips-Grafflin    #
#                                                   #
#   Alternative ROS transport method over serial    #
#                                                   #
#####################################################

import rospy
import serial
import StringIO
import threading
from copy import deepcopy

class XbeeBridge:

    def __init__(self, port, rate, input_topic, input_topic_type, max_queue=5):
        [self.topic_type, self.topic_package] = self.extract_type_and_package(input_topic_type)
        self.dynamic_load(self.topic_package)
        self.xbee = serial.Serial(port, rate, 8, 'N', 1, timeout=0.0)
        self.queue = [None for index in range(max_queue)]
        self.cur_index = 0
        self.max_index = max_queue - 1
        self.queue_lock = threading.Lock()
        self.sub = rospy.Subscriber(input_topic, eval(self.topic_type), self.sub_cb)
        rospy.loginfo("Loaded Xbee bridge [TX]")
        while not rospy.is_shutdown():
            self.send()

    def send(self):
        self.queue_lock.acquire()
        msg = self.queue[self.cur_index - 1]
        if (msg is not None):
            msg = deepcopy(msg)
            self.queue[self.cur_index - 1] = None
            self.queue_lock.release()
            buff = StringIO.StringIO()
            buff.write("<xbtxfs>")
            msg.serialize(buff)
            buff.write("</xbtxe>")
            self.xbee.write(buff.getvalue())
            buff.close()
        else:
            self.queue_lock.release()

    def sub_cb(self, msg):
        self.queue_lock.acquire()
        self.queue[self.cur_index] = msg
        if (self.cur_index < self.max_index):
            self.cur_index += 1
        else:
            self.cur_index = 0
        self.queue_lock.release()

    def extract_type_and_package(self, input_topic_type):
        topic_package = input_topic_type.split("/")[0]
        topic_type = input_topic_type.split("/")[1]
        return [topic_type, topic_package]

    def dynamic_load(self, package_name):
        rospy.loginfo("Dynamic loading messages from package: " + package_name)
        eval_str = "from " + package_name + ".msg import *"
        exec(eval_str, globals())

if __name__ == '__main__':
    rospy.init_node('xbee_bridge_tx')
    topic = rospy.get_param("~topic", "test/Test")
    topic_type = rospy.get_param("~topic_type", "std_msgs/String")
    port = rospy.get_param("~port", "/dev/ttyUSB0")
    rate = rospy.get_param("~rate", 9600)
    queue = rospy.get_param("~queue_size", 5)
    XbeeBridge(port, rate, topic, topic_type, queue_size)

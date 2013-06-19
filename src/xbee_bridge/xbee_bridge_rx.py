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

class XbeeBridge:

    def __init__(self, port, rate, output_topic, output_topic_type):
        [self.topic_type, self.topic_package] = self.extract_type_and_package(output_topic_type)
        self.dynamic_load(self.topic_package)
        self.xbee = serial.Serial(port, rate, 8, 'N', 1, timeout=0.0)
        self.pub = rospy.Publisher(output_topic, eval(self.topic_type))
        rospy.loginfo("Loaded Xbee bridge [RX]")
        self.buffer = ""
        while not rospy.is_shutdown():
            self.buffer += self.xbee.read(1)
            sx = self.buffer.find("<xbtxfs>")
            ex = self.buffer.find("</xbtxe>")
            if (sx != -1 and ex != -1):
                raw_data = self.buffer[sx + 8:ex]
                self.clean_and_pub(raw_data)
                try:
                    self.buffer = self.buffer[ex + 8:]
                except:
                    self.buffer = ""
        
    def clean_and_pub(self, raw):
        new_msg = eval(self.topic_type)()
        new_msg.deserialize(raw)
        self.pub.publish(new_msg)

    def extract_type_and_package(self, input_topic_type):
        topic_package = input_topic_type.split("/")[0]
        topic_type = input_topic_type.split("/")[1]
        return [topic_type, topic_package]

    def dynamic_load(self, package_name):
        rospy.loginfo("Dynamic loading messages from package: " + package_name)
        eval_str = "from " + package_name + ".msg import *"
        exec(eval_str, globals())

if __name__ == '__main__':
    rospy.init_node('xbee_bridge_rx')
    topic = rospy.get_param("~topic", "test/Test")
    topic_type = rospy.get_param("~topic_type", "std_msgs/String")
    port = rospy.get_param("~port", "/dev/ttyUSB0")
    rate = rospy.get_param("~rate", 9600)
    XbeeBridge(port, rate, topic, topic_type)

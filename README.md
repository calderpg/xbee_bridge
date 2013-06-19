xbee_bridge
===========

ROS Message transport over Xbee-compatible serial radios, Catkinized for ROS Groovy+.

This package enables ROS messages to be sent transparently between two computers (and, in fact, two ROS masters) using a serial link. This is done by using the same message serialization/deserialization system used internally in ROS to convert the messages to and from raw bytes that can be sent over serial.

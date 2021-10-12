#!/usr/bin/env python
import serial
import struct

import rospy
from geometry_msgs.msg import WrenchStamped, Wrench, Vector3


class serial_connection ():

    def __init__(self):
        prefix = rospy.get_name() + '/'
        self.port = rospy.get_param(prefix + "port", default="/dev/ttyACM0")
        self.baudrate = rospy.get_param(prefix + "baudrate", default=2000000)
        frequency = rospy.get_param(prefix + "frequency", default=500)
        
        topic = rospy.get_param(prefix + "topic", default="resense_ft/wrench")
        topic = rospy.get_namespace() + topic

        rospy.loginfo("Publishing wrench to: " + topic)

        self.publisher = rospy.Publisher(topic, WrenchStamped, queue_size=10)
        self.publisher_rate = rospy.Rate(frequency)
        self.is_published = False

    def connect(self):

        dataNumBytes = 4
        numData = 7
        data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        rawData = bytearray(numData * dataNumBytes)

        try:

            rospy.loginfo("Connecting to sensor port: " + self.port)
            self.serial_object = serial.Serial(self.port, self.baudrate, timeout=1)

            try:
                rospy.loginfo("Publishing FT data, Press ctrl + c to stop")
                while not rospy.is_shutdown():

                    now = rospy.get_rostime()
                    self.serial_object.readinto(rawData)
                    for i in range(numData):
                        bytedata = rawData[(i * dataNumBytes):(dataNumBytes + i * dataNumBytes)]
                        value, = struct.unpack('f', bytedata)
                        data[i] = value

                    msg = WrenchStamped()
                    msg.header.stamp = now
                    wrench = Wrench()
                    wrench.force = Vector3(array[:3])
                    wrench.torque = Vector3(array[3:])
                    msg.wrench = wrench

                    self.publisher.publish(msg)
                    self.publisher_rate.sleep()

                self.serial_object.close()
            except Exception as e:
                rospy.logerror("Something is wrong" + str(e))
                self.serial_object.close()

        except Exception as e:
            rospy.logerror("Error: No Connection!" + str(e))


def main():
    rospy.init_node('wittenstain_publisher', anonymous=True)

    serial_connection_obj = serial_connection()
    serial_connection_obj.connect()


if __name__ == "__main__":

    try:
        main()
    except rospy.ROSInterruptException:
        pass

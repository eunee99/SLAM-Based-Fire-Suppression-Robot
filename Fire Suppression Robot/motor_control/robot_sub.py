#!/usr/bin/env python
import paho.mqtt.client as mqtt
import rospy
from std_msgs.msg import Int8

class fire_detector:
    def __init__(self):
        rospy.init_node('fire_detector')
        self.pub = rospy.Publisher('detection_flag', Int8)
        self.server = "13.125.77.88"
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(self.server, 1883, 60)

    def on_connect(self, client, userdata, flags, rc):
        print("Connected with RC : " + str(rc))
        client.subscribe("#")

    def on_message(self, client, userdata, msg):
        if(msg.topic == "deviceid/juju/cmd/flame"):
            value = float(msg.payload.decode('utf-8'))
            print(msg.topic + " " + msg.payload.decode('utf-8'))
            if(value == 1): #flame detect
                self.publish(1)
                print("1")

            else:
                self.publish(0)
                print("0")

    def loop_forever(self):
        self.client.loop_forever()

if __name__ == '__main__':
    fd = fire_detector()
    fd.loop_forever()
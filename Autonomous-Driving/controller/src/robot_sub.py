#!/usr/bin/env python
import paho.mqtt.client as mqtt

class fire_detector:
    def __init__(self):
        self.server = "13.125.77.88"
        self.detection_flag = 0
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
                self.detection_flag = 1
                print("1")
                exit()

            else:
                self.detection_flag = 0
                print("0")

    def loop_forever(self):
        self.client.loop_forever()

if __name__ == '__main__':
    fd = fire_detector()
    fd.loop_forever()
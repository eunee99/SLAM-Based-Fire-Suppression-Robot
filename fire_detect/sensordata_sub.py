import paho.mqtt.client as mqtt
server = "13.125.77.88"
def on_connect(client, userdata, flags, rc):
    print("Connected with RC : " + str(rc))
    client.subscribe("#")

def on_message(client, userdata, msg):
    print(msg.topic+" "+msg.payload.decode('utf-8'))

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(server, 1883, 60)
client.loop_forever()

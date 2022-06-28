import paho.mqtt.client as mqtt
server = "13.125.77.88"
def on_connect(client, userdata, flags, rc):
    print("Connected with RC : " + str(rc))
    client.subscribe("#")

def on_message(client, userdata, msg):
    if(msg.topic == "deviceid/juju/evt/flame"):
        flamevalue = float(msg.payload.decode('utf-8'))
        print(msg.topic+" "+msg.payload.decode('utf-8'))
        if(flamevalue == 1):
            print("화재 경보")
            client.publish("deviceid/juju/cmd/flame", "1") # 화재감지기로부터 값을 받아오면 robot에게 publish
        else:
            print("안전")
            client.publish("deviceid/juju/cmd/flame", "0")

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(server, 1883, 60)
client.loop_forever()

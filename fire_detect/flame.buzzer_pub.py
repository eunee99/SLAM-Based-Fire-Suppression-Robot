import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO
import time

subtopic = "#"
pubtopic0 = "deviceid/juju/evt/flame"

server = "13.125.77.88"

FLAME = 17  # BCM. 17, wPi. 0, Physical. 11(DOUT에 연결)  #flame
gpio_pin = 12  # buzzer
scale = [261, 294, 329, 349, 392, 440, 493, 523]
list = [7, 5, 7, 5, 7, 5, 7, 5, 7, 5, 7, 5]

GPIO.setmode(GPIO.BCM)
GPIO.setup(FLAME, GPIO.IN)
GPIO.setup(gpio_pin, GPIO.OUT)


# The callback function. It will be triggered when trying to connect to the MQTT broker
# client is the client instance connected this time
# userdata is users' information, usually empty. If it is needed, you can set it through user_data_set function.
# flags save the dictionary of broker response flag.
# rc is the response code.
# Generally, we only need to pay attention to whether the response code is 0.

def on_connect(client, userdata, flags, rc):
    print("Connected with RC : " + str(rc))
    client.subscribe(subtopic)


client = mqtt.Client()
client.connect(server, 1883, 60)
client.on_connect = on_connect

client.loop_start()
time.sleep(1)

while (1):

    now = time.localtime()
    timestamp = ("%04d-%02d-%02d %02d:%02d:%02d" %
                 (now.tm_year, now.tm_mon, now.tm_mday,
                  now.tm_hour, now.tm_min, now.tm_sec))
    if GPIO.input(FLAME) == 1:  # 평소 1을 전송함
        print(timestamp, "안전")
        client.publish(pubtopic0, "0")  # 불꽃 감지 x : 0 보냄
    else:  # 0 을 전송함
        print(timestamp, "화재 경보")
        client.publish(pubtopic0, "1")  # 불꽃 감지 o : 1 보냄
        try:  # buzzer on
            p = GPIO.PWM(gpio_pin, 100)
            p.start(100)
            p.ChangeDutyCycle(90)

            for i in range(12):
                p.ChangeFrequency(scale[list[i]])
                if i == 6:
                    time.sleep(1)
                else:
                    time.sleep(0.5)
            p.stop()
        finally:
            time.sleep(1)
    time.sleep(1)

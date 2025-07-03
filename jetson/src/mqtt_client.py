import threading
import paho.mqtt.client as mqtt
from arduino_connection_test import ArduinoConnection
import time

# Constants
CMD_CONTROL = "Control"
CMD_STOP = "Stop_control"

class MQTTClientJetson:
    def __init__(self, arduino_connection: ArduinoConnection = None, broker_address="192.168.1.3", port=1883):
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)  # type: ignore
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_disconnect = self.on_disconnect

        self.arduino_connection = arduino_connection

        try:
            self.client.connect(broker_address, port, 60)
        except Exception as e:
            raise ConnectionError(f"Failed to connect to MQTT broker: {e}")

        self.client.loop_start()

        self.stop_control = False
        self.elevator = 0
        self.jetson_state = "None"
        self.pi_state = "None"

    def on_connect(self, client, userdata, flags, rc, *args):
        print("Connected with result code " + str(rc))
        self.client.subscribe("handshake/request")
        self.client.subscribe("jetson/command")
        self.client.subscribe("jetson/state")
        self.client.subscribe("arduino/elevator")
        self.client.subscribe("pi/response")

    def on_disconnect(self, client, userdata, rc):
        print(f"Disconnected from broker with code {rc}, attempting to reconnect...")
        while True:
            try:
                self.client.reconnect()
                print("Reconnected to MQTT broker.")
                break
            except Exception as e:
                print(f"Reconnect failed: {e}")
                time.sleep(5)

    def on_message(self, client, userdata, msg):
        payload = msg.payload.decode()
        topic = msg.topic
        print(f"Message received on topic '{topic}': {payload}")

        if topic == "handshake/request":
            if payload == "pi":
                print("Received handshake request from Pi")
                self.client.publish("handshake/response", "ack", qos=1)
                self.client.publish("pi/command", "booted", qos=1)

        elif topic == "jetson/command":
            if payload == CMD_STOP:
                self.stop_control = True
                print("Stop control activated.")
            elif payload == CMD_CONTROL:
                self.stop_control = False
                print("Control resumed.")
            else:
                print(f"Executing custom command: {payload}")
                # Execute or forward command immediately here if needed
                # Example: self.arduino_connection.send_command(payload)

        elif topic == "jetson/state":
            self.jetson_state = payload

        elif topic == "pi/response":
            self.pi_state = payload

        elif topic == "arduino/elevator":
            self.elevator = payload

    def publish_image(self, image):
        self.client.publish("camera/feed", image, qos=0)

    def publish_ball_info(self, ball_info):
        self.client.publish("ball/info", ball_info, qos=0)

    def stop(self):
        print("Stopping Jetson MQTT client...")
        self.client.loop_stop()
        self.client.disconnect()

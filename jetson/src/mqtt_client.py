import threading
import paho.mqtt.client as mqtt
from arduino_connection_test import ArduinoConnection
from queue import Queue, Empty
import time

# Constants
STATE_IDLE = "0.0"
STATE_MOVING = "1.0"
STATE_DONE = "2.0"
CMD_CONTROL = "Control"
CMD_STOP = "Stop_control"

class MQTTClientJetson(threading.Thread):
    def __init__(self, arduino_connection: ArduinoConnection = None, broker_address="192.168.1.3", port=1883):
        super().__init__()
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

        # Internal state
        self.jetson_state = STATE_IDLE
        self.elevator = 0
        self.stop_control = False
        self.command_queue = Queue()
        self.running = True

        self.start()

    def on_connect(self, client, userdata, flags, rc, *args):
        print("Connected with result code " + str(rc))
        self.client.subscribe("handshake/request")
        self.client.subscribe("jetson/command")
        self.client.subscribe("jetson/state")
        self.client.subscribe("arduino/elevator")
        self.client.subscribe("pi/response")  # Added since you're handling it

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

        if topic == "jetson/state":
            self.jetson_state = payload

        elif topic == "jetson/command":
            if payload in [STATE_IDLE, STATE_MOVING, STATE_DONE]:
                self.jetson_state = payload
                self.client.publish("pi/state", payload, qos=1)
            elif payload == CMD_STOP:
                self.stop_control = True
            elif payload == CMD_CONTROL:
                self.stop_control = False
            else:
                self.command_queue.put(payload)

        elif topic == "handshake/request":
            if payload == "pi":
                print("Received handshake request from Pi")
                self.client.publish("handshake/response", "ack", qos=1)
                self.client.publish("pi/command", STATE_IDLE, qos=1)

        elif topic == "pi/response":
            self.pi_state = payload

        elif topic == "arduino/elevator":
            self.elevator = payload

    def publish_image(self, image):
        self.client.publish("camera/feed", image, qos=0)

    def publish_ball_info(self, ball_info):
        self.client.publish("ball/info", ball_info, qos=0)

    def run(self):
        while self.running:
            try:
                command = self.command_queue.get(timeout=1)
                print(f"Processing command from queue: {command}")
                # Handle command if needed
            except Empty:
                continue

    def stop(self):
        print("Stopping Jetson MQTT client...")
        self.running = False
        self.client.loop_stop()
        self.client.disconnect()
import threading
import paho.mqtt.client as mqtt
from arduino_connection_test import ArduinoConnection
from queue import Queue, Empty
import time

# Constants
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
        self.command_queue = Queue()
        self.running = True

        self.handshake_complete = False

        try:
            self.client.connect(broker_address, port, 60)
        except Exception as e:
            raise ConnectionError(f"Failed to connect to MQTT broker: {e}")

        self.client.loop_start()
        self.start()  # Start the thread that processes the command queue

    def on_connect(self, client, userdata, flags, rc, *args):
        print("Connected with result code " + str(rc))
        self.client.subscribe("handshake/request")
        self.client.subscribe("jetson/command")
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
                self.handshake_complete = True

        elif topic == "jetson/command":
            self.command_queue.put(payload)

        elif topic == "arduino/elevator":
            print(f"Elevator value received: {payload}")

        elif topic == "pi/response":
            print(f"Pi response received: {payload}")

    def run(self):
        while self.running:
            try:
                command = self.command_queue.get(timeout=1)
                self.process_command(command)
            except Empty:
                continue

    def process_command(self, command):
        print(f"Processing command: {command}")

        if command == CMD_CONTROL:
            print("Control resumed.")
            # Example: self.arduino_connection.resume()
        elif command == CMD_STOP:
            print("Stop control activated.")
            # Example: self.arduino_connection.stop()
        else:
            print(f"Forwarding custom command to Arduino: {command}")
            if self.arduino_connection:
                self.arduino_connection.send_command(command)

    def publish_image(self, image):
        self.client.publish("camera/feed", image, qos=0)

    def publish_ball_info(self, ball_info):
        self.client.publish("ball/info", ball_info, qos=0)

    def stop(self):
        print("Stopping Jetson MQTT client...")
        self.running = False
        self.client.loop_stop()
        self.client.disconnect()

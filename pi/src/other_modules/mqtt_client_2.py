import threading
import paho.mqtt.client as mqtt
import time
import base64
import numpy as np
import cv2
from other_modules.ui import tuning_screen

class MQTTClientPi(threading.Thread):
    def __init__(self, broker_address='192.168.1.3', port=1883):
        super().__init__()
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2) # type: ignore
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        self.broker_address = broker_address
        self.port = port
        
        self.app = None
        self.handshake_complete = False
        self.pi_state = "None"
        self.img = None
        self.jetson_path = "None"
        self.screen2_instance = None
        self.ball_found = None
        self.img = None
        self.timeout = False
        self.path_found = False
        
        self.connected = False
        self.retry_interval = 1  # Initial retry interval in seconds
        
        #self.connect_to_broker()
        threading.Thread(target=self.connect_to_broker, daemon=True).start()

    def connect_to_broker(self):
        while not self.connected:
            try:
                print(f"[MQTT] Attempting to connect to {self.broker_address}:{self.port}...")
                self.client.connect(self.broker_address, self.port, keepalive=60)
                self.client.loop_start()
                print("[MQTT] Connected. Waiting for on_connect callback...")
                # Do not set self.connected = True here!
                return  # Let on_connect() set it
            except Exception as e:
                print(f"[MQTT] Connection failed: {e}. Retrying in {self.retry_interval} seconds...")
                time.sleep(self.retry_interval)
                self.retry_interval = min(self.retry_interval * 2, 30)  # backoff

    def set_app(self, app):
        self.app = app

    def on_connect(self, client, userdata, flags, rc, *args):
        if rc == 0:
            print("[MQTT] on_connect: Success")
            self.connected = True
            self.retry_interval = 1
            self.client.subscribe("handshake/response", qos=1)
            self.client.subscribe("pi/command", qos=1)
            self.client.subscribe("pi/camera")
            self.client.subscribe("data/updates")
            self.client.subscribe("pi/state")
            self.client.subscribe("jetson/path")
            self.client.subscribe("pi/info")
            self.initiate_handshake()
        else:
            print(f"Failed to connect with result code {rc}")
            #self.connected = False
            #self.connect_to_broker()

    def on_message(self, client, userdata, msg):
        # Delegate to the appropriate handler based on the topic
        print(f"[MQTT] Received message on topic: {msg.topic}")  # Debugging line
        if msg.topic == "pi/command":
            pass
        elif msg.topic == "pi/info":
            if msg.payload.decode() == "ball_found":
                self.ball_found = True
            if msg.payload.decode() == "ball_not_found":
                self.ball_found = False
            if msg.payload.decode() == "timeout":
                print("[Pi] Ball not found for too long, returning to main menu.")
                self.timeout = True
            if msg.payload.decode() == "path_found":
                print("[Pi] Path found by Jetson.")
                self.finding_path = False
        elif msg.topic == "handshake/response":
            if msg.payload.decode() == "ack":
                self.handshake_complete = True
                print("Handshake completed with Jetson")
        elif msg.topic == "pi/camera":
            try:
                image_data = base64.b64decode(msg.payload)
                nparr = np.frombuffer(image_data, np.uint8)
                frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                self.img = frame
            except Exception as e:
                print(f"[ERROR] Failed to decode image: {e}")
        else:
            print(f"Received message on unhandled topic: {msg.topic}")  # Debugging line

    def handshake(self, topic, message):
        self.client.publish(topic, message, 0)

    def initiate_handshake(self):
        print("[Pi] Starting handshake thread...")
        def handshake_loop():
            time.sleep(3)
            while not self.handshake_complete:
                print("Initiating handshake with Jetson")
                self.handshake("handshake/request", "pi")
                time.sleep(5)
        
        # Start the handshake loop in a separate thread
        threading.Thread(target=handshake_loop, daemon=True).start()
    
    def shut_down(self):
        self.client.disconnect()
        self.client.loop_stop()
        #self.thread.join()
        print("Disconnected from broker and stopped the thread")

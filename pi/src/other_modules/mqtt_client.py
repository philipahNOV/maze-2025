import threading
import paho.mqtt.client as mqtt
import time
import base64
import numpy as np
import cv2

class MQTTClientPi(threading.Thread):
    def __init__(self, broker_address='192.168.1.3', port=1883):
        super().__init__()
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2) # type: ignore
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        self.broker_address = broker_address
        self.port = port
        
        self.handshake_complete = False
        self.pi_state = "None"
        self.img = None
        self.jetson_path = "None"
        self.screen2_instance = None
        self.ball_info = None
        
        self.connected = False
        self.retry_interval = 1  # Initial retry interval in seconds
        
        self.connect_to_broker()

    def connect_to_broker(self):
        while not self.connected:
            try:
                self.client.connect(self.broker_address, self.port, 60)
                self.connected = True
                print("Successfully connected to broker")
                self.thread = threading.Thread(target=self.client.loop_forever)
                self.thread.start()
            except Exception as e:
                print(f"Connection failed: {e}. Retrying in {self.retry_interval} seconds...")
                time.sleep(self.retry_interval)
                self.retry_interval = min(self.retry_interval * 2, 60)  # Exponential backoff up to 60 seconds

    def on_connect(self, client, userdata, flags, rc, *args):
        if rc == 0:
            print("Connected with result code " + str(rc))
            self.connected = True
            self.retry_interval = 1  # Reset retry interval after successful connection
            # Subscribe to all necessary topics
            self.client.subscribe("camera/feed")
            self.client.subscribe("data/updates")
            self.client.subscribe("pi/command")
            self.client.subscribe("pi/state")
            self.client.subscribe("handshake/response")
            self.client.subscribe("jetson/path")
        else:
            print(f"Failed to connect with result code {rc}")
            self.connected = False
            self.connect_to_broker()

    def on_message(self, client, userdata, msg):
        # Delegate to the appropriate handler based on the topic
        if msg.topic == "camera/feed":
            image_data = base64.b64decode(msg.payload)
            nparr = np.frombuffer(image_data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            self.img = frame
        elif msg.topic == "data/updates":
            pass
        elif msg.topic == "pi/state":
            self.pi_state = (msg.payload.decode())
        elif msg.topic == "pi/command":
            self.pi_state = (msg.payload.decode())
            self.client.publish("jetson/state", msg.payload.decode(), 0)
            print(f"Jetson state: {self.pi_state}")
        elif msg.topic == "handshake/response":
            if msg.payload.decode() == "ack":
                self.handshake_complete = True
                print("Handshake completed with Jetson")
        elif msg.topic == "jetson/path":
            self.jetson_path = msg.payload.decode()
        elif msg.topic == "ball/info":
            self.ball_info = msg.payload.decode()
        else:
            print(f"Received message on unhandled topic: {msg.topic}")  # Debugging line

    def handshake(self, topic, message):
        self.client.publish(topic, message, 0)

    def initiate_handshake(self):
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
        self.thread.join()
        print("Disconnected from broker and stopped the thread")

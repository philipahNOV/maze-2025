import tkinter as tk
from tkinter import messagebox
import threading
import paho.mqtt.client as mqtt
import time
import base64
import numpy as np
import cv2
import json

class MQTTClientPi(threading.Thread):
    def __init__(self, broker_address='192.168.1.3', port=1883):
        super().__init__()
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(broker_address, port, 60)
        self.broker_address = broker_address
        self.port = port
        self.handshake_complete = False
        self.pi_state = "None"
        self.img = None
        self.jetson_path = "None"
        self.screen2_instance = None
        self.running = True

    def run(self):
        self.client.loop_forever()

    def on_connect(self, client, userdata, flags, rc, *args):
        print("Connected with result code " + str(rc))
        self.client.subscribe("camera/feed")
        self.client.subscribe("data/updates")
        self.client.subscribe("pi/command")
        self.client.subscribe("pi/state")
        self.client.subscribe("handshake/response")
        self.client.subscribe("jetson/path")
        self.handshake("handshake/request", "pi")

    def on_message(self, client, userdata, msg):
        if msg.topic == "camera/feed":
            image_data = base64.b64decode(msg.payload)
            nparr = np.frombuffer(image_data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            self.img = frame
        elif msg.topic == "data/updates":
            pass
        elif msg.topic == "pi/state":
            self.pi_state = (msg.payload.decode())
            time.sleep(1)
            if msg.payload.decode() == "0.0":
                self.client.publish("jetson/command", "1.0", 0)
            elif msg.payload.decode() == "1.0":
                self.client.publish("jetson/command", "1.1", 0)
            elif msg.payload.decode() == "1.2":
                self.client.publish("jetson/command", "1.3", 0)
        elif msg.topic == "pi/command":
            self.pi_state = (msg.payload.decode())
            self.client.publish("jetson/state", msg.payload.decode(), 0)
            if msg.payload.decode() == "1.2":
                self.client.publish("pi/state", msg.payload.decode(), 0)
            print(f"Jetson state: {self.pi_state}")
        elif msg.topic == "handshake/response":
            if msg.payload.decode() == "ack":
                self.handshake_complete = True
                print("Handshake completed with Jetson")
                self.client.publish("jetson/command", "0.0", 0)
        elif msg.topic == "jetson/path":
            self.jetson_path = msg.payload.decode()
        else:
            print(f"Received message on unhandled topic: {msg.topic}")

    def handshake(self, topic, message):
        self.client.publish(topic, message, 0)

    def initiate_handshake(self):
        while not self.handshake_complete:
            print("Initiating handshake with Jetson")
            self.handshake("handshake/request", "pi")
            time.sleep(5)

    def stop(self):
        self.client.disconnect()
        self.running = False

def on_closing(mqtt_client):
    if messagebox.askokcancel("Quit", "Do you want to quit?"):
        mqtt_client.stop()
        root.destroy()

root = tk.Tk()
root.title("MQTT Client")

mqtt_client = MQTTClientPi()
mqtt_client.start()

close_button = tk.Button(root, text="Close", command=lambda: on_closing(mqtt_client))
close_button.pack(pady=20)

root.protocol("WM_DELETE_WINDOW", lambda: on_closing(mqtt_client))
root.mainloop()
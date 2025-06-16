import threading
import cv2
import base64
import paho.mqtt.client as mqtt
from cam_loop import CameraThread  # Ensure CameraThread is correctly imported
from state_variables import send_images
class ImagePublisherThread(threading.Thread):
    def __init__(self, camera_thread, mqtt_broker='192.168.1.3', mqtt_port=1883, topic='camera/feed'):
        super().__init__()
        self.camera_thread = camera_thread
        self.topic = topic
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect(mqtt_broker, mqtt_port, 60)
        self.mqtt_client.loop_start()
        self.running = True

    def run(self):
        while self.running:
            if send_images:
                frame = self.camera_thread.get_latest_frame()
                if frame is not None:
                    success, buffer = cv2.imencode('.jpg', frame)
                    if success:
                        jpg_as_text = base64.b64encode(buffer).decode('utf-8') # type: ignore
                        self.mqtt_client.publish(self.topic, jpg_as_text)
                        print("sent image")

    def stop(self):
        self.running = False
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        print("Image Publisher Thread stopped.")

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
        self.finding_path = False
        self.path_failed = False
        self.connected = False
        self.retry_interval = 1
        
        threading.Thread(target=self.connect_to_broker, daemon=True).start()

    def connect_to_broker(self):
        while not self.connected:
            try:
                print(f"[MQTT] Attempting to connect to {self.broker_address}:{self.port}...")
                self.client.connect(self.broker_address, self.port, keepalive=60)
                self.client.loop_start()
                print("[MQTT] Connected. Waiting for on_connect callback...")
                return
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
            self.client.subscribe("pi/tracking_status")
            self.client.subscribe("pi/leaderboard_data/+")
            self.initiate_handshake()
        else:
            print(f"Failed to connect with result code {rc}")
            #self.connected = False
            #self.connect_to_broker()

    def on_message(self, client, userdata, msg):
        if msg.topic == "pi/command":
            payload = msg.payload.decode()
            if payload.startswith("playalone_success:"):
                duration = payload.split(":")[1]
                rank = payload.split(":")[2]
                if self.app and hasattr(self.app, 'frames'):
                    play_alone_frame = self.app.frames.get('PlayAloneStartScreen')
                    if play_alone_frame:
                        play_alone_frame.show_game_result("success", duration)
                    victory_screen = self.app.frames.get('PlayAloneVictoryScreen')
                    if victory_screen:
                        victory_screen.duration = duration
                        victory_screen.rank = rank
                    self.app.show_frame("PlayAloneVictoryScreen")
            elif payload == "playalone_fail":
                if self.app and hasattr(self.app, 'frames'):
                    play_alone_frame = self.app.frames.get('PlayAloneStartScreen')
                    if play_alone_frame:
                        play_alone_frame.show_game_result("failure", None)
            elif payload == "playalone_start":
                if self.app and hasattr(self.app, 'frames'):
                    play_alone_frame = self.app.frames.get('PlayAloneStartScreen')
                    if play_alone_frame:
                        play_alone_frame.on_start_game_click()
            elif payload == "show_playvsai_screen":
                if self.app:
                    self.app.show_frame("PlayVsAIScreen")
            elif payload == "playvsai_pid_started":
                if self.app and hasattr(self.app, 'frames'):
                    playvsai_frame = self.app.frames.get('PlayVsAIScreen')
                    if playvsai_frame:
                        playvsai_frame.handle_pid_started()
            elif payload.startswith("playvsai_pid_success:"):
                duration = float(payload.split(":")[1])
                if self.app and hasattr(self.app, 'frames'):
                    playvsai_frame = self.app.frames.get('PlayVsAIScreen')
                    if playvsai_frame:
                        playvsai_frame.handle_pid_result(True, duration)
            elif payload.startswith("playvsai_pid_fail"):
                if self.app and hasattr(self.app, 'frames'):
                    playvsai_frame = self.app.frames.get('PlayVsAIScreen')
                    if playvsai_frame:
                        failure_reason = None
                        if ":" in payload:
                            failure_reason = payload.split(":")[1]
                        playvsai_frame.handle_pid_result(False, failure_reason=failure_reason)
            elif payload == "playvsai_human_started":
                if self.app and hasattr(self.app, 'frames'):
                    playvsai_frame = self.app.frames.get('PlayVsAIScreen')
                    if playvsai_frame:
                        playvsai_frame.handle_human_started()
            elif payload.startswith("playvsai_human_success:"):
                duration = float(payload.split(":")[1])
                if self.app and hasattr(self.app, 'frames'):
                    playvsai_frame = self.app.frames.get('PlayVsAIScreen')
                    if playvsai_frame:
                        playvsai_frame.handle_human_result(True, duration)
            elif payload == "playvsai_human_fail":
                if self.app and hasattr(self.app, 'frames'):
                    playvsai_frame = self.app.frames.get('PlayVsAIScreen')
                    if playvsai_frame:
                        playvsai_frame.handle_human_result(False)
            elif payload == "clear_image_buffer":
                print("[MQTT] Clearing image buffer")
                self.img = None
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
            if msg.payload.decode() == "path_not_found":
                print("[Pi] Path not found by Jetson.")
                self.finding_path = False
                self.path_failed = True
        elif msg.topic == "pi/tracking_status":
            if self.app and hasattr(self.app, 'frames'):
                play_alone_frame = self.app.frames.get('PlayAloneStartScreen')
                playvsai_frame = self.app.frames.get('PlayVsAIScreen')
                if play_alone_frame:
                    payload = msg.payload.decode()
                    if payload == "tracking_started":
                        play_alone_frame.update_tracking_status(tracking_ready=True, ball_detected=False)
                        playvsai_frame.update_tracking_status(tracking_ready=True, ball_detected=False)
                    elif payload == "ball_detected":
                        play_alone_frame.update_tracking_status(tracking_ready=True, ball_detected=True)
                        playvsai_frame.update_tracking_status(tracking_ready=True, ball_detected=True)
                    elif payload == "ball_lost":
                        play_alone_frame.update_tracking_status(tracking_ready=True, ball_detected=False)
                        playvsai_frame.update_tracking_status(tracking_ready=True, ball_detected=False)
        elif msg.topic.startswith("pi/leaderboard_data/"):
            try:
                maze_id = int(msg.topic.split("/")[-1])
                csv_data = msg.payload.decode()
                
                if self.app and hasattr(self.app, 'frames'):
                    leaderboard_frame = self.app.frames.get('LeaderboardScreen')
                    if leaderboard_frame:
                        leaderboard_frame.update_leaderboard_data(maze_id, csv_data)
                        print(f"[MQTT] Updated leaderboard data for maze {maze_id}")
            except Exception as e:
                print(f"[MQTT] Failed to handle leaderboard data: {e}")
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
            print(f"Received message on unhandled topic: {msg.topic}")

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
        
        threading.Thread(target=handshake_loop, daemon=True).start()
    
    def shut_down(self):
        self.client.disconnect()
        self.client.loop_stop()
        #self.thread.join()
        print("Disconnected from broker and stopped the thread")
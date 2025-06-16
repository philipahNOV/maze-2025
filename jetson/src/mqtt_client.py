import threading
import paho.mqtt.client as mqtt


class MQTTClientJetson(threading.Thread):
    def __init__(self, broker_address="192.168.1.3", port=1883):
        super().__init__()
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2) # type: ignore
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        try:
            self.client.connect(broker_address, port, 60)
        except Exception as e:
            raise ConnectionError(f"Failed to connect to MQTT broker: {e}")
        
        self.thread = threading.Thread(target=self.client.loop_forever)
        self.thread.start()
        self.jetson_state = "None"
        self.elevator = 0


    def on_connect(self, client, userdata, flags, rc, *args):
        print("Connected with result code " + str(rc))
        self.client.subscribe("handshake/request")
        self.client.subscribe("jetson/command")
        self.client.subscribe("jetson/state")
        self.client.subscribe("arduino/elevator")
        


    def on_message(self, client, userdata, msg):
        """
        Callback function that is called when a message is received.

        Args:
            client: The MQTT client instance that received the message.
            userdata: Any user-defined data that was passed to the MQTT client.
            msg: The received message object.

        Returns:
            None

        Raises:
            None
        """
        print("Message received on topic '{}': {}".format(msg.topic, msg.payload.decode()))
        if msg.topic == "jetson/state":
            self.jetson_state = msg.payload.decode()   
        elif msg.topic == "jetson/command":
            self.jetson_state = msg.payload.decode()
            self.client.publish("pi/state", msg.payload.decode(), 0)
        elif msg.topic == "handshake/request":
            if msg.payload.decode() == "pi":
                print("Received handshake request from Pi")
                self.client.publish("handshake/response", "ack", 0)
                self.client.publish("pi/command", "0.0", 0)
                
                
        elif msg.topic == "pi/response":
            self.pi_state = msg.payload.decode()
        
        elif msg.topic == "arduino/elevator":
            self.elevator = msg.payload.decode()
    
    

    def publish_image(self, image):
        self.client.publish("camera/feed", image, 0)
    
    def publish_ball_info(self, ball_info):
        self.client.publish("ball/info", ball_info, 0)
    
 

    def stop(self):
        self.client.loop_stop()
        self.client.disconnect()

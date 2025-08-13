import paho.mqtt.client as mqtt
import sys
import os
import subprocess
import traceback

BROKER_ADDRESS = "192.168.1.3"
PORT = 1883
RECOVERY_TOPIC = "jetson/recovery"

def on_connect(client, userdata, flags, rc, *args):
    client.subscribe(RECOVERY_TOPIC)

def on_message(client, userdata, msg):
    payload = msg.payload.decode()

    if payload == "recover":        
        try:
            script_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "main.py")

            if not os.path.isfile(script_path):
                return

            log_path = os.path.join(os.path.dirname(__file__), "recovery_log.txt")
            with open(log_path, "w") as log_file:
                result = subprocess.Popen([
                    "gnome-terminal",
                    "--", "bash", "-c",
                    f"{sys.executable} '{script_path}'; exec bash"
                ], stdout=log_file, stderr=subprocess.STDOUT)
        except Exception as e:
            print("Exception during recovery action:")
            traceback.print_exc()

def main():
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.on_connect = on_connect
    client.on_message = on_message

    try:
        client.connect(BROKER_ADDRESS, PORT, 60)
    except Exception as e:
        print(f"Failed to connect to broker: {e}")
        return

    client.loop_forever()

if __name__ == "__main__":
    main()
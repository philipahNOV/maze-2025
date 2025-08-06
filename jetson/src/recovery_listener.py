import paho.mqtt.client as mqtt
import sys
import os
import subprocess

BROKER_ADDRESS = "192.168.1.3"
PORT = 1883
RECOVERY_TOPIC = "jetson/recovery"

def on_connect(client, userdata, flags, rc, *args):
    print("Connected to broker with result code", rc)
    client.subscribe(RECOVERY_TOPIC)

def on_message(client, userdata, msg):
    payload = msg.payload.decode()
    print(f"[RecoveryListener] Received: {payload}")

    if payload == "recover":
        print("[RecoveryListener] Triggering recovery action...")

        script_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "main.py")

        try:
            subprocess.Popen(["lxterminal", "--command", f"python3 {script_path}"])
            print(f"Launched main.py in new lxterminal window.")
        except FileNotFoundError:
            print("lxterminal not found. Trying xterm...")
            try:
                subprocess.Popen(["xterm", "-e", f"python3 {script_path}"])
                print("Launched in xterm.")
            except FileNotFoundError:
                print("No compatible terminal found. Please install lxterminal or xterm.")

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

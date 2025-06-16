import time

def elManuel(mqtt_client, camera_thread, arduino_thread):
    """
    This function represents the main logic for the 'elManuel' state.

    Parameters:
    - mqtt_client: The MQTT client object.
    - camera_thread: The camera thread object.
    - arduino_thread: The Arduino thread object.

    Returns:
    - None
    """
    print("State 2 running")
    arduino_thread.send_target_positions(100, 100, 100, 100)
    time.sleep(1)
    while mqtt_client.jetson_state != "0.0":
        if mqtt_client.elevator == "1":
            mqtt_client.elevator = "0"
            arduino_thread.send_target_positions(300,300,300,300)
        time.sleep(0.05)
    
    time.sleep(1)
    arduino_thread.send_target_positions(200, 200, 200, 200)

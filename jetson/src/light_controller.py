import time

def blinking_red(arduino_thread, stop_event):
    while not stop_event.is_set():
        arduino_thread.send_rgb(255, 0, 0)
        time.sleep(0.5)
        arduino_thread.send_rgb(0, 0, 0)
        time.sleep(0.5)
    arduino_thread.send_rgb(0, 0, 0)
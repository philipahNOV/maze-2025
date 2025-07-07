import time

def blinking_red(arduino_thread):
    arduino_thread.send_color(255, 0, 0)
    time.sleep(2)
    arduino_thread.send_color(0, 0, 0)
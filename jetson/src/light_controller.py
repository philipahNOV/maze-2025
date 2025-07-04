import time

def blinking_red(arduino_thread, ball_pos):
    while ball_pos is None:
        arduino_thread.send_rgb(255, 0, 0)
        time.sleep(0.5)
        arduino_thread.send_rgb(0, 0, 0)
        time.sleep(0.5) 
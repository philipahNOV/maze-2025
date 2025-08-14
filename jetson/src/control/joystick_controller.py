import pygame
import time
from functools import wraps
from arduino_connection import ArduinoConnection
from mqtt.mqtt_client import MQTTClientJetson


class JoystickController:
    def __init__(self, arduino: ArduinoConnection, mqtt_client: MQTTClientJetson, playalone_wait):
        self.arduino = arduino
        self.mqtt_client = mqtt_client
        self.running = False
        self.playalone_wait = playalone_wait
        self.deadzone = 5000
        self.max_raw = 32767
        self.update_rate_hz = 120
        self.elevator_state = 1
        self.prev_button_state = 0
        self.ball_in_elevator = False
        self.r2_scaled = 0.0

    def _apply_deadzone(func):
        @wraps(func)
        def wrapper(self, value):
            raw = int(value * self.max_raw)
            if abs(raw) <= self.deadzone:
                return 0
            return func(self, raw)
        return wrapper

    @_apply_deadzone
    def scaled_output(self, raw):
        scaled = (abs(raw) - self.deadzone) / (self.max_raw - self.deadzone) * (255 - 22) + 22
        scaled = int(scaled * abs(self.r2_scaled - 1))
        return int(scaled) if raw > 0 else -int(scaled)

    def start(self):
        pygame.init()
        pygame.joystick.init()

        try:
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
        except pygame.error:
            return

        self.running = True
        interval = 1.0 / self.update_rate_hz

        try:
            while self.running:
                loop_start = time.time()
                pygame.event.pump()

                if self.playalone_wait:
                    button = joystick.get_button(0)
                    if button and self.prev_button_state != button and self.ball_in_elevator:
                        self.prev_button_state = button
                        self.mqtt_client.client.publish("pi/command", "playalone_start")
                        self.playalone_wait = False
                    time.sleep(max(0, interval - (time.time() - loop_start)))
                    continue

                axis_x = -joystick.get_axis(1)  # venstre horisontal
                axis_y = -joystick.get_axis(0)  # venstre vertikal
                vel_x = self.scaled_output(axis_x)
                vel_y = self.scaled_output(axis_y)
                self.arduino.send_speed(vel_x, vel_y)
                button = joystick.get_button(0)

                if button and not self.prev_button_state:
                    self.elevator_state *= -1
                    self.arduino.send_elevator(self.elevator_state)

                r2_value = joystick.get_axis(5)  # 5 is common for RT, but may vary
                self.r2_scaled = ((r2_value + 1) / 2) * (1.0 - 0.087) + 0.087

                self.prev_button_state = button
                time.sleep(max(0, interval - (time.time() - loop_start)))

        except Exception as e:
            print(f"[XboxController] Exception: {e}")

        finally:
            self.arduino.send_speed(0, 0)
            pygame.quit()

    def stop(self):
        self.running = False
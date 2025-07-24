import pygame
import time
from functools import wraps
from arduino_connection import ArduinoConnection


class JoystickController:
    def __init__(self, arduino: ArduinoConnection):
        self.arduino = arduino
        self.running = False
        self.deadzone = 5000
        self.max_raw = 32767
        self.update_rate_hz = 120

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
        scaled = (abs(raw) - self.deadzone) / (self.max_raw - self.deadzone) * 254 + 1
        return int(scaled) if raw > 0 else -int(scaled)

    def start(self):
        pygame.init()
        pygame.joystick.init()

        try:
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
        except pygame.error:
            print("[XboxController] No joystick found.")
            return

        self.running = True
        interval = 1.0 / self.update_rate_hz
        print("[XboxController] Started polling at {:.0f} Hz".format(self.update_rate_hz))

        try:
            while self.running:
                loop_start = time.time()

                pygame.event.pump()
                axis_x = -joystick.get_axis(1)  # venstre horisontal
                axis_y = -joystick.get_axis(0)  # vensre vertikal

                vel_x = self.scaled_output(axis_x)
                vel_y = self.scaled_output(axis_y)

                self.arduino.send_speed(vel_x, vel_y)

                elevator_down = joystick.get_button(0)
                if elevator_down:
                    self.arduino.send_elevator(-1)

                elevator_up = joystick.get_button(3)
                if elevator_up:
                    self.arduino.send_elevator(1)

                time.sleep(max(0, interval - (time.time() - loop_start)))

        except Exception as e:
            print(f"[XboxController] Exception: {e}")

        finally:
            self.arduino.send_speed(0, 0)
            pygame.quit()
            print("[XboxController] Clean exit, resources released.")

    def stop(self):
        self.running = False
        print("[XboxController] Stop requested.")
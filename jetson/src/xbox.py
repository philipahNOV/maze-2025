import pygame
import time
from arduino_connection import ArduinoConnection

class XboxController:
    def __init__(self, arduino: ArduinoConnection):
        self.arduino = arduino
        self.running = False

    def scaled_output(self, value):
        """
        Skaler joystickverdi med dødsone til området -255 til 255.
        """
        DEADZONE = 5000
        MAX_RAW = 32767

        raw = int(value * MAX_RAW)

        if abs(raw) <= DEADZONE:
            return 0
        else:
            scaled = (abs(raw) - DEADZONE) / (MAX_RAW - DEADZONE) * 254 + 1
            return int(scaled) if raw > 0 else -int(scaled)

    def start(self):
        """
        Starter kontrolleren og leser joystickverdier i 60 Hz.
        """
        pygame.init()
        pygame.joystick.init()
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        self.running = True
        print("Xbox Controller started.")

        target_interval = 1.0 / 120  # 60 Hz kontrollsløyfe

        try:
            while self.running:
                loop_start = time.time()

                pygame.event.pump()
                x_axis = -joystick.get_axis(1)  # venstre horisontal
                y_axis = -joystick.get_axis(0)  # venstre vertikal

                vel_x = self.scaled_output(x_axis)
                vel_y = self.scaled_output(y_axis)

                self.arduino.send_speed(vel_x, vel_y)

                elapsed = time.time() - loop_start
                sleep_time = max(0, target_interval - elapsed)
                time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("Xbox Controller stopped.")

        finally:
            self.arduino.send_speed(0, 0)
            pygame.quit()
            print("Xbox Controller resources released.")
    
    def stop(self):
        """
        Stopper kontrolleren.
        """
        self.running = False
        print("Stopping Xbox Controller...")

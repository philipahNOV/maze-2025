import pygame
import time
from arduino_connection import ArduinoConnection

class XboxController:
    def __init__(self, arduino: ArduinoConnection, deadzone=0.1, scale=100):
        self.arduino = arduino
        self.deadzone = deadzone
        self.scale = scale
        self.running = False

    def start(self):
        """
        Starts the Xbox controller thread.
        Initializes pygame and starts the controller event loop.
        """
        pygame.init()
        pygame.joystick.init()
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        self.running = True
        print("Xbox Controller started.")

        try:
            while self.running:
                pygame.event.pump()
                x_axis = joystick.get_axis(0)
                y_axis = joystick.get_axis(1)

                if abs(x_axis) < self.deadzone:
                    x_axis = 0
                if abs(y_axis) < self.deadzone:
                    y_axis = 0
                
                vel_x = int(x_axis * self.scale)
                vel_y = int(y_axis * self.scale)

                self.arduino.send_speed(vel_x, vel_y)

                time.sleep(0.02)
                
        except KeyboardInterrupt:
            print("Xbox Controller stopped.")

        finally:
            self.arduino.send_speed(0, 0)
            pygame.quit()
            print("Xbox Controller resources released.")
    
    def stop(self):
        """
        Stops the Xbox controller thread.
        Sets the running flag to False to exit the event loop.
        """
        self.running = False
        print("Stopping Xbox Controller...")
import math
import matplotlib.pyplot as plt
import numpy as np

def simulate_ball_rolling(angle, current_velocity, current_position=0.0, _=0.0, time_step=0.05, g=9.81):
    current_position = 0.0
    angle_rad = np.radians(angle)  # Convert angle to radians
    a = g * np.sin(angle_rad)  # Acceleration along the plane
    a_mm = a * 1000
    new_velocity = current_velocity + a_mm * time_step
    new_position = current_position + current_velocity * time_step + 0.5 * a_mm * time_step**2

    return new_velocity, new_position
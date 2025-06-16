import numpy as np

def calculate_reward(distance_to_goal, velocity):
    if distance_to_goal == 0.0 and velocity == 0.0:
        return 100
    elif distance_to_goal < 0.0:
        if velocity <= 0:
            return 5
        else:
            return -5
    elif distance_to_goal > 0.0:
        if velocity >= 0:
            return 5
        else:
            return -5
    else:
        return 0

test_cases = [
    (10.0, 5.0, 5),
    (-10.0, -5.0, 5),
    (10.0, -5.0, -5),
    (-10.0, 5.0, -5),
    (0.0, 0.0, 100),
    (5.0, 2.0, 5),
    (-5.0, -2.0, 5),
    (5.0, -2.0, -5),
    (-5.0, 2.0, -5),
    (0.0, 5.0, 0),
    (0.0, -5.0, 0),
    (10.0, 0.0, 0),
    (-10.0, 0.0, 0),
]

def run_tests():
    for i, (distance_to_goal, velocity, expected_reward) in enumerate(test_cases):
        result = calculate_reward(distance_to_goal, velocity)
        if result != expected_reward:
            print(f"Test case {i+1} failed: Expected {expected_reward}, got {result}")
        else:
            print(f"Test case {i+1} passed.")

run_tests()
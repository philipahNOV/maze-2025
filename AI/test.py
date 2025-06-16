from simulation import simulate_ball_rolling
state = [1, 0.3, 5]
for i in range(1, 5):
    angle = state[-3]
    velocity = state[-2]
    distance_to_goal = state[-1]
    new_vel , new_pos_change = simulate_ball_rolling(angle, velocity)
    state.append(angle)
    state.append(new_vel)
    distance_to_goal += new_pos_change
    state.append(distance_to_goal)


print(state)
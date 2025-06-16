from simulation import simulate_ball_rolling
import matplotlib.pyplot as plt
def able_to_stop(angle, velocity, distance):
    pos_vec = []
    angle_vec = []
    pos = 0
    time_over = 0
    if distance > 0:
        sing = -1
    else:
        sing = 1
    while True:
        time_over += 1
        angle += sing * 0.2
        angle = max(-2, min(angle, 2))
        angle_vec.append(angle)
        velocity, pos_change = simulate_ball_rolling(angle, velocity)
        pos += pos_change
        pos_vec.append(pos)
        if sing == -1:
            if velocity <= 0.0 and pos <= distance:
                return True, pos, pos_vec, angle_vec
            elif velocity <= 0.0 and pos > distance:
                return False, pos, pos_vec, angle_vec
        else:
            if velocity >= 0.0 and pos >= distance:
                return True, pos, pos_vec,angle_vec
            elif velocity >= 0.0 and pos < distance:
                return False, pos, pos_vec, angle_vec
            

"""
dist = [0.5, -0.5]
vel = [0.1, -0.1]
ang = [1, -1]
"""
dist = [-0.03351*1000]# -0.04352, 
vel = [-0.08132*1000]# 0.002563
ang = [-1.8]# -2, 
for d in dist:
    for v in vel:
        for a in ang:
            able, pos, pos_graph, angle_vec = able_to_stop(a, v, d)
            print(f'{able}, {pos}, {a}, {v}, {d}')
            #print(pos_graph)

            plt.plot(pos_graph)
            plt.axhline(y=d, color='r', linestyle='--')
            plt.xlabel('Time')
            plt.ylabel('Position')
            plt.title('Position vs Time')
            plt.show()
            plt.plot(angle_vec)
            plt.xlabel('Time')
            plt.ylabel('Position')
            plt.title('Position vs Time')
            plt.show()



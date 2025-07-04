import testing.yolov1.hsv3 as tracking
import time
import cv2
import positionController_2
import lowPassFilter
import path_following
from mqtt_client import MQTTClientJetson
from astar.astar import astar, astar_downscaled
from astar.brightness import get_dynamic_threshold, create_binary_mask, dilate_mask
import math
import numpy as np
from logger import OfflineLogger

clicked_goal = None

def on_mouse_click(event, x, y, flags, param):
    global clicked_goal
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_goal = (y, x)
        print(f"[INFO] Clicked goal set to: {clicked_goal}")

def snap_to_nearest_walkable(mask, point, max_radius=10):
    y, x = point
    if mask[y, x] != 0:
        return point

    dist = cv2.distanceTransform(mask.astype(np.uint8), cv2.DIST_L2, 5)
    min_val = float('inf')
    nearest = point

    h, w = mask.shape
    for dy in range(-max_radius, max_radius + 1):
        for dx in range(-max_radius, max_radius + 1):
            ny, nx = y + dy, x + dx
            if 0 <= ny < h and 0 <= nx < w and mask[ny, nx] > 0:
                if dist[ny, nx] < min_val:
                    min_val = dist[ny, nx]
                    nearest = (ny, nx)
    return nearest

def angle_between(p1, p2, p3):
    a = np.array(p1)
    b = np.array(p2)
    c = np.array(p3)
    ba = a - b
    bc = c - b
    cos_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
    return np.degrees(np.arccos(np.clip(cos_angle, -1.0, 1.0)))

def is_clear_path(mask, p1, p2, kernel_size=6):
    line_img = np.zeros_like(mask, dtype=np.uint8)
    cv2.line(line_img, (p1[1], p1[0]), (p2[1], p2[0]), 1, 1)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_size, kernel_size))
    corridor = cv2.dilate(line_img, kernel)
    return np.all(mask[corridor.astype(bool)] > 0)


def sample_waypoints(path, mask, target_count=20, angle_threshold=135):
    if not path or len(path) < 2:
        return path or []

    total_length = sum(
        math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        for p1, p2 in zip(path[:-1], path[1:])
    )
    spacing = total_length / target_count

    waypoints = [path[0]]
    last_wp_idx = 0
    accumulated = 0.0

    for i in range(1, len(path)):
        p1 = path[i - 1]
        p2 = path[i]
        step_dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        accumulated += step_dist

        if i < len(path) - 1:
            angle = angle_between(path[last_wp_idx], path[i], path[i + 1])
            if angle < angle_threshold and accumulated >= spacing / 2:
                if is_clear_path(mask, path[last_wp_idx], path[i]):
                    waypoints.append(path[i])
                    last_wp_idx = i
                    accumulated = 0.0
                    continue

        if accumulated >= spacing:
            best_idx = last_wp_idx + 1
            for j in range(i, last_wp_idx, -1):
                if is_clear_path(mask, path[last_wp_idx], path[j]):
                    best_idx = j
            waypoints.append(path[best_idx])
            last_wp_idx = best_idx
            accumulated = 0.0

    if waypoints[-1] != path[-1]:
        waypoints.append(path[-1])

    return waypoints

def draw_path(image, path, waypoints, start, goal):
    out = image.copy()
    if len(out.shape) == 2 or out.shape[2] == 1:
        out = cv2.cvtColor(out, cv2.COLOR_GRAY2BGR)
    elif out.shape[2] == 4:
        out = cv2.cvtColor(out, cv2.COLOR_BGRA2BGR)

    h, w = out.shape[:2]

    # for y, x in path or []:
    #     if 0 <= y < h and 0 <= x < w:
    #         out[y, x] = (0, 0, 255)

    # for y, x in waypoints or []:
    #     if 0 <= y < h and 0 <= x < w:
    #         cv2.circle(out, (x, y), 2, (0, 255, 255), -1)

    if waypoints and len(waypoints) > 1:
        for i in range(1, len(waypoints)):
            pt1 = (waypoints[i - 1][1], waypoints[i - 1][0])  # (x, y)
            pt2 = (waypoints[i][1], waypoints[i][0])
            cv2.line(out, pt1, pt2, (255, 255, 255), 1, lineType=cv2.LINE_AA)  # white line

    for y, x in waypoints or []:
        if 0 <= y < h and 0 <= x < w:
            cv2.circle(out, (x, y), 2, (0, 255, 255), -1)  # yellow

    if start:
        cv2.circle(out, (start[1], start[0]), 5, (0, 255, 0), -1)
    if goal:
        cv2.circle(out, (goal[1], goal[0]), 5, (255, 0, 0), -1)

    return out

def main(tracker: tracking.BallTracker, controller: positionController_2.Controller, mqtt_client: MQTTClientJetson):
    smoother = lowPassFilter.SmoothedTracker(alpha=0.5)
    logger = OfflineLogger()  # <-- NEW
    prev_state = None
    prev_action = None
    prev_wp = None

    # === Initialization ===
    while not tracker.initialized:
        time.sleep(0.1)

    while tracker.frame is None:
        time.sleep(0.1)

    maze_frame = tracker.get_stable_frame()
    while maze_frame is None:
        time.sleep(0.1)
        maze_frame = tracker.get_stable_frame()

    gray = get_dynamic_threshold(maze_frame)
    binary_mask = create_binary_mask(gray)
    safe_mask = dilate_mask(binary_mask)

    start = (604, 950)
    global clicked_goal
    clicked_goal = None

    cv2.namedWindow("Safe Mask")
    cv2.setMouseCallback("Safe Mask", on_mouse_click)

    preview_mask = safe_mask.copy()
    cv2.circle(preview_mask, (start[1], start[0]), 10, 127, -1)

    while clicked_goal is None:
        cv2.imshow("Safe Mask", preview_mask)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            return

    cv2.destroyWindow("Safe Mask")
    goal = clicked_goal

    path = astar_downscaled(safe_mask, start, goal, repulsion_weight=5.0, scale=0.55)
    waypoints = sample_waypoints(path, safe_mask)
    path_array = [(x, y) for y, x in waypoints]
    pathFollower = path_following.PathFollower(path_array, controller)

    try:
        while True:
            frame = tracker.frame
            if frame is None:
                continue

            ball_pos = tracker.get_position()
            ball_pos = smoother.update(ball_pos)
            if ball_pos is None:
                continue

            orientation = tracker.get_orientation()
            if orientation is None:
                continue

            # === STATE ===
            x, y = ball_pos
            alpha, beta = orientation
            state = {"x": x, "y": y, "alpha": alpha, "beta": beta}

            # === ACTION ===
            action = {
                "vel_alpha": controller.prev_vel_x,
                "vel_beta": controller.prev_vel_y
            }

            # === REWARD ===
            reward = -0.1
            current_wp = pathFollower.next_waypoint

            if prev_wp is not None:
                if current_wp > prev_wp:
                    reward += 1.0
                elif current_wp < prev_wp:
                    reward -= 1.0

            goal_reached = (current_wp == pathFollower.length - 1) and \
                np.linalg.norm(np.array(ball_pos) - np.array(pathFollower.path[-1])) < controller.pos_tol

            if goal_reached:
                reward += 20.0
                done = True
            elif controller.stuck:
                reward -= 5.0
                done = True
            else:
                done = False

            # === LOG ===
            if prev_state is not None and prev_action is not None:
                logger.log_step(prev_state, prev_action, reward, state, done)

            prev_state = state
            prev_action = action
            prev_wp = current_wp

            # === Render & Control ===
            frame = draw_path(frame, path, waypoints, start, goal)
            pathFollower.follow_path(ball_pos)

            cv2.circle(frame, ball_pos, 8, (0, 255, 0), -1)
            cv2.imshow("Ball & Marker Tracking", frame)

            if done:
                print(f"[Episode {logger.episode_counter}] Done. Waiting for 20s to reset ball...")
                time.sleep(20)
                return

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            if mqtt_client.stop_control:
                mqtt_client.stop_control = False
                controller.arduinoThread.send_target_positions(0, 0)
                break

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        tracker.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
Traceback (most recent call last):
  File "/usr/lib/python3.10/threading.py", line 1016, in _bootstrap_inner
    self.run()
  File "/home/student/Documents/maze-2025/jetson/src/uitility_threads.py", line 121, in run
    waypoints = astar.waypoint_sampling_2.sample_waypoints(path, safe_mask)
  File "/home/student/Documents/maze-2025/jetson/src/astar/waypoint_sampling_2.py", line 49, in sample_waypoints
    print(f"REJECTED: too_close @ {k} (dist_map={dist_map[int(b[1]), int(b[0])]})")
IndexError: index 1017 is out of bounds for axis 0 with size 720




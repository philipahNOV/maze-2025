Exception in thread Thread-9 (main):
Traceback (most recent call last):
  File "/usr/lib/python3.10/threading.py", line 1016, in _bootstrap_inner
    self.run()
  File "/usr/lib/python3.10/threading.py", line 953, in run
    self._target(*self._args, **self._kwargs)
  File "/home/student/Documents/maze-2025/jetson/src/run_controller_main.py", line 39, in main
    pathFollower = path_following.PathFollower(path_array, controller, config)
  File "/home/student/Documents/maze-2025/jetson/src/path_following.py", line 49, in __init__
    self.vel_threshold = self.config["path_following"]["normal"].get("velocity_threshold", 50)  # pixels per second
TypeError: 'NoneType' object is not subscriptable


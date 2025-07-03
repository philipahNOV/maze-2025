Message received on topic 'jetson/command': Stop_control
[2025-07-03 10:08:20 UTC][ZED][ERROR] [ZED] sl::Camera::Open has not been called, no Camera instance running.
[INFO] Tracker stopped.
Exception in thread Thread-4 (producer_loop):
Traceback (most recent call last):
  File "/usr/lib/python3.10/threading.py", line 1016, in _bootstrap_inner
    self.run()
  File "/usr/lib/python3.10/threading.py", line 953, in run
    self._target(*self._args, **self._kwargs)
  File "/home/student/Documents/maze-2025/jetson/src/testing/yolov1/hsv3.py", line 69, in producer_loop
    frames = self.grab_frame()
  File "/home/student/Documents/maze-2025/jetson/src/testing/yolov1/hsv3.py", line 63, in grab_frame
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
cv2.error: OpenCV(4.11.0) /io/opencv/modules/imgproc/src/color.cpp:199: error: (-215:Assertion failed) !_src.empty() in function 'cvtColor'


> Overload resolution failed:
>  - Can't parse 'center'. Sequence item with index 0 has a wrong type
>  - Can't parse 'center'. Sequence item with index 0 has a wrong type

Traceback (most recent call last):
  File "/home/student/Documents/maze-2025/jetson/src/run_controller.py", line 107, in main
    cropped_frame = image_controller.update(ball_pos, pathFollowerLookahead, mqtt_client)
  File "/home/student/Documents/maze-2025/jetson/src/image_controller.py", line 92, in update
    self.draw_waypoints_lookahead(pathFollower)
  File "/home/student/Documents/maze-2025/jetson/src/image_controller.py", line 42, in draw_waypoints_lookahead
    cv2.circle(self.frame, pathFollower.lookahead_point, 5, (100, 200, 100), -1)
cv2.error: OpenCV(4.11.0) :-1: error: (-5:Bad argument) in function 'circle'
> Overload resolution failed:
>  - Can't parse 'center'. Sequence item with index 0 has a wrong type
>  - Can't parse 'center'. Sequence item with index 0 has a wrong type


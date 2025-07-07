from camera.camera_manager import CameraManager
from camera.ball_tracker import BallTracker

class TrackerService:
    def __init__(self, model_path="v8-291.pt"):
        self.camera = CameraManager()
        self.camera.init_camera()
        self.tracker = None
        self.started = False
        self.model_path = model_path

    def start_tracker(self):
        self.tracker = BallTracker(self.camera, model_path=self.model_path)
        self.tracker.start()
        self.started = True

    def stop_tracker(self):
        if self.started:
            self.tracker.stop()
            self.started = False
            self.camera.close()

    def get_ball_position(self):
        return self.tracker.get_position() if self.started else None

    def get_stable_frame(self):
        return self.tracker.get_frame() if self.started else None

    def get_orientation(self):
        return self.camera.get_orientation() if self.started else None

    @property
    def is_initialized(self):
        return self.tracker.initialized if self.tracker else False
    
    def retrack(self):
        if self.started and self.tracker:
            self.tracker.retrack() # call this using tracker_service.retrack()
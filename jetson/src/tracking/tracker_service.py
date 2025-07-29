from tracking.camera_manager import CameraManager
from tracking.ball_tracker import BallTracker

class TrackerService:
    def __init__(self, model_path="v8-291.pt", tracking_config=None, full_config=None):
        self.camera = CameraManager()
        self.camera.init_camera()
        self.tracker = None
        self.started = False
        self.model_path = model_path
        self.tracking_config = tracking_config or {}
        # Add full config reference for accessing camera parameters
        if full_config:
            self.tracking_config["full_config"] = full_config

    def start_tracker(self):
        self.tracker = BallTracker(self.camera, self.tracking_config, model_path=self.model_path)
        self.tracker.start()
        self.started = True

    def stop_tracker(self):
        if self.started:
            self.tracker.stop()
            self.started = False

    def get_ball_position(self):
        return self.tracker.get_position() if self.started else None

    def get_stable_frame(self):
        return self.tracker.get_frame() if self.started else None

    def get_orientation(self):
        return self.camera.get_orientation()

    @property
    def is_initialized(self) -> bool:
        return self.tracker.initialized if self.tracker else False
    
    def retrack(self) -> None:
        if self.started and self.tracker:
            self.tracker.retrack() # call this using tracker_service.retrack()
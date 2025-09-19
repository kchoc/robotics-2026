from sr.robot3.camera import AprilCamera as Camera

class Arena:
    _cameras: list[Camera] = []

    @property
    def cameras(self) -> list[Camera]:
        return self._cameras

    def __init__(self, cameras: list[Camera] = []):
        self._cameras = cameras
    
    
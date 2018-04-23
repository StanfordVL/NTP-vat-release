import numpy as np

from .body import Entity


class Camera(Entity):
    """Camera setting."""

    def __init__(self, pe, pos, euler, frame, focal_dist=0.5, name=None):
        self._pe = pe
        self._pos = pos
        self._euler = euler
        self._focal_dist = focal_dist
        self._frame = frame
        self._name = name

    @property
    def pos(self):
        if self._frame is None:
            frame_pos = np.zeros((3,), dtype=np.float32)
            frame_euler = np.zeros((3,), dtype=np.float32)
        else:
            frame_pos = self._frame.pos
            frame_euler = self._frame.euler
        frame = (frame_pos, frame_euler)
        pos = self._pe.pos_in_frame(self._pos, frame)
        return pos

    @property
    def euler(self):
        if self._frame is None:
            frame_euler = np.zeros((3,), dtype=np.float32)
        else:
            frame_euler = self._frame.euler
        frame = (None, frame_euler)
        euler = self._pe.euler_in_frame(self._euler, frame)
        return euler

    @property
    def focal_dist(self):
        return self._focal_dist

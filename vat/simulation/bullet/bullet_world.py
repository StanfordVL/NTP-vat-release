import os
import numpy as np
import pybullet as p

from ..world import World
from bullet_physics_engine import BulletPhysicsEngine


class BulletWorld(World):
    """World with Bullet."""

    def __init__(self,
                 display=True,
                 data_dir='./data',
                 verbose=False,
                 key=None,
                 camera_params={}):

        self._pe = BulletPhysicsEngine()

        self._display = display
        self._data_dir = data_dir
        self._verbose = verbose

        self._bodies = None
        self._robots = None
        self._time_step = None
        self._ctrl_listeners = []

        self._key_dict = None
        self._key_act_dict = None
        self._modifier_dict = None

        # Camera Parameters
        fov = camera_params.get('fov', 60)
        aspect = camera_params.get('aspect', 1)
        near = camera_params.get('near', 0.02)
        far = camera_params.get('far', 1)
        view_matrix = camera_params.get(
            'view_matrix',
            [[0.0, -0.4, 1.4], [0, 0.0, 0], [1, 0, 0]]
        )


        self.view_matrix = p.computeViewMatrix(*view_matrix)
        self.projection_matrix = p.computeProjectionMatrixFOV(
            fov, aspect, near, far)

        self.video_log_key = 0

        # Connect to the simulation
        # TODO(Kuan): If VR
        #     p.connect(p.SHARED_MEMORY)
        if self._display:
            p.connect(p.GUI)
        else:
            if key is None:
                p.connect(p.DIRECT)
            else:
                p.connect(p.DIRECT, key=key)

    def start(self, time_step=None):
        """Start the simulation."""

        if time_step:
            self._time_step = time_step
        # Choose real time or step simulation
        if self._time_step is None:
            p.setRealTimeSimulation(1)
        else:
            p.setRealTimeSimulation(0)
            p.setTimeStep(self._time_step)

    def log_video(self, task_name):
        """
        Logs video of each task being executed
        """
        if not os.path.exists("video_logs/"):
            os.makedirs("video_logs")
        try:
            p.stopStateLogging(self.curr_recording)
            self.video_log_key += 1
        except Exception:
            print("No Video Currently Being Logged")
        self.curr_recording = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4,
                                                  "video_logs/task_vid_" +
                                                  str(task_name) + "_" + str(self.video_log_key) + ".mp4")

    def capture_image(self):
        width, height, im, depth, seg = p.getCameraImage(64, 64, list(
            self.view_matrix), list(self.projection_matrix), renderer=p.ER_TINY_RENDERER)
        self.depth = depth
        im = np.array(im).reshape([height, width, -1])
        return im[:, :, :3]

    def restart(self):
        """Restart the simulation"""
        # Reset
        p.resetSimulation()
        self.load()
        self.start()

    def step(self):
        """Take a simulation step."""
        # Handle controller listener events
        for cl in self._ctrl_listeners:
            if cl is not None:
                cl.listen()
        # Simulate a step
        if self._time_step is None:
            pass
        else:
            p.stepSimulation()
        # Update camera
        self.set_camera(self._camera)
        return

    def close(self):
        """Terminate the simulation"""
        p.disconnect()

    def _key_events_fetcher(self):
        """Fetch and decode keyboard events in Bullet GUI.

        Returns:
            events: list of (key, key_act, modifiers) tuples.
        """
        # Fetch events
        bullet_events = p.getKeyboardEvents()
        # Map event id to key name
        if self._key_dict is None:
            key_dict = {}
            for i in xrange(128):
                key_dict[i] = str(unichr(i))
            key_dict[65295] = 'LEFT'
            key_dict[65296] = 'RIGHT'
            key_dict[65297] = 'UP'
            key_dict[65298] = 'DOWN'
            key_dict[65309] = 'ENTER'
            self._key_dict = key_dict
        # Map event id to key activity name
        if self._key_act_dict is None:
            key_act_dict = {}
            key_act_dict[1] = 'DOWN'
            key_act_dict[2] = 'TRIGGERED'
            key_act_dict[3] = 'UNKNOWN(3L)'
            key_act_dict[4] = 'RELEASED'
            key_act_dict[5] = 'UNKNOWN(5L)'
            key_act_dict[6] = 'UNKNOWN(6L)'
            self._key_act_dict = key_act_dict
        # Map event id to modifier key names
        if self._modifier_dict is None:
            modifier_dict = {}
            modifier_dict[65306] = 'SHIFT'
            modifier_dict[65307] = 'CTRL'
            modifier_dict[65308] = 'OPTION'
            self._modifier_dict = modifier_dict
        # Decode events
        events = []
        for key, key_act in bullet_events.items():
            if self._key_dict.has_key(key):
                key_name = self._key_dict[key]
            elif self._modifier_dict.has_key(key):
                key_name = self._modifier_dict[key]
            else:
                key_name = None
            key_act_name = self._key_act_dict[key_act]
            events.append((key_name, key_act_name))
        return events

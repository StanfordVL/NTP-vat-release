import os.path as osp

import pybullet as p

import vat.controller as controller
from .io import parse_world_from_file
from .physics_engine import PhysicsEngine
from .body import Body
from .robot import get_robot
from .camera import Camera


class World(object):
    """World base class."""

    def __init__(self,
            display=True,
            data_dir='./data',
            verbose=False):
        """Initialize the simulator.

        Args:
        """
        self._pe = PhysicsEngine()

        self._display = display
        self._data_dir = data_dir
        self._verbose = verbose

        self._bodies = None
        self._robots = None
        self._time_step = None
        self._ctrl_listeners = []

    def load(self, path=None):
        """Build the world"""
        if path:
            self.w = parse_world_from_file(path)['world']
        self._bodies = {}
        self._robots = {}
        # Build
        for body_descr in self.w['body']:
            # Build uncontrollable bodies
            body = Body.create_from_descr(self.pe, body_descr, self._data_dir)
            self._bodies[body.name] = body
        for robot_descr in self.w['robot']:
            # Build uncontrollable bodies
            robot = get_robot(self.pe, robot_descr, self._data_dir)
            self._robots[robot.name] = robot
            for body_name, body in robot.bodies.items():
                self._bodies[body_name] = body
        # Set gravity
        self.pe.set_gravity(self.w['gravity'])
        # Set camera
        self._camera = self.create_camera(self.w['gui']['camera'])

    def add_body(self, body_descr):
        body = Body.create_from_descr(self.pe, body_descr, self._data_dir)
        self._bodies[body.name] = body

    def start(self):
        """Start the simulation."""
        raise NotImplementedError

    def restart(self):
        """Restart the simulation"""
        raise NotImplementedError

    def step(self):
        """Take a simulation step."""
        raise NotImplementedError

    def close(self):
        """Terminate the simulation"""
        raise NotImplementedError

    def create_camera(self, descr):
        """Create the camera."""
        pos = descr['pose']['xyz']
        euler = descr['pose']['rpy']
        frame = None
        if descr['frame'] is not None:
            for body_name, body in self.bodies.items():
                if body_name == descr['frame']:
                    frame = body
                    break
        name = descr['name']
        return Camera(self.pe, pos, euler, frame, name=name)

    def set_camera_val(self, pos, focal_dist, euler):
        self.pe.set_camera(pos, focal_dist, euler)

    def set_camera(self, camera):
        """Set the camera."""
        self.pe.set_camera(camera.pos, camera.focal_dist, camera.euler)

    def add_ctrl_listener(self, ctrl_listener, robot=None):
        """Set up controller listeners."""
        # Set the events fetcher
        if type(ctrl_listener) == type(None):
            pass
        elif type(ctrl_listener) == controller.KeyListener:
            ctrl_listener.events_fetcher = self._key_events_fetcher
        else:
            raise ValueError('Unrecognized controller listener type {}'\
                    .format(type(ctrl_listener)))
        # Find the robot
        if robot is None:
            ind = len(self._ctrl_listeners)
            robot = self.robots.values()[ind]
        elif type(robot) == str:
            robot = self.robots[robot]
        else:
            robot = robot
        # Set the controller listener for the robot
        robot.setup_ctrl_handlers(ctrl_listener)
        self._ctrl_listeners.append(ctrl_listener)

    def _key_events_fetcher(self):
        """Fetch and decode keyboard events.

        Returns:
            events: list of (key, key_act, modifiers) tuples.
        """
        raise NotImplementedError

    @property
    def pe(self):
        return self._pe

    @property
    def bodies(self):
        return self._bodies

    @property
    def robots(self):
        return self._robots

    @property
    def data_dir(self):
        return self._data_dir

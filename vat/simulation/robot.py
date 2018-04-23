import os.path as osp
import numpy as np

from .body import Entity
from .body import Body
import vat.controller as controller


def get_robot(pe, descr, data_dir):
    """ """
    if descr['type'] == 'robot':
        return Robot.create_from_descr(pe, descr, data_dir)
    elif descr['type'] == 'gripper':
        return Gripper.create_from_descr(pe, descr, data_dir)
    else:
        raise ValueError


class Constraint(Entity):
    """Joint constraint."""
    def __init__(self, pe, uid, name=None):
        # Physics engine API wrapper
        self._pe = pe
        # Link unique ID
        self._uid = uid
        # Link name
        self._name = name

    @ classmethod
    def create_from_descr(cls, pe, descr):
        """Create an instance from description."""
        uid = pe.create_cstr(descr)
        name = descr['name']
        return cls(pe, uid, name)

    @property
    def uid(self):
        return self._uid


class Robot(Entity):
    """Robot."""

    def __init__(self, pe, bodies, name=None):
        # Physics engine API wrapper
        self._pe = pe
        # Joint name
        self._name = name
        # Add bodies
        self._bodies = bodies

    @ classmethod
    def create_from_descr(cls, pe, descr, data_dir='./data'):
        """Create an instance from description."""
        frame_xyz = descr['pose']['xyz']
        frame_rpy = descr['pose']['rpy']
        frame = (frame_xyz, frame_rpy)
        body_dict = {}
        for body_descr in descr['body']:
            body_name = body_descr['name']
            body = Body.create_from_descr(pe, body_descr, data_dir, frame)
            body_dict[body_name] = body
        name = descr['name']
        return cls(pe, body_dict, name)

    def setup_ctrl_handlers(self, ctrl_listener):
        """Setup the controller handlers. """
        if type(ctrl_listener) == type(None):
            pass
        elif type(ctrl_listener) == controller.KeyListener:
            self._setup_key_handlers(ctrl_listener)
        else:
            raise ValueError('Unrecognized ctrl_listener type {}.'\
                    .format(type(ctrl_listener)))

    def _setup_key_handlers(self, ctrl_listener):
        """Setup the keyboard handlers. """
        raise NotImplementedError

    def _setup_vr_handlers(self, ctrl_listener):
        """Setup the VR handlers. """
        raise NotImplementedError

    def _add_cstr(self, parent_body, parent_link, child_body, child_link,
            joint_type, joint_axis, parent_frame_pos, child_frame_pos,
            parent_frame_quat=None, child_frame_quat=None, name=None):
        """Add a joint constraint."""
        cstr = {
                'name': '{:s}/{:s}'.format(self._name, name),
                'parent_body': parent_body,
                'parent_link': parent_link,
                'child_body': child_body,
                'child_link': child_link,
                'joint_type': joint_type,
                'joint_axis': joint_axis,
                'parent_frame_pos': parent_frame_pos,
                'child_frame_pos': child_frame_pos,
                'parent_frame_quat': parent_frame_quat,
                'child_frame_quat': child_frame_quat,
                'uid': None
                }
        return cstr

    @property
    def bodies(self):
        return self._bodies


class Gripper(Robot):
    """Robotic gripper."""

    def __init__(self, pe, body_dict, name=None):
        super(Gripper, self).__init__(pe, body_dict, name)
        self._gripper = self.bodies['gripper']
        # Constraints
        cstr_descr = self._add_cstr(self._gripper.uid,
                -1, -1, -1, 'fixed', [0 ,0, 0], [0, 0, 0], self._gripper.pos)
        self._base_cstr = Constraint.create_from_descr(pe, cstr_descr)
        # Specs
        self._max_force_move = 1000.0
        self._grip_cstr = None
        self._state = 0
        #
        self.release()

    @property
    def state(self):
        return self._state

    def _setup_key_handlers(self, ctrl_listener):
        """Set up the keyboard handlers."""
        # Controller hyperparameters
        trans = 0.01
        rot = 0.03
        delay = 0.03

        # if self._first_person:
        if True: # TODO
            # Translate up in the camera frame
            ctrl_listener.on_key_event('f', 'DOWN', [], delay=delay,
                    handler=lambda: self.first_person_move([0, 0, trans]))
            # Translate down in the camera frame
            ctrl_listener.on_key_event('d', 'DOWN', [], delay=delay,
                    handler=lambda: self.first_person_move([0, 0, -trans]))
            # Translate forward in the camera frame
            ctrl_listener.on_key_event('UP', 'DOWN', [], delay=delay,
                    handler=lambda: self.first_person_move([trans, 0, 0]))
            # Translate backward in the camera frame
            ctrl_listener.on_key_event('DOWN', 'DOWN', [], delay=delay,
                    handler=lambda: self.first_person_move([-trans, 0, 0]))
            # Translate left in the camera frame
            ctrl_listener.on_key_event('LEFT', 'DOWN', [], delay=delay,
                    handler=lambda: self.first_person_move([0, trans, 0]))
            # Translate right in the camera frame
            ctrl_listener.on_key_event('RIGHT', 'DOWN', [], delay=delay,
                    handler=lambda: self.first_person_move([0, -trans, 0]))
        else:
            # Translate up
            ctrl_listener.on_key_event('f', 'DOWN', [], delay=delay,
                    handler=lambda: self.move([0, 0, trans]))
            # Translate down
            ctrl_listener.on_key_event('d', 'DOWN', [], delay=delay,
                    handler=lambda: self.move([0, 0, -trans]))
            # Translate forward
            ctrl_listener.on_key_event('UP', 'DOWN', [], delay=delay,
                    handler=lambda: self.move([trans, 0, 0]))
            # Translate backward
            ctrl_listener.on_key_event('DOWN', 'DOWN', [], delay=delay,
                    handler=lambda: self.move([-trans, 0, 0]))
            # Translate left
            ctrl_listener.on_key_event('LEFT', 'DOWN', [], delay=delay,
                    handler=lambda: self.move([0, trans, 0]))
            # Translate right
            ctrl_listener.on_key_event('RIGHT', 'DOWN', [], delay=delay,
                    handler=lambda: self.move([0, -trans, 0]))

        # Rotate along x
        ctrl_listener.on_key_event('f', 'DOWN', ['SHIFT'], delay=delay,
                handler=lambda: self.move([0, 0, 0], [rot, 0, 0]))
        # Rotate along x reversely
        ctrl_listener.on_key_event('d', 'DOWN', ['SHIFT'], delay=delay,
                handler=lambda: self.move([0, 0, 0], [-rot, 0, 0]))
        # Rotate along y
        ctrl_listener.on_key_event('UP', 'DOWN', ['SHIFT'], delay=delay,
                handler=lambda: self.move([0, 0, 0], [0, rot, 0]))
        # Rotate along y reversely
        ctrl_listener.on_key_event('DOWN', 'DOWN', ['SHIFT'], delay=delay,
                handler=lambda: self.move([0, 0, 0], [0, -rot, 0]))
        # Rotate along z
        ctrl_listener.on_key_event('LEFT', 'DOWN', ['SHIFT'], delay=delay,
                handler=lambda: self.move([0, 0, 0], [0, 0, rot]))
        # Rotate along z reversely
        ctrl_listener.on_key_event('RIGHT', 'DOWN', ['SHIFT'], delay=delay,
                handler=lambda: self.move([0, 0, 0], [0, 0, -rot]))

        # Grip
        ctrl_listener.on_key_event(' ', 'DOWN', None, delay=0.0,
                handler=lambda: self.grip())
        # Release
        ctrl_listener.on_key_event(' ', 'RELEASED', None, delay=0.0,
                handler=lambda: self.release())

    def move(self, trans=None, euler=None, scale=1.):
        """Move the gripper given translation and rotation.

        Args:
            trans: A 3-dimensional vector of translation.
            euler: A 3-dimensional Euler rotation angle.
            scale: The scale of the translation.
        """
        if trans is None:
            trans = np.zeros((3,), dtype=np.float32)
        if euler is None:
            euler = np.zeros((3,), dtype=np.float32)
        old_pos, old_euler = self.pe.get_cstr_dof(self._base_cstr.uid)
        new_pos = old_pos + np.array(trans) * scale
        new_euler = old_euler + euler
        self.move_to(new_pos, new_euler)

    def first_person_move(self, trans=None, euler=None, scale=1.):
        """Move the gripper given translation and rotation.
        Args:
            trans: A 3-dimensional vector of translation.
            euler: A 3-dimensional Euler rotation angle.
            scale: The scale of the translation.
        """
        if trans is None:
            trans = np.zeros((3,), dtype=np.float32)
        if euler is None:
            euler = np.zeros((3,), dtype=np.float32)
        old_pos, old_euler = self.pe.get_cstr_dof(self._base_cstr.uid)
        frame = ([0, 0, 0], self._gripper.euler) # TODO
        trans_in_frame = self.pe.pos_in_frame(trans, frame)
        self.move(trans_in_frame, euler, scale)

    def move_to(self, pos, euler=None):
        """Move the gripper to the specified position and euler angle."""
        self.pe.set_cstr_dof(self._base_cstr.uid, pos, euler,
                max_force=self._max_force_move)

    def cstr_grip(self, obj_body):
        self._state = 1
        gmat = self._pe.get_body_mat33(self._gripper.uid)
        opos = np.dot(obj_body.pos - self._gripper.pos, np.linalg.inv(gmat).T)
        cstr_descr = self._add_cstr(self._gripper.uid,
                -1, obj_body.uid, -1, 'fixed', [0, 0, 0], opos, [0, 0, 0],
                                    parent_frame_quat=[0, 0, 0],
                                    child_frame_quat=self._gripper.quat)
        self._grip_cstr = Constraint.create_from_descr(self._pe, cstr_descr)

    def cstr_release(self):
        self._state = 0
        if self._grip_cstr:
            self._pe.remove_cstr(self._grip_cstr.uid)
            self._grip_cstr = None

    def grip(self):
        self._state = 1
        """Grip."""
        left_joint = self.bodies['gripper'].joints['left_gripper_joint']
        left_joint.control_pos(left_joint.limit['lower'])
        right_joint = self.bodies['gripper'].joints['right_gripper_joint']
        right_joint.control_pos(right_joint.limit['lower'])

    def release(self):
        self._state = 0
        """Release the gripper."""
        left_joint = self.bodies['gripper'].joints['left_gripper_joint']
        left_joint.control_pos(left_joint.limit['upper'])
        right_joint = self.bodies['gripper'].joints['right_gripper_joint']
        right_joint.control_pos(right_joint.limit['upper'])

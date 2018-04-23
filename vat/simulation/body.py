import os.path as osp
import numpy as np


class Entity(object):
    """Entity base class."""
    def __init__(self, pe, name=None):
        # Physics engine API wrapper
        self._pe = pe
        # Link name
        self._name = name

    @property
    def pe(self):
        return self._pe

    @property
    def name(self):
        return self._name


class Link(Entity):
    """Link."""
    def __init__(self, pe, uid, name=None):
        # Physics engine API wrapper
        self._pe = pe
        # Link unique ID
        self._uid = uid
        # Link name
        self._name = name

    @property
    def uid(self):
        return self._uid


class Joint(Entity):
    """Joint."""
    def __init__(self, pe, uid, limit, name=None):
        # Physics engine API wrapper
        self._pe = pe
        # Joint unique ID
        self._uid = uid
        # Joint name
        self._name = name
        # Joint limit
        self._limit = limit
        # Set joint initial pose

    def control_pos(self, pos, max_vel=None, max_force=None):
        """ """
        if max_vel is None:
            max_vel = self.limit['velocity']
        if max_force is None:
            max_force = self.limit['effort']
        self.pe.control_joint_pos(
                self.uid[0],
                self.uid[1],
                pos=pos,
                max_vel=max_vel,
                max_force=max_force)

    def control_vel(self, vel, max_force=None):
        """ """
        if max_force is None:
            max_force = self.limit['effort']
        self.pe.control_joint_vel(
                self.uid[0],
                self.uid[1],
                vel=vel,
                max_force=max_force)

    def control_torque(self, torque):
        """ """
        self.pe.control_joint_vel(
                self.ud[0],
                self.uid[1],
                torque=torque)

    @property
    def uid(self):
        return self._uid

    @property
    def limit(self):
        return self._limit


class Body(Entity):
    """Body."""

    def __init__(self, pe, uid, name=None, boundary=None, scale=None):
        # Physics engine API wrapper
        self._pe = pe
        # Body unique ID
        self._uid = uid
        # Name
        self._name = name
        # Body links
        self._links = {}
        for link_uid in self.pe.get_link_uids(uid):
            link_name = self.pe.get_link_name(uid, link_uid)
            link = Link(self.pe, (uid, link_uid), link_name)
            self._links[link_name] = link
        # Body joints
        self._joints = {}
        for joint_uid in self.pe.get_joint_uids(uid):
            joint_name = self.pe.get_link_name(uid, joint_uid)
            limit = self.pe.get_joint_limit(uid, joint_uid)
            # if initial is not None:
            #     self.pe.reset_joint(uid, joint_uid, initial[joint_uid])
            joint = Joint(self.pe, (uid, joint_uid), limit, joint_name)
            self._joints[joint_name] = joint

        self._boundary = boundary
        self._scale = scale

    @ classmethod
    def create_from_descr(cls, pe, descr, data_dir='./data', frame=None):
        """Create an instance from description."""
        filename = descr['model']['filename']
        boundary, scale = None, None
        if 'boundary' in descr:
            boundary = np.array(descr['boundary'])
        if 'scale' in descr:
            scale = np.array(descr['scale'])

        xyz = descr['pose']['xyz']
        rpy = descr['pose']['rpy']
        if frame is not None:
            xyz = pe.pos_in_frame(xyz, frame)
            rpy = pe.euler_in_frame(rpy, frame)
        fixed = descr['fixed']
        path = osp.join(data_dir, filename)
        uid = pe.load(path, xyz, rpy, fixed)
        name = descr['name']
        return cls(pe, uid, name, boundary, scale)

    @property
    def boundary(self):
        return self._boundary

    @property
    def scale(self):
        return self._scale

    @property
    def uid(self):
        return self._uid

    @property
    def links(self):
        return self._links

    @property
    def joints(self):
        return self._joints

    @property
    def pos(self):
        return self.pe.get_body_pos(self.uid)

    @pos.setter
    def pos(self, value):
        self.pe.set_body_pos(self.uid, value)

    @property
    def quat(self):
        return self.pe.get_body_quat(self.uid)

    @quat.setter
    def quat(self, value):
        self.pe.set_body_quat(self.uid, value)

    @property
    def euler(self):
        return self.pe.get_body_euler(self.uid)

    @euler.setter
    def euler(self, value):
        self.pe.set_body_euler(self.uid, value)

    def apply_force(self, vec):
        self.pe.apply_force(self.uid, -1, vec, self.pos)

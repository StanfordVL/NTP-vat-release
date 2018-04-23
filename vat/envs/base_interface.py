"""
Simulation / Robot environment base wrapper class
"""


import numpy as np


class BaseInterface(object):

    def set_callback(self, callback_func, freq):
        pass

    def unset_callback(self):
        pass

    @property
    def obj(self):
        raise NotImplementedError

    @property
    def carrying(self):
        """the name of the object being carried"""
        return self._carrying

    @property
    def gpos(self):
        raise NotImplementedError

    @property
    def gorn(self):
        raise NotImplementedError

    def add_body(self):
        raise NotImplementedError

    def absolute_pos(self, objname):
        pos = self.obj[objname].pos
        return pos

    def relative_pos(self, objname):
        pos = self.obj[objname].pos
        rpos = pos - self.gpos
        return rpos

    def reset(self):
        raise NotImplementedError

    def start(self):
        raise NotImplementedError

    def reset_custom(self):
        raise NotImplementedError

    def grip(self, obj_name):
        raise NotImplementedError

    def release(self):
        raise NotImplementedError

    def wait(self, x):
        raise NotImplementedError

    def is_on_top_of(self, o1_name, o2_name, eps=None):
        """
        check if o1 is on top of o2
        """
        o1 = self.obj[o1_name]
        o2 = self.obj[o2_name]
        if eps is None:
            xy_error = o2.scale[:2].max()
        else:
            xy_error = eps

        distance = o1.pos - o2.pos

        xy_dist = np.linalg.norm(distance[:2])
        # print(xy_dist)
        z_dist = distance[2]
        return xy_dist <= xy_error and z_dist > 0

    def no_collision_with(self, o1pos, o1boundary, o2name):
        """
        check if o1pos is outside of the collision boundary of o2
        given boundary of o1
        args:
            o1pos: position of object o1
            o1boundary: collision boundary of object o1
            o2: body of object o2
        """
        o2 = self.obj[o2name]
        min_dist = max(o1boundary[:2]) / 2 + max(o2.boundary[:2]) / 2
        dist = np.linalg.norm((o1pos - o2.pos)[:2])
        return dist > min_dist

    def pos_on_top_of(self, o1boundary, o2name):
        """
        compute position directly on top of o2
        args:
            o1boundary: collision boundary of o1
            o2: body of o2
        """
        o2 = self.obj[o2name]
        tpos = o2.pos
        tpos[2] += o1boundary + o2.boundary
        return tpos

    def _move_to(self, pos, orn, speed=0.1):
        raise NotImplementedError

    def move_to_z(self, h, orn=None, speed=0.1):
        gpos = self.gpos
        gpos[2] = h
        return self.move_to_xyz(gpos, orn)

    def move_relative_z(self, rh, speed=0.1):
        gpos = self.gpos
        gpos[2] += rh
        return self.move_to_xyz(gpos, None, speed=speed)

    def move_to_xy(self, pos, orn=None, speed=0.1):
        assert(len(pos) == 2)
        pos = np.array(pos)
        gpos = self.gpos
        gpos[:2] = pos
        return self.move_to_xyz(gpos, orn, speed=speed)

    def move_to_xyz(self, pos, orn=None, speed=0.1):
        self._move_to(pos, orn, speed=speed)
        return True

import numpy as np
from base_interface import BaseInterface
from IPython import embed
import time

OFFSETS = {'gripper_z': np.array([0, 0, 0.32])}

ORNS = {'gripper_x': np.array([np.pi / 2, np.pi / 2, 0]),
        'gripper_y': np.array([0, np.pi / 2, 0])}

POSES = {'gripper_reset': (np.array([0, 0, 1.32]), ORNS['gripper_y'])}


def error_sign(tgt, src, error, radius=False):
    diff = (tgt - src)
    if radius:
        for i, d in enumerate(diff):
            if d > np.pi / 2:
                diff[i] -= np.pi / 2
            elif d < - np.pi / 2:
                diff[i] += np.pi / 2
    pc = (diff > error).astype(np.float32)
    nc = (diff < -error).astype(np.float32) * -1
    return pc + nc


def to_np(arr):
    l = isinstance(arr, list)
    t = isinstance(arr, tuple)
    if l or t:
        return np.array(arr)
    elif isinstance(arr, np.ndarray):
        return arr
    else:
        raise NotImplementedError('in correct pos type')


class BulletInterface(BaseInterface):

    def __init__(self, world, pos_step=0.001, orn_step=0.001):
        self.bullet = world
        self.gripper = self.bullet.robots['pr2_gripper']
        self.pos_error = to_np([1, 1, 1]) * pos_step
        self.orn_error = to_np([1, 1, 1]) * orn_step
        self.pos_step_size = self.pos_error
        self.orn_step_size = self.orn_error

        self.max_single_move_step = 1000
        self.max_world_tick = 500000
        self.time_out = False
        self.world_tick = 0
        self.step_callback = None
        self.callback_freq = 100
        self._carrying = None
        self._stats = {}

    def set_callback(self, callback_func, freq):
        self.step_callback = callback_func
        self.callback_freq = freq

    def unset_callback(self):
        self.step_callback = None

    @property
    def obj(self):
        return self.bullet.bodies

    def add_body(self, spec):
        self.bullet.add_body(spec)

    @property
    def gpos(self):
        gpos = to_np(self.obj['gripper'].pos)
        return gpos - OFFSETS['gripper_z']

    @property
    def gorn(self):
        return to_np(self.obj['gripper'].euler)

    def reset_gripper(self):
        self._set_to(*POSES['gripper_reset'])
        for _ in xrange(100):
            self.bullet.step()

    def reset(self):
        self.bullet.restart()
        self.gripper = self.bullet.robots['pr2_gripper']

    def start(self):
        self.reset_gripper()
        self.world_tick = 0
        self.time_out = False
        self._carrying = None

    def grip(self, obj_name):
        self._carrying = obj_name
        self.gripper.cstr_grip(self.obj[obj_name])
        # self.gripper.grip()

    def release(self):
        # self.gripper.release()
        self._carrying = None
        self.gripper.cstr_release()

    def move_to_above(self, obj_name, orn=None):
        # preventive move
        self.move_relative_z(0.2)

        obj = self.obj[obj_name]
        tgt_pos = obj.pos
        tgt_pos[2] += 0.2
        return self.move_to_xyz(tgt_pos)

    def reach_to_grasp(self, obj_name):
        obj = self.obj[obj_name]
        h = 0.02  # grasp heights
        pos = obj.pos
        pos[2] += h
        success = self.move_to_z(pos[2])
        return success

    def reach_to_drop(self, obj_name):
        obj = self.obj[obj_name]
        if not self.carrying:
            return False
        cobj = self.obj[self.carrying]

        g_to_cobj = (self.gpos - cobj.pos)[2]
        g_to_cobj_bottom = g_to_cobj + cobj.scale[2] / 2

        obj_top = obj.pos[2] + obj.scale[2] / 2

        target_z = obj_top + g_to_cobj_bottom + 0.02
        success = self.move_to_z(target_z)
        return success

    def reach_to_press(self, obj_name, obj2_name, loc):
        obj = self.obj[obj_name]
        obj2 = self.obj[obj2_name]
        if not self.carrying:
            return False
        cobj = self.obj[self.carrying]

        g_to_cobj = (self.gpos - cobj.pos)[2]
        g_to_cobj_bottom = g_to_cobj + cobj.scale[2] / 2

        obj_top = obj.pos[2] + obj.scale[2] / 2
        print loc
        if loc == 1:
            target_y, target_x = -0.06 + \
                obj.pos[1], -.02 + (obj.pos[0] + obj2.pos[0]) / 2
        elif loc == 2:
            target_y, target_x = -0.22 + \
                obj.pos[1], -.02 + (obj.pos[0] + obj2.pos[0]) / 2

        print target_x, target_y

        target_z = obj_top + g_to_cobj_bottom + .02
        success = 0
        print cobj.pos,  obj.pos, obj2.pos
        print "REACH TO PRESS", cobj._name
        success = self.move_to_xy([target_x, target_y])
        print cobj.pos,  obj.pos, obj2.pos
        print "CORRECT XY"
        success = self.move_to_z(target_z)

        print target_z, cobj.pos[2], obj.pos[2], success
        return success

    def wait(self, x):
        for _ in xrange(x):
            self.bullet.step()

    def step_simulation(self):
        self.bullet.step()
        if self.step_callback and self.world_tick % self.callback_freq == 0:
            self.step_callback()
        self.world_tick += 1
        if self.world_tick > self.max_world_tick:
            self.time_out = True
            print('time out!')

    def reach_error_sign(self, pos, orn=None):
        pr = error_sign(pos, self.gpos, self.pos_error)
        if not orn:
            rr = np.zeros(3).astype(np.float32)
        else:
            rr = error_sign(orn, self.gorn, self.orn_error, radius=True)
        return pr, rr

    def _move_to(self, pos, orn=None, speed=0.1):
        pr, rr = self.reach_error_sign(pos, orn)

        count = 0
        while not np.all(pr == 0) or not np.all(rr == 0):
            self._step_move(pr, rr)
            self.step_simulation()
            pr, rr = self.reach_error_sign(pos, orn)
            count += 1
            if count > self.max_single_move_step:
                return

    def _step_move(self, trans, orn=None):
        if orn is not None:
            orn_step = orn * self.orn_step_size
        pos_step = trans * self.pos_step_size
        self.gripper.move(pos_step, orn_step)

    def _set_to(self, pos, orn):
        self.gripper.move_to(pos, orn)

    def perturb(self, oname):
        o = self.obj[oname]
        pos = o.pos
        if pos[2] > 0.66:
            pos[0] += 0.08
            o.pos = pos
        self.wait(1000)

"""
Action and object interface for the task environment
"""


import numpy as np
import abc


def action(f):
    def wrapper(*args):
        result = f(*args)
        world = args[0]

        world.check_n_satisfied()
        if result is None:
            done = False
            reward = 0
            stats = world.stats
            if world.interface.time_out:
                done = True

        else:
            reward, done, stats = result
        return reward, done, stats
    return wrapper


class TaskObject(object):

    def __init__(self, name, index, interface):
        self._name = name
        self._index = index
        self._interface = interface

        self._instances = []
        self._masked_instances = []
        self._locked_instance = None

    @property
    def name(self):
        return self._name

    @property
    def instances(self):
        return self._instances

    @property
    def index(self):
        return self._index

    def add_instance(self, spec):
        spec = spec.copy()
        pfix = '_%i' % len(self.instances)
        spec['name'] += pfix
        self._interface.add_body(spec)
        self._instances.append(spec['name'])

    def has_instance(self, instance):
        return instance in self._instances

    def reset(self):
        self._instances = []
        self._locked_instance = None
        self._masked_instances = []

    def mask_instance(self, instance):
        assert(instance in self._instances)
        self._masked_instances.append(instance)

    @property
    def locked_instance(self):
        return self._locked_instance

    def lock_nearest_instance(self):
        instances = []
        for o in self.instances:
            if o not in self._masked_instances:
                instances.append(o)
        if len(instances) == 0:
            instances = self.instances

        comp_func = lambda name: np.linalg.norm(
            self._interface.relative_pos(name))
        ni = min(instances, key=comp_func)
        self._locked_instance = ni


class BaseWorld(object):

    def __init__(self, interface, scene_specs):
        self.interface = interface
        self._dimensions = None
        self.scene_specs = scene_specs
        self.task_objects = []
        for i, o in enumerate(scene_specs['task_objects']):
            self.task_objects.append(TaskObject(o, i, interface))

    def start_world(self):
        raise NotImplementedError

    def reset_world(self):
        raise NotImplementedError

    @property
    def state(self):
        return {'object_state': self.object_state,
                'agent_state': self.agent_state,
                'images': self.image}

    @abc.abstractproperty
    def image(self):
        """return object position relative to the gripper"""
        return np.array(self.interface.bullet.capture_image())

    @property
    def depth(self):
        """return object position relative to the gripper"""
        return np.array(self.interface.bullet.depth)

    @property
    def object_state(self):
        """return object position relative to the gripper"""
        poses = []

        for tobj in self.task_objects:
            poses.append(self.interface.relative_pos(tobj.locked_instance))

        a = np.vstack(poses).astype(np.float32)
        return a

    @property
    def agent_state(self):
        state = np.zeros(2, dtype=np.float32)
        state[self.interface.gripper.state] = 1
        return state

    @property
    def dimensions(self):
        if not self._dimensions:
            self._dimensions = {'object_state': [len(self.task_objects), 3],
                                'agent_state': self.agent_state.shape,
                                'images': self.image.shape}
        return self._dimensions

    @property
    def all_object_instances(self):
        all_inst = []
        for tobj in self.task_objects:
            all_inst += tobj.instances
        return all_inst

    def ind_to_name(self, query_idx):
        """
        object index to task object name
        """
        return self.task_objects[query_idx].name

    def name_to_ind(self, query_name):
        """
        object name to task object index
        """
        for i, tobj in enumerate(self.task_objects):
            if tobj.name == query_name:
                return i
        return None

    def get_task_object(self, query):
        if isinstance(query, basestring):
            query = self.name_to_ind(query)
        return self.task_objects[query]

    def get_object_instance(self, query):
        return self.get_task_object(query).locked_instance

    def set_callback(self, callback_func, freq):
        self.interface.set_callback(callback_func, freq)

    def unset_callback(self):
        self.interface.unset_callback()

    @action
    def action_move_to(self, obj):
        """move to above"""
        obj = self.get_object_instance(obj)
        self.interface.move_to_above(obj)

    @action
    def action_grasp(self, obj):
        """reach and grip"""
        obj = self.get_object_instance(obj)
        self.interface.reach_to_grasp(obj)
        self.interface.grip(obj)

    @action
    def action_drop(self, obj):
        """reach and release"""
        obj = self.get_object_instance(obj)
        self.interface.reach_to_drop(obj)
        self.interface.release()

    @action
    def action_press(self, obj, obj2, loc):
        """reach and release"""
        obj = self.get_object_instance(obj)
        obj2 = self.get_object_instance(obj2)
        self.interface.reach_to_press(obj, obj2, loc)
        self.interface.release()

    @action
    def action_release(self):
        """release the gripped object"""
        self.interface.wait(10)
        self.interface.release()

    @action
    def action_noop(self):
        return None

    def satisfied(self, cstr):
        """
        check if a condition is satisfied
        """
        all_satisfied = True
        n_satisfied = 0
        if cstr['type'] == 'on_top':
            src_objs = self.get_task_object(cstr['src']).instances
            tgt_obj = self.get_task_object(cstr['target']).instances[0]
            for so in src_objs:
                if not self.interface.is_on_top_of(so, tgt_obj):
                    all_satisfied = False
                else:
                    n_satisfied += 1
        else:
            raise NotImplementedError(
                'cannot check condition! %s' % cstr['type'])

        return all_satisfied, n_satisfied

    def mask_object(self, oname):
        """
        mask out an object with the given name
        """
        for tobj in self.task_objects:
            if tobj.has_instance(oname):
                tobj.mask_instance(oname)

    def lock_task_objects(self):
        """
        for each task object, find the nearest instance and lock it
        """
        for tobj in self.task_objects:
            tobj.lock_nearest_instance()

    def add_instance(self, spec):
        tobj = self.get_task_object(spec['name'])
        tobj.add_instance(spec)

"""
Simulated environment wrapper
"""

import numpy as np
from base_world import BaseWorld
from builtins import range


class SimWorld(BaseWorld):

    @property
    def image(self):
        """return object position relative to the gripper"""
        return np.array(self.interface.bullet.capture_image())

    def start_world(self):
        self.setup_scene()
        self.reset_custom()
        self.interface.start()

    def reset_world(self):
        self.interface.reset()
        self.start_world()

    def setup_scene(self):
        """
        Setup the task scene given the scene specs
        """
        for tobj in self.task_objects:
            tobj.reset()

        workspace = self.scene_specs['workspace']
        objects = self.scene_specs['objects']
        # divide workspace into xy grid
        xmin, xmax = workspace['xlim']
        ymin, ymax = workspace['ylim']
        grid_size = workspace['size']

        if 'pos_eps' in workspace:
            pos_eps = workspace['pos_eps']
        else:
            pos_eps = 0

        grid_x, grid_y = np.mgrid[xmin:xmax:grid_size, ymin:ymax:grid_size]
        grid = np.vstack((grid_x.flatten(), grid_y.flatten())).T
        grid = np.random.permutation(grid)
        eps = (np.random.random(grid.shape) * 2 - 1) * pos_eps
        # [-pos_eps, pos_eps]
        grid += eps

        def get_spec(ospec):
            s = {'name': ospec['name'],
                 'model': {'filename': ospec['filename']},
                 'fixed': ospec['fixed'],
                 'boundary': ospec['boundary'],
                 'scale': ospec['scale'],
                 'pose': {'rpy': None, 'xyz': None}}
            return s

        def find_valid_pos(s, z, grid):
            for x, y in grid:
                tpos = np.array([x, y, z])
                if self.no_collision(tpos, s['boundary']):
                    return tpos
            raise ValueError('cannot find valid position')

        # resolve fixed first
        for ospec in objects:
            if ospec['pose']['type'] == 'preset':
                s = get_spec(ospec)
                s['pose']['rpy'] = ospec['pose']['rpy']
                s['pose']['xyz'] = ospec['pose']['xyz']
                self.add_instance(s)

        # resolve random
        for ospec in objects:
            if ospec['pose']['type'] == 'random':
                s = get_spec(ospec)
                s['pose']['rpy'] = ospec['pose']['rpy']
                z = ospec['pose']['z']
                s['pose']['xyz'] = find_valid_pos(s, z, grid)
                self.add_instance(s)
            elif ospec['pose']['type'] == 'random_repeat':
                repeat_range = ospec['pose']['n_repeat']
                z = ospec['pose']['z']
                n_repeat = np.random.choice(
                    np.arange(*repeat_range), size=1)[0]
                for i in range(n_repeat):
                    s = get_spec(ospec)
                    s['pose']['xyz'] = find_valid_pos(s, z, grid)
                    s['pose']['rpy'] = ospec['pose']['rpy']
                    self.add_instance(s)
            elif ospec['pose']['type'] == 'repeat':
                repeat_range = ospec['pose']['n_repeat']
                n_repeat = np.random.choice(
                    np.arange(*repeat_range), size=1)[0]
                for i in range(n_repeat):
                    s = get_spec(ospec)
                    s['pose']['xyz'] = ospec['pose']['xyz']
                    s['pose']['rpy'] = ospec['pose']['rpy']
                    self.add_instance(s)

        # resolve dependencies
        for ospec in objects:
            if ospec['pose']['type'] == 'on_top':
                s = get_spec(ospec)
                s['pose']['rpy'] = ospec['pose']['rpy']
                s['pose']['xyz'] = self.pos_on_top_of(s['boundary'],
                                                      ospec['pose']['target'])
                self.add_instance(s)

    def no_collision(self, tpos, boundary):
        """
        check if tpos has any collision with other object
        tpos:
            tpos: target position
            boundary: object boundary of the target position
        """
        for o in self.all_object_instances:
            if not self.interface.no_collision_with(tpos, boundary, o):
                return False
        return True

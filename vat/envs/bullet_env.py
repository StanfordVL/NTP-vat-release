from vat.simulation import get_world
from api import get_api, get_task_world
from bullet_interface import BulletInterface


class BulletEnv:

    def __init__(self, scene_file, time_step, display,
                 data_dir, verbose=False, key=None):
        self.sim = get_world('bullet', display, data_dir, verbose, key=key)

        # load world
        self.sim.load(scene_file)
        # start simulation
        self.sim.start(time_step)

    def close(self):
        self.sim.close()

    def configure(self, config, scene_specs):
        TaskWorld = get_task_world(config['task_name'], real=False)
        self.task_name = config['task_name']
        interface = BulletInterface(self.sim)
        self.world = TaskWorld(interface,
                               scene_specs,
                               random_task=config['random_task'])

        self.api = get_api(config['api'])(self.world, config['full_demo'])
        print('API: %s' % config['api'])

        for act in self.api.ACT:
            assert(act in self.program_names)

        for adp in self.api.ADAPTIVE:
            assert(adp in self.program_names)

        try:
            self.world.start_world()
        except ValueError:
            print('setup scene failed, retry')
            self.world.reset_world()

    @property
    def dimensions(self):
        return self.world.dimensions

    def change_task(self, task):
        self.world.set_task(task)

    def reset(self):
        try:
            self.world.reset_world()
        except ValueError:
            print('setup scene failed, retry')
            self.world.reset_world()

    def start_task(self, video_logging=False):
        # Logs video of task from start to finish
        if video_logging:
            self.sim.log_video(self.task_name)
        self.world.start_task()

    def step(self, action):
        program_ind, args = action
        reward, done, info = self.api.programs[program_ind][1](args)
        return self.state, reward, done, info

    @property
    def programs(self):
        return self.api.programs

    @property
    def program_names(self):
        return self.api.program_names

    @property
    def vocabs(self):
        return self.api.vocabs

    @property
    def program_to_ind(self):
        return self.api.program_to_ind

    @property
    def ACT(self):
        return [self.api.program_to_ind[p] for p in self.api.ACT]

    @property
    def ADAPTIVE(self):
        return [self.api.program_to_ind[p] for p in self.api.ADAPTIVE]

    @property
    def entry_point(self):
        return self.api.entry_point

    def expert_program_trace(self):
        return self.api.expert_program_trace()

    @property
    def state(self):
        return self.world.state

    def task_done(self):
        return self.world.task_done

    @property
    def num_task(self):
        return self.world.num_task

    @property
    def task_objects(self):
        return self.world.task_objects

    def change_scene(self, scene_specs):
        self.world.scene_specs = scene_specs

    @property
    def stats(self):
        stats = self.world.stats
        stats['n_satisfied'] = self.world.check_n_satisfied()
        stats['n_task'] = len(self.world.task)
        return stats

    def wait(self, n_step):
        self.world.wait(n_step)

from sim_world import SimWorld
try:
    from robot_world import RobotWorld
except ImportError:
    RobotWorld = object
    print('Robot World cannot be imported')
import time
from npi_view import NPIView
import numpy as np
import threading
np.random.seed(10)


def get_api(api_name):
    """
    factory function for APIs
    """
    if api_name == 'flat':
        return FlatAPI
    elif api_name == 'full':
        return FullAPI
    else:
        raise NotImplementedError('%s is not a valid API' % api_name)


class Observer(threading.Thread):

    def __init__(self, api):
        self.__api = api
        threading.Thread.__init__(self)

    def run(self):
        while True:
            self.__api.observe()
            time.sleep(1)
            if self.__api.stop_obs:
                break


class FullAPI(NPIView):

    def __init__(self, sw, full_demo=False, robot=False):
        NPIView.__init__(self)
        self.world = sw
        self._state_log_tmp = {'object_states': [],
                               'agent_states': [],
                               'commands': [],
                               'images': []}
        self.full_demo = full_demo
        self.target = None
        self.isrobot = robot
        if self.isrobot:
            self.stop_obs = False

        # Only needed if generating a robot demo
        # self.obs = Observer(self)
        # self.obs.start()

    def observe(self):
        self.state_log['object_states'].append(self.world.object_state)
        self.state_log['agent_states'].append(self.world.agent_state)
        self.state_log['images'].append(self.world.image)

        if self.world.interface.time_out:
            self.success = False

    @property
    def current_frame(self):
        return len(self.state_log['object_states']) - 1

    def command(self, words):
        for w in words:
            assert(w in self.vocabs)
            cmd = np.zeros(len(self.vocabs), dtype=np.float32)
            cmd_ind = self.vocabs.index(w)
            cmd[cmd_ind] = 1
            self.state_log['commands'].append(cmd)

    @property
    def current_command(self):
        return len(self.state_log['commands'])

    @property
    def programs(self):
        return [('stack', self.program_noop),
                ('sorting', self.program_noop),
                ('pick_place', self.program_noop),
                ('pick_release', self.program_noop),
                ('pick', self.program_noop),
                ('place', self.program_noop),
                ('release', self.program_noop),
                ('move', self.program_move),
                ('move_grasp', self.program_move_grasp),
                ('move_drop', self.program_move_drop),
                ('move_release', self.program_move_release)]

    @property
    def expert_programs(self):
        return {'stack': self.expert_stack,
                'sorting': self.expert_sorting,
                'pick_place': self.expert_pick_place,
                'pick_release': self.expert_pick_release,
                'pick': self.expert_pick,
                'place': self.expert_place,
                'release': self.expert_release,
                'move': self.expert_move,
                'move_grasp': self.expert_move_grasp,
                'move_drop': self.expert_move_drop,
                'move_release': self.expert_move_release}

    @property
    def entry_point(self):
        if self.world.task_name == 'stacking':
            return (self.program_to_ind['stack'], 0)
        elif self.world.task_name == 'sorting':
            return (self.program_to_ind['sorting'], 0)
        else:
            raise NotImplementedError('unimplemented task')

    def program_move(self, target, full_demo=False):
        if full_demo:
            self.world.set_callback(self.observe, 100)
        r, d, s = self.world.action_move_to(target)
        self.world.unset_callback()
        self.target = target
        return r, d, s

    def program_move_grasp(self, args=None, full_demo=False):
        nstep = self.world.stats['n_step']
        if nstep < len(self.world.task):
            curr_task = self.world.task[nstep]
            gt_target = self.world.name_to_ind(curr_task['src'])
            if gt_target != self.target and not self.world.already_failed:
                self.world.stats['wrong_pick'] = 1

        if self.target is not None:
            if full_demo:
                self.world.set_callback(self.observe, 100)
            out = self.world.action_grasp(self.target)
            self.world.unset_callback()
            self.target = None
            return out
        return self.world.action_noop()

    def program_move_drop(self, args=None, full_demo=False):
        nstep = self.world.stats['n_step']
        if nstep < len(self.world.task):
            curr_task = self.world.task[nstep]
            gt_target = self.world.name_to_ind(curr_task['target'])
            if not self.world.already_failed:
                self.world.stats['n_step'] += 1
                if gt_target != self.target:
                    self.world.stats['wrong_place'] = 1

        if self.target is not None:
            if full_demo:
                self.world.set_callback(self.observe, 100)
            out = self.world.action_drop(self.target)
            # self.world.wait(500)
            self.world.unset_callback()
            self.target = None
            return out
        return self.world.action_noop()

    def program_move_release(self, args=None, full_demo=False):
        nstep = self.world.stats['n_step']
        if nstep < len(self.world.task):
            curr_task = self.world.task[nstep]
            gt_target = self.world.name_to_ind(curr_task['target'])
            if not self.world.already_failed:
                self.world.stats['n_step'] += 1
                if gt_target != self.target:
                    self.world.stats['wrong_place'] = 1

        if full_demo:
            self.world.set_callback(self.observe, 100)
        carrying = self.world.interface.carrying
        out = self.world.action_release()

        self.world.unset_callback()

        try:
            self.observe()
        except:
            pass
        if carrying is not None:
            self.world.mask_object(carrying)

        movs = self.world.lock_task_objects()
        # self.movs = max(movs)

        self.target = None
        return out

    def program_noop(self):
        return self.world.action_noop()

    def expert_stack(self, trace):
        if self.isrobot:
            self.stop_obs = False
            self.obs = Observer(self)
            self.obs.start()
        while True:
            self.curr_task, n_remain = self.world.next_task()
            if n_remain == 0:
                break

            self.call_expert(trace, 'pick_place')

            if not self.success:
                return
        caller_ptr, trace_ptr = self.call_stop(trace)  # end of program
        self.append_trace(caller_ptr, trace_ptr)
        self.success = self.world.task_done
        if self.isrobot:
            self.stop_obs = True
        # del self.obs

    def expert_sorting(self, trace):
        while True:
            self.curr_task, n_remain = self.world.next_task()
            if n_remain == 0:
                break

            self.call_expert(trace, 'pick_release')

            if not self.success:
                return

            # self.observe()

        caller_ptr, trace_ptr = self.call_stop(trace)  # end of program
        self.append_trace(caller_ptr, trace_ptr)
        self.success = self.world.task_done

    def expert_pick_place(self, trace):
        self.call_expert(trace, 'pick', command=['pick'])
        self.call_expert(trace, 'place', command=['place'], stop=True)

    def expert_pick_press(self, trace):
        self.call_expert(trace, 'pick', command=['pick'])
        self.call_expert(trace, 'press', command=['press'], stop=True)

    def expert_pick_release(self, trace):
        self.call_expert(trace, 'pick', command=['pick'])
        self.call_expert(trace, 'release', command=['release'], stop=True)

    def expert_pick(self, trace):
        pick_name = self.curr_task['src']
        pick_target = self.world.name_to_ind(pick_name)
        self.call_expert(trace, 'move', pick_target)
        self.call_expert(trace, 'move_grasp', stop=True)

    def expert_press(self, trace):
        place_name = self.curr_task['target']
        place_target = self.world.name_to_ind(place_name)
        self.call_expert(trace, 'move', place_target)
        self.call_expert(trace, 'move_press', stop=True)

    def expert_place(self, trace):
        place_name = self.curr_task['target']
        place_target = self.world.name_to_ind(place_name)
        self.call_expert(trace, 'move', place_target)
        self.call_expert(trace, 'move_drop', stop=True)

    def expert_release(self, trace):
        place_name = self.curr_task['target']
        place_target = self.world.name_to_ind(place_name)
        self.call_expert(trace, 'move', place_target)
        self.call_expert(trace, 'move_release', stop=True)

    def expert_move(self, trace):
        caller_ptr, trace_ptr = self.call_stop(trace)
        self.program_move(trace['in_args'], full_demo=self.full_demo)
        self.observe()
        self.append_trace(caller_ptr, trace_ptr)

    def expert_move_grasp(self, trace):
        caller_ptr, trace_ptr = self.call_stop(trace)
        self.program_move_grasp(trace['in_args'], full_demo=self.full_demo)
        self.observe()
        self.append_trace(caller_ptr, trace_ptr)

    def expert_move_release(self, trace):
        caller_ptr, trace_ptr = self.call_stop(trace)
        self.program_move_release(trace['in_args'], full_demo=self.full_demo)
        self.append_trace(caller_ptr, trace_ptr)
        self.observe()  # for sorting, make train and eval consistent
        # return movs

    def expert_move_press(self, trace):
        caller_ptr, trace_ptr = self.call_stop(trace)
        self.program_move_press(trace['in_args'], full_demo=self.full_demo)
        self.append_trace(caller_ptr, trace_ptr)
        self.observe()  # for sorting, make train and eval consistent

    def expert_move_drop(self, trace):
        caller_ptr, trace_ptr = self.call_stop(trace)
        self.program_move_drop(trace['in_args'], full_demo=self.full_demo)
        self.observe()
        self.append_trace(caller_ptr, trace_ptr)

    @property
    def ACT(self):
        return ['move', 'move_grasp', 'move_drop', 'move_release']

    @property
    def ADAPTIVE(self):
        return ['stack', 'sorting', 'pick_place', 'pick_release', 'pick', 'place', 'release']

    @property
    def vocabs(self):
        return ['pick', 'place', 'release'] + [k.name for k in self.world.task_objects]


class FlatAPI(FullAPI):

    @property
    def programs(self):
        return [('stack', self.program_noop),
                ('sorting', self.program_noop),
                ('move', self.program_move),
                ('move_grasp', self.program_move_grasp),
                ('move_drop', self.program_move_drop),
                ('move_release', self.program_move_release)]

    @property
    def expert_programs(self):
        return {'stack': self.expert_stack,
                'sorting': self.expert_sorting,
                'move': self.expert_move,
                'move_grasp': self.expert_move_grasp,
                'move_drop': self.expert_move_drop,
                'move_release': self.expert_move_release}

    def expert_stack(self, trace):
        # print "STARINT STACK"
        while True:
            curr_task, n_remain = self.world.next_task()
            if n_remain == 0:
                break

            pick_name = curr_task['src']
            place_name = curr_task['target']
            pick_target = self.world.name_to_ind(pick_name)
            place_target = self.world.name_to_ind(place_name)

            self.call_expert(trace, 'move', pick_target)
            self.call_expert(trace, 'move_grasp', command=['pick'])
            self.call_expert(trace, 'move', place_target)
            self.call_expert(trace, 'move_drop', command=['place'])

            if not self.success:
                break

        caller_ptr, trace_ptr = self.call_stop(trace)  # end of program
        self.append_trace(caller_ptr, trace_ptr)
        self.success = self.world.task_done

    def expert_sorting(self, trace):
        while True:
            curr_task, n_remain = self.world.next_task()
            if n_remain == 0:
                break

            pick_name = curr_task['src']
            place_name = curr_task['target']
            pick_target = self.world.name_to_ind(pick_name)
            place_target = self.world.name_to_ind(place_name)

            self.call_expert(trace, 'move', pick_target)
            self.call_expert(trace, 'move_grasp', command=['pick'])
            self.call_expert(trace, 'move', place_target)
            self.call_expert(trace, 'move_release', command=['release'])

            if not self.success:
                break

        self.observe()

        caller_ptr, trace_ptr = self.call_stop(trace)  # end of program
        self.append_trace(caller_ptr, trace_ptr)
        self.success = self.world.task_done

    @property
    def ACT(self):
        return ['move', 'move_grasp', 'move_drop', 'move_release']

    @property
    def ADAPTIVE(self):
        return ['stack', 'sorting']


def get_task_world(task_name, real=False):
    class TaskWorld(RobotWorld if real else SimWorld):

        def __init__(self, *args, **kwargs):
            random_task = kwargs.pop('random_task')
            if real:
                RobotWorld.__init__(self, *args, **kwargs)
            else:
                SimWorld.__init__(self, *args, **kwargs)

            self.task_specs = None
            self._task = []
            self.random_task = random_task
            self.task_name = task_name
            self._init_stats()

        def _init_stats(self):
            self._stats = {'n_step': 0,
                           'wrong_pick': 0,
                           'wrong_place': 0,
                           'move_failure': 0}

        def reset_custom(self, same=False):
            """
            some custom fields to be reset
            """
            self._init_stats()
            self.n_satisfied = 0
            self.task_ptr = 0
            if not same:
                self._config_task(randomize=self.random_task)

        @property
        def task(self):
            return self._task

        def set_task(self, new_task):
            self.task_specs = new_task['goals']
            self.end_constraints = new_task['end_constraints']
            self._config_task(randomize=self.random_task)

        def _config_task(self, randomize=False):
            if self.task_specs is None:
                return

            self._task = []

            if randomize:
                task_order = np.random.permutation(
                    np.arange(len(self.task_specs)))
                curr_task_specs = [self.task_specs[t] for t in task_order]
            else:
                curr_task_specs = self.task_specs

            for ts in curr_task_specs:
                if ts.has_key("count"):
                    count = ts['count']
                else:
                    count = 9999
                if ts['name'] == 'pick_place':
                    for o in self.get_task_object(ts['src']).instances[:count]:
                        self._task.append({'src': ts['src'],
                                           'target': ts['target']})
                elif ts['name'] == 'pick_release':
                    for o in self.get_task_object(ts['src']).instances[:count]:
                        self._task.append({'src': ts['src'],
                                           'target': ts['target']})
                elif ts['name'] == 'pick_press':
                    for o in self.get_task_object(ts['src']).instances[:count]:
                        self._task.append({'src': ts['src'],
                                           'target': ts['target'],
                                           'target2': ts['target2'],
                                           'loc': ts['loc']
                                           })
                else:
                    raise NotImplementedError(
                        'Cannot handle %s task type' % ts['name'])

        def start_task(self):
            raise NotImplementedError

        def next_task(self):
            curr_task = None
            n_task_remain = 0

            if self.task_ptr < self.num_task:
                curr_task = self.task[self.task_ptr]
                n_task_remain = self.num_task - self.task_ptr
                self.task_ptr += 1

            return curr_task, n_task_remain

        @property
        def task_done(self):
            for c in self.end_constraints:
                s, ns = self.satisfied(c)
                if c.has_key('count'):
                    return ns == c['count']
                if not s:
                    return False
            return True

        @property
        def already_failed(self):
            return self.stats['move_failure'] or self.stats['wrong_pick'] or \
                self.stats['wrong_place']

        def check_n_satisfied(self):
            ns = 0
            for c in self.end_constraints:
                _, cns = self.satisfied(c)
                ns += cns
            if ns < self.n_satisfied and not self.already_failed:
                self.stats['move_failure'] = 1
            self.n_satisfied = ns
            return ns

        @property
        def num_task(self):
            return len(self.task)

        @property
        def stats(self):
            return self._stats

    class Stacking(TaskWorld):

        def start_task(self):
            self.lock_task_objects()

    class Sorting(TaskWorld):

        def start_task(self):
            self.lock_task_objects()
            ti = np.random.randint(0, 4)
            self.action_move_to('traybox_%i' % ti)

    factory_dict = {'sorting': Sorting,
                    'stacking': Stacking}

    return factory_dict[task_name]

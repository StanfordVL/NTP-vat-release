"""
Function that generates neural program traces
"""

from collections import defaultdict
from copy import deepcopy


class NPIView:

    def __init__(self):
        self._trace = {'in_prgs': 0,  # input program id
                       'in_args': 0,  # input argument
                       'out_prgs': 0,  # output program id
                       'out_args': 0,  # ouput argument
                       'out_stops': 0,  # if stop the program
                       'out_prg_mask': 1,  # if valid program output
                       'out_arg_mask': 1,  # if valid argument output
                       'out_stop_mask': 1,  # if valid stop output
                       'in_boundary_begin': 0,  # begin of sequence
                       'in_boundary_end': 0,  # end of sequence
                       'out_boundary_begin': 0,  # begin out output program
                       'out_boundary_end': 0,  # end of output program
                       'cmd_in_boundary_begin': 0,  # begin of sequence
                       'cmd_in_boundary_end': 0,  # end of sequence
                       'cmd_out_boundary_begin': 0,  # begin out output program
                       'cmd_out_boundary_end': 0,  # end of output program
                       'out_boundary_mask': 1,  # if learn boundary
                       'adaptive': 1,
                       'state_seq_ptr': None,  # which state sequence
                       'frame_ptr': None,  # frame index of current state
                       'psid': None,  # program sequence id
                       'caller_ptr': None,  # caller trace pointer
                       'callee_ptr': None
                       }

    def observe(self):
        raise NotImplementedError

    def command(self, words):
        raise NotImplementedError

    @property
    def entry_point(self):
        raise NotImplementedError

    @property
    def trace(self):
        return self._trace.copy()

    @property
    def current_frame(self):
        raise NotImplementedError

    @property
    def current_command(self):
        raise NotImplementedError

    @property
    def expert_programs(self):
        raise NotImplementedError

    def add_trace(self, trace):
        pname = self.program_names[trace['in_prgs']]
        psid = trace['psid']
        trace['frame_ptr'] = self.current_frame
        trace['out_boundary_begin'] = self.current_frame
        trace['cmd_out_boundary_begin'] = self.current_command

        """
        print('PRG: %s ARGS: %s OPRG: %s OARGS: %s' % (pname,
                                                       self.world.ind_to_name(trace['in_args']),
                                                       self.program_names[trace['out_prgs']],
                                                       self.world.ind_to_name(trace['out_args'])))
        print(self.state_log['agent_states'][self.current_frame])
        print(self.state_log['object_states'][self.current_frame])
        """

        self.push_trace(trace)

        # add to serialized trace log
        for k, v in trace.items():
            # initialization of the dictionry
            if k not in self.all_trace[pname]:
                self.all_trace[pname][k] = [[]]

            # new program call of this program
            if len(self.all_trace[pname][k]) < psid + 1:
                self.all_trace[pname][k].append([])

            self.all_trace[pname][k][psid].append(v)

        seq_ptr = len(self.all_trace[pname]['in_prgs'][psid]) - 1
        trace_ptr = (pname, psid, seq_ptr)

        # also point caller's trace to the callee
        if trace['caller_ptr']:
            caller_name, caller_psid, seq_i = trace['caller_ptr']
            self.all_trace[caller_name]['callee_ptr'][
                caller_psid][seq_i] = trace_ptr

        return (pname, psid, seq_ptr)

    def call_expert(self, caller_trace, pname, args=None, stop=False, command=None):
        """append the current trace and call the next program"""
        caller_trace = caller_trace.copy()
        caller_trace['out_prgs'] = self.program_to_ind[pname]
        caller_trace['out_args'] = args if args else 0
        caller_trace['out_arg_mask'] = int(args is not None)
        caller_trace['out_boundary_mask'] = int(
            pname in self.ADAPTIVE)  # if callee is adaptive
        caller_trace['out_stops'] = int(stop)

        # add to program trace
        caller_ptr = self.add_trace(caller_trace)

        # call expert program
        self.call_expert_helper(pname, args, caller_ptr, command)

        if stop:
            self.call_stop_helper(caller_trace['caller_ptr'], caller_ptr)

    def call_expert_helper(self, pname, args, caller_ptr=None, command=None):
        """format the trace for the next program and call the expert program"""
        # call the expert program
        callee_trace = self.trace
        callee_trace['caller_ptr'] = caller_ptr
        callee_trace['adaptive'] = int(pname in self.ADAPTIVE)
        callee_trace['in_prgs'] = self.program_to_ind[pname]
        callee_trace['in_args'] = args if args else 0
        callee_trace['in_boundary_begin'] = self.current_frame
        callee_trace['cmd_in_boundary_begin'] = self.current_command
        self.psid[pname] += 1
        callee_trace['psid'] = self.psid[pname]

        if command:
            self.command(command)
        self.expert_programs[pname](callee_trace)

    def call_stop(self, trace, args=None):
        """call stop and append the trace"""
        trace = trace.copy()
        trace['out_stops'] = 1
        trace['out_args'] = args if args else 0
        trace['out_arg_mask'] = int(args is not None)
        trace['out_prg_mask'] = 0  # dummy prg
        trace['out_boundary_mask'] = 0  # dummy boundary
        trace_ptr = self.add_trace(trace)
        return trace['caller_ptr'], trace_ptr

    def append_trace(self, caller_ptr, trace_ptr):
        depth_trace = self.call_stop_helper(caller_ptr, trace_ptr)
        self.depth_trace.extend(depth_trace)

    def call_stop_helper(self, caller_ptr, trace_ptr):
        """collect depth trace and add to log"""
        # leaf node
        depth_trace = self.pop_trace()

        # record end boundary of this program sequence
        pname, psid, _ = trace_ptr
        for i in xrange(len(self.all_trace[pname]['in_boundary_end'][psid])):
            self.all_trace[pname]['in_boundary_end'][
                psid][i] = self.current_frame + 1
            self.all_trace[pname]['cmd_in_boundary_end'][
                psid][i] = self.current_command

        # record boundary frame in caller's trace
        if caller_ptr is not None:
            caller_name, caller_psid, seq_i = caller_ptr
            self.all_trace[caller_name]['out_boundary_end'][
                caller_psid][seq_i] = self.current_frame + 1
            self.all_trace[caller_name]['cmd_out_boundary_end'][
                caller_psid][seq_i] = self.current_command
        return depth_trace

    def push_trace(self, trace):
        self.trace_stack[-1].append(trace)
        if trace['out_prg_mask']:
            self.trace_stack.append([])

    def pop_trace(self):
        curr_leaf = self.trace_stack.pop()
        for leaf_trace in curr_leaf:
            leaf_trace['in_boundary_end'] = self.current_frame + 1
            leaf_trace['cmd_in_boundary_end'] = self.current_command

        if self.program_names[curr_leaf[0]['in_prgs']] not in self.ACT:
            curr_leaf = [curr_leaf[-1]]

        # end the caller output boundary
        if self.trace_stack:
            self.trace_stack[-1][-1]['out_boundary_end'] = self.current_frame + 1
            self.trace_stack[-1][-1]['cmd_out_boundary_end'] = self.current_command

        depth_trace = []
        for leaf_trace in curr_leaf:
            # end the input boundary of this program
            root_to_leaf = []
            for trace_list in self.trace_stack:
                root_to_leaf.append(trace_list[-1])
            root_to_leaf.append(leaf_trace)
            depth_trace.append(root_to_leaf)

        return depth_trace

    def expert_program_trace(self):
        """
        entry point
        args:
            pname: entry program
            args: argument of entry program
        """
        pid, args = self.entry_point
        pname = self.program_names[pid]

        # initialization
        self.psid = {}
        self.all_trace = {}
        self.depth_trace = []
        self.trace_stack = [[]]
        self.success = True
        for n in self.program_names:
            self.psid[n] = -1
            self.all_trace[n] = defaultdict(list)

        self.state_log = deepcopy(self._state_log_tmp)
        self.observe()

        # call root program
        self.call_expert_helper(pname, args)
        if not self.success:
            print('demo failed')
            return None

        if self.isrobot:
            use = raw_input("Use Demo (yes/no)?")
            if use == "no":
                return None

        self.trace_sanity_check()
        serial_depth_trace = self.serialize_depth_trace()

        return self.all_trace, serial_depth_trace, self.state_log

    def serialize_depth_trace(self):
        serial_trace = {}
        for k in self.trace:
            serial_trace[k] = []

        for seq_trace in self.depth_trace:
            for k in serial_trace:
                serial_trace[k].append([])

            for trace in seq_trace:
                for k, v in trace.items():
                    serial_trace[k][-1].append(v)

        return serial_trace

    def trace_sanity_check(self):
        assert (not self.trace_stack)

        for p, pentry in self.all_trace.items():
            num_seq = len(pentry['in_prgs'])
            for i in xrange(num_seq):
                seq_len = len(pentry['in_prgs'][i])
                for j in xrange(seq_len):
                    caller_ptr = pentry['caller_ptr'][i][j]
                    if caller_ptr is None:
                        continue

                    caller_name, ci, cj = caller_ptr
                    assert(self.all_trace[caller_name]['out_boundary_begin'][ci][cj] ==
                           pentry['in_boundary_begin'][i][j])
                    assert(self.all_trace[caller_name]['out_boundary_end'][ci][cj] ==
                           pentry['in_boundary_end'][i][j])

    @property
    def program_to_ind(self):
        d = {}
        for i, name in enumerate(self.program_names):
            d[name] = i
        return d

    @property
    def program_names(self):
        return [p[0] for p in self.programs]

    @property
    def programs(self):
        raise NotImplementedError

    @property
    def ACT(self):
        raise NotImplementedError

    @property
    def ADAPTIVE(self):
        raise NotImplementedError

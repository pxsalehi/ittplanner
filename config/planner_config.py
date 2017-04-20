from optparse import OptionParser
from enum import Enum
import sys

__author__ = 'pxsalehi'


class Planner(Enum):
    GREEDY = 'greedy'
    BFS = 'bfs'
    IP = 'ip'


# search space for the IP-based solver
class IPSearchSpaceType(Enum):
    ALL = 'ALL'  # consider all actions
    ALL_PATHS = 'ALL_PATHS'  # consider only combination of nodes on all reconf path
    EACH_PATH = 'EACH_PATH'  # consider combination of each reconf path
    EACH_SUBTREE = 'EACH_SUBTREE'  # consider combination of each subtree


class CostModelType(Enum):
        FIXED = 0
        RANDOM = 1
        RUNTIME = 2  # calculate action cost for each step based on previous actions and performance metrics

plan_len_inc_default = 1
topo_max_child_default = 7  # default value
plan_len_start_default = 5
max_steps_default = 10
threads_default = 4
presolve_pass_default = -1
presolve_default = -1  # -1: automatic, 0: off, 1: conservative, 2: aggressive
search_space_default = IPSearchSpaceType.ALL.name
action_cost_default = CostModelType.FIXED.name
bfs_max_qsize_default = 10**5
stat_filename_default = './stats.txt'


class PlannerConfig():
    def __init__(self, planner, size=None, src=None, dest=None, change_rate=None, seed=None, threads=threads_default,
                 tune=False, max_steps=max_steps_default, prepasses=presolve_pass_default, presolve=presolve_default,
                 use_move=False, search_space=search_space_default, plan_len_start=plan_len_start_default,
                 plan_len_inc=plan_len_inc_default, action_cost=action_cost_default, write_model=False,
                 compute_iis=False, gen_props_from_actions=False, cost_file=None, stat_filename=stat_filename_default,
                 no_solve=False, time_limit=None, bfs_qsize=bfs_max_qsize_default):
        self.planner = Planner(planner.lower())
        if (size is None or size < 1) and src is None:
            raise RuntimeError('Please provide topology size or initial topology')
        if (change_rate is None or not (0 < change_rate <= 1)) and dest is None:
            raise RuntimeError('Please provide change rate or target topology')
        if seed is None:
            raise RuntimeError('Seed cannot be none!')
        self.size = src.size if src is not None else size
        self.src = src
        self.dest = dest
        self.change_rate = change_rate
        self.seed = seed
        self.threads = threads
        self.tune = tune
        self.max_steps = max_steps
        self.prepasses = prepasses
        self.presolve = presolve
        self.use_move = use_move
        self.search_space = search_space
        self.plan_len_start = plan_len_start
        self.plan_len_inc = plan_len_inc
        self.action_cost_model = action_cost
        self.write_model = write_model
        self.compute_iis = compute_iis
        self.gen_props_from_actions = gen_props_from_actions
        self.cost_file = cost_file
        self.stat_filename = stat_filename
        self.stat_file = None
        self.no_solve = no_solve
        self.time_limit = time_limit
        self.bfs_qsize = bfs_qsize


def parse_cl(arg_list=sys.argv[1:]):
    parser = create_parser()
    (options, args) = parser.parse_args(arg_list)
    if not options.planner:
        parser.error('Choose planner type: greedy, bfs, ip')
    else:
        try:
            options.planner = Planner(options.planner.lower())
        except ValueError as e:
            parser.error('Incorrect planner name! Choose planner type: greedy, bfs, ip')
    if not options.seed:
        parser.error('You forgot --seed!')
    if not options.src_topo and not options.size:
        parser.error('Please provide either --size or --src!')
    if not options.dest_topo and not options.change_rate:
        parser.error('Please provide either --change-rate or --dest!')
    if options.planner == Planner.BFS and options.time_limit is None:
        parser.error('Please provide --time-limit for BFS planner!')
    return options, args


def create_parser():
    parser = OptionParser()
    parser.add_option('--planner', type='string', dest='planner', help='planner type')
    parser.add_option('--size', action='store', type='int', dest='size',
                      help='number of brokers in the overlay')
    parser.add_option('--change-rate', action='store', type='float', dest='change_rate',
                      help='percentage of overlay links that change')
    parser.add_option('--src', type='string', dest='src_topo', default=None, help='source topology')
    parser.add_option('--dest', type='string', dest='dest_topo', default=None, help='target topology')
    parser.add_option('--seed', action='store', type='int', dest='seed',
                      help='seed for randomization')
    parser.add_option('--threads', action='store', type='int', dest='threads', default=threads_default,
                      help='number of threads to be used by the solver')
    parser.add_option('--tune', action='store_true', dest='tune', default=False,
                      help='whether to tune the solver for the problem first')
    parser.add_option('--max-steps', action='store', type='int', dest='max_steps', default=max_steps_default,
                      help='maximum number of plan steps')
    parser.add_option('--prepasses', type='int', dest='prepasses', default=presolve_pass_default,
                      help='Gurobi prepasses option')
    parser.add_option('--presolve', type='int', dest='presolve', default=presolve_default,
                      help='Gurobi presolve parameter')
    parser.add_option('--use-move', action='store_true', dest='use_move', default=False,
                      help='Whether to include move operations')
    parser.add_option('--topo-max-child', type='int', dest='topo_max_child', default=topo_max_child_default,
                      help='maximum broker children for generated topologies')
    parser.add_option('--search-space', type='string', dest='search_space', default=search_space_default,
                      help='type of search space: all, all_paths, each_path, each_subtree')
    parser.add_option('--plan-len-start', type='int', dest='plan_len_start', default=plan_len_start_default,
                      help='initial length of plan')
    parser.add_option('--plan-len-inc', type='int', dest='plan_len_inc', default=plan_len_inc_default,
                      help='increment plan length upon infeasible plan')
    parser.add_option('--action-cost', type='string', dest='action_cost_model', default=action_cost_default,
                      help='action cost model: fixed, random, runtime')
    parser.add_option('--write-model', action='store_true', dest='write_model', default=False,
                      help='dump generated IP model to file')
    parser.add_option('--compute-iis', action='store_true', dest='compute_iis', default=False,
                      help='compute infeasible set')
    parser.add_option('--gen-props-from-actions', action='store_true', dest='gen_props_from_actions', default=False)
    parser.add_option('--cost-file', type='string', dest='cost_file', default=None, help='action cost file')
    parser.add_option('--stat-file', type='string', dest='stat_filename', default=stat_filename_default,
                      help='output file for statistics')
    parser.add_option('--no-solve', action='store_true', dest='no_solve', default=False,
                      help='only generate the model, do not solve it!')
    parser.add_option('--time-limit', type='int', dest='time_limit', default=None)
    parser.add_option('--bfs-qsize', type='int', dest='bfs_qsize', default=bfs_max_qsize_default)
    parser.add_option('--output-dir', type='string', dest='output_dir', default=None,
                      help='output dir to write stats and plan')
    return parser

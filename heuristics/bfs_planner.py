from queue import PriorityQueue
import functools
import random
from timeit import default_timer as timer
from collections import OrderedDict

import networkx as nx
import sys

from topology.generator import change_links, generate_tree, convert_edges_to_networkx, read_topology_from_json
import config.planner_config as pconf


__author__ = 'pxsalehi'

TIME_LIMIT_SEC = 10*60
SIZE_LIMIT = 100000


class ClosedListElm:
    def __init__(self, step_count, hval, actions=[], closed=False):
        self.step_count = step_count
        self.hval = hval
        self.actions = actions
        self.closed = closed


@functools.total_ordering
class OpenListElm:
    def __init__(self, state, step_count, hval, actions=[]):
        self.state = state
        self.step_count = step_count
        self.hval = hval
        self.actions = actions

    def get_plan_estimate(self):
        return self.hval + self.step_count

    def __eq__(self, other):
        return self.step_count + self.hval == other.step_count + other.hval

    def __lt__(self, other):
        return self.step_count + self.hval < other.step_count + other.hval

    def __str__(self):
        return '{}, steps={}, hval={}, a={}'.format(self.state, self.step_count, self.hval, self.actions)


def create_hashable_state(edges):
    return frozenset([frozenset(e) for e in edges])


# returns calculated plan and the time it took to calculate the best plan
def calculate_plan(initial_topo, target_topo, options=None):
    TIME_LIMIT_SEC = options.time_limit
    SIZE_LIMIT = options.bfs_qsize
    s0 = initial_topo.edges
    s0_hval = estimate(s0, target_topo)
    closed_list = dict()
    open_list = PriorityQueue()
    best_score = float('inf')
    best_plan = None
    best_plan_time = None
    start_time = timer()
    while timer() - start_time < TIME_LIMIT_SEC:
        print('.', flush=True, end='')
        closed_list.clear()
        closed_list[create_hashable_state(s0)] = ClosedListElm(0, s0_hval)
        open_list.put(OpenListElm(s0, 0, s0_hval))
        last_state = None
        while not open_list.empty() and (timer() - start_time) < TIME_LIMIT_SEC:
            # print('.', end='', flush=True)
            # check size limit
            if len(closed_list) > SIZE_LIMIT:
                # print('closed list reached size limit', flush=True, end='')
                break
            next_state = get_next_state_to_search(open_list, closed_list, best_score)
            if next_state is not None:
                last_state = next_state
            if next_state is None or is_state_subset(target_topo.edges, next_state.state):
                # print('search reached an end state', flush=True, end='')
                break
            all_ops = set()
            for edge in next_state.state:
                if not is_edge_in(edge, target_topo.edges):
                    # all_ops.update(find_all_shifts(edge, next_state.state))
                    all_ops.update(find_all_shifts_smart(edge, next_state.state, target_topo))
            for op in all_ops:
                new_state = compute_state(next_state.state, op)
                hashable_new_state = create_hashable_state(new_state)
                previous = closed_list.get(hashable_new_state)
                if (previous is None or next_state.step_count + 1 < previous.step_count) and next_state.step_count + 1 < best_score:
                    hval = estimate(new_state, target_topo)
                    closed_list[hashable_new_state] = ClosedListElm(next_state.step_count + 1,
                                                                    hval, next_state.actions + [op], True)
                    open_list.put(OpenListElm(new_state, next_state.step_count + 1, hval, next_state.actions + [op]))
        if last_state is not None and is_state_subset(target_topo.edges, last_state.state):
            print('Found a plan with len', last_state.step_count, 'time left', TIME_LIMIT_SEC - (timer()-start_time), 'sec')
            if best_plan is None or len(last_state.actions) < len(best_plan):
                best_plan = last_state.actions
                best_plan_time = timer() - start_time
                print('New best plan length:', len(best_plan))
    end_time = timer()
    if best_plan:
        plan = dict()
        step = 1
        for op in best_plan:
            plan[step] = [op]
            step += 1
        best_plan = plan
    # TODO: convert plan from list of tuple to list of string
    return best_plan, best_plan_time


def estimate(topo_edges, target_topo):
    # find goal edges
    goal_edges = []
    for e in target_topo.edges:
        if not is_edge_in(e, topo_edges):
            goal_edges.append(e)
    rem_edges = [e for e in topo_edges if not target_topo.contains_edge(e)]
    topo_nx = convert_edges_to_networkx(topo_edges)
    # for each goal edge, find all removables located on the path
    total_ops = 0
    for g in goal_edges:
        path = nx.shortest_path(topo_nx, g[0], g[1])
        path_edges = {(path[i], path[i+1]) for i in range(0, len(path) - 1)}
        rems_on_path = {e for e in rem_edges if is_edge_in(e, path_edges)}
        # find the closest removable
        closest_rem = None
        min_ops = float('inf')
        for rem in rems_on_path:
            dist = estimate_distance(topo_nx, rem, g)
            if dist < min_ops:
                min_ops = dist
                closest_rem = rem
        if closest_rem is not None:
            total_ops += min_ops
            rem_edges.remove(closest_rem)
    return total_ops


# given a topology as a nx graph, estimates number of shifts it takes to remove rem and establish goal edge
def estimate_distance(topo_nx, rem, goal):
    path = nx.shortest_path(topo_nx, goal[0], goal[1])
    if len(path) <= 1:
        return 0
    return max(len(path) - 2, 1)


def compute_state(state, shift):
    new_state = set()
    for e in state:
        new_state.add(tuple(e))
    i, j, k = shift
    if (i, j) in new_state:
        new_state.remove((i, j))
    else:
        new_state.remove((j, i))
    new_state.add((i, k))
    return new_state


def get_next_state_to_search(open_list, closed_list, best_score):
    candidates = []
    while not open_list.empty():
        ol_elm = open_list.get()
        if len(candidates) > 0 and ol_elm.get_plan_estimate() != candidates[0].get_plan_estimate():
            open_list.put(ol_elm)
            break
        hashable_state = create_hashable_state(ol_elm.state)
        cl_elm = closed_list.get(hashable_state)
        if cl_elm is None:
            continue
        if (not cl_elm.closed or ol_elm.step_count <= cl_elm.step_count) and ol_elm.step_count < best_score:
            candidates.append(ol_elm);
    if len(candidates) > 0:
        # return a random state among equal ones
        return candidates[random.randint(0, len(candidates)-1)]
    return None


# given a removable edge and a topology, find set of all possible shifts
def find_all_shifts_smart(r, state, target_topo):
    goals = [e for e in target_topo.edges if not is_edge_in(e, state)]
    topo_nx = nx.Graph()
    topo_nx.add_edges_from(state)
    res = []
    # case 1: r[0]=i, r[1]=j
    i = r[0]
    j = r[1]
    # any neighbor of j except i is a potential k
    ks = []
    for e in topo_nx.edges():
        if e[0] == j and e[1] != i:
            ks.append(e[1])
        if e[1] == j and e[0] != i:
            ks.append(e[0])
    for g in goals:
        path_g = nx.shortest_path(topo_nx, g[0], g[1])
        for k in ks:
            if not topo_nx.has_edge(i, k) and topo_nx.has_edge(j, k):
                op = (i, j, k)
                # make sure this shift moves r towards g, which means, if applied, the shift decreases the length of path(g)
                new_topo = topo_nx.copy()
                new_topo.remove_edge(i, j)
                new_topo.add_edge(i, k)
                # if nx.is_connected(new_topo):
                new_path_g = nx.shortest_path(new_topo, g[0], g[1])
                if len(new_path_g) <= len(path_g):
                    res.append(op)
    # case 2: r[1]=i, r[0]=j
    i = r[1]
    j = r[0]
    # any neighbor of j except i is a potential k
    ks = []
    for e in topo_nx.edges():
        if e[0] == j and e[1] != i:
            ks.append(e[1])
        if e[1] == j and e[0] != i:
            ks.append(e[0])
    for g in goals:
        path_g = nx.shortest_path(topo_nx, g[0], g[1])
        for k in ks:
            if not topo_nx.has_edge(i, k) and topo_nx.has_edge(j, k):
                op = (i, j, k)
                # make sure this shift moves r towards g, which means, if applied, the shift decreases the length of path(g)
                new_topo = topo_nx.copy()
                new_topo.remove_edge(i, j)
                new_topo.add_edge(i, k)
                # if nx.is_connected(new_topo):
                new_path_g = nx.shortest_path(new_topo, g[0], g[1])
                if len(new_path_g) < len(path_g):
                    res.append(op)
    return set(res)


def is_state_subset(sub, super):
    for e in sub:
        if not is_edge_in(e, super):
            return False
    return True


def is_edge_in(edge, edge_set):
    return edge in edge_set or tuple(reversed(edge)) in edge_set


# given a removable edge and a topology, find set of all possible shifts
def find_all_shifts(r, state):
    res = []
    # case 1: r[0]=i, r[1]=j
    i = r[0]
    j = r[1]
    # any neighbor of j except i is a potential k
    ks = []
    for e in state:
        if e[0] == j and e[1] != i:
            ks.append(e[1])
        if e[1] == j and e[0] != i:
            ks.append(e[0])
    for k in ks:
        if not is_edge_in((i, k), state) and is_edge_in((j, k), state):
            res.append((i, j, k))
    # case 2: r[1]=i, r[0]=j
    i = r[1]
    j = r[0]
    # any neighbor of j except i is a potential k
    ks = []
    for e in state:
        if e[0] == j and e[1] != i:
            ks.append(e[1])
        if e[1] == j and e[0] != i:
            ks.append(e[0])
    for k in ks:
        if not is_edge_in((i, k), state) and is_edge_in((j, k), state):
            res.append((i, j, k))
    return res


def main():
    (options, args) = pconf.parse_cl()
    print('Running with', sorted(sys.argv[1:]))
    options.stat_file = open(options.stat_filename, 'a')
    random.seed(options.seed)
    # create initial topology
    if options.src_topo:
        init_topo = read_topology_from_json(options.src_topo)
        vertices = init_topo.v
    else:
        vertices = list(range(0, options.size))
        init_topo = generate_tree(size=len(vertices), max_child=options.topo_max_child, seed=options.seed)
    # create target topology
    if options.dest_topo:
        target_topo = read_topology_from_json(options.dest_topo)
    else:
        target_topo = change_links(init_topo, options.change_rate)
    # make sure init and target topo have the same broker set
    assert len(init_topo.vertices) == len(target_topo.vertices)
    # init_topo = Topology(6)
    # init_topo.set_edges([(0, 1), (0, 2), (1, 3), (3, 5), (2, 4)])
    # target_topo = Topology(6)
    # target_topo.set_edges([(0, 2), (1, 3), (3, 5), (2, 4), (4, 5)])
    print('init:', init_topo)
    print('target:', target_topo)
    plan, time = calculate_plan(init_topo, target_topo, options)
    print('best plan len:', 0 if plan is None else len(plan))
    print(plan)
    print('time:', time)


if __name__ == '__main__':
    main()

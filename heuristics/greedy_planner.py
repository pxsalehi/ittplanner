import copy
import random
from timeit import default_timer as timer

import networkx as nx

from topology.generator import convert_topo_to_networkx, generate_tree, change_links
from topology.reader import create_topology_from_file
import config.planner_config as pconf
from topology.topology import Topology


__author__ = 'pxsalehi'


# returns calculated plan and the time it took to calculate (ms)
def calculate_plan(initial_topo, target_topo, options=None):
    random.seed()
    planning_start = timer()
    # find set of goal edges
    init_topo = copy.deepcopy(initial_topo)
    plan = dict()
    plan_ops = set()
    step = 1
    while not init_topo.equals(target_topo):
        goal_edges = {edge for edge in target_topo.edges if edge not in init_topo.edges}
        init_topo_nx = convert_topo_to_networkx(init_topo)
        rem_edges = set()
        for goal_edge in goal_edges:
            path = nx.shortest_path(init_topo_nx, goal_edge[0], goal_edge[1])
            # add removable edges on the path to the set
            path_edges = {(path[i], path[i+1]) for i in range(0, len(path) - 1)}
            rem_edges.update({edge for edge in path_edges if not target_topo.contains_edge(edge)})
        candidate_ops = set()
        for g in goal_edges:
            for r in rem_edges:
                candidate_ops.update(find_all_shifts(r, g, init_topo_nx, check_effect_on_goal=True))
        candidate_ops.difference_update(plan_ops)
        if len(candidate_ops) == 0:
            print("Error: cannot find a plan!")
            return None, None
        op = list(candidate_ops)[random.randint(0, len(candidate_ops)-1)]
        # apply shift(i, j, k) to init_topo, remove (i, j) and add (i, k)
        init_topo.rem_edge(op[0], op[1])
        init_topo.add_edge(op[0], op[2])
        print('step', step, op, flush=True)
        plan[step] = op
        plan_ops.add(op)
        step += 1
    planning_end = timer()
    return plan, planning_end - planning_start


# given a removable edge and a topology, find set of all possible shifts
def find_all_shifts(r, g, topo_nx, check_effect_on_goal=False):
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
    path_g = nx.shortest_path(topo_nx, g[0], g[1])
    for k in ks:
        if not topo_nx.has_edge(i, k) and topo_nx.has_edge(j, k):
            op = (i, j, k)
            if check_effect_on_goal:
                # make sure this shift moves r towards g, which means, if applied, the shift decreases the length of path(g)
                new_topo = topo_nx.copy()
                new_topo.remove_edge(i, j)
                new_topo.add_edge(i, k)
                if nx.is_connected(new_topo):
                    new_path_g = nx.shortest_path(new_topo, g[0], g[1])
                    if len(new_path_g) <= len(path_g):
                        res.append(op)
            else:
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
    path_g = nx.shortest_path(topo_nx, g[0], g[1])
    for k in ks:
        if not topo_nx.has_edge(i, k) and topo_nx.has_edge(j, k):
            op = (i, j, k)
            if check_effect_on_goal:
                # make sure this shift moves r towards g, which means, if applied, the shift decreases the length of path(g)
                new_topo = topo_nx.copy()
                new_topo.remove_edge(i, j)
                new_topo.add_edge(i, k)
                if nx.is_connected(new_topo):
                    new_path_g = nx.shortest_path(new_topo, g[0], g[1])
                    if len(new_path_g) <= len(path_g):
                        res.append(op)
            else:
                res.append(op)
    return set(res)


def main():
    (options, args) = pconf.parse_cl()
    options.stat_file = open(options.stat_filename, 'a')
    random.seed(options.seed)
    # create initial topology
    if options.src_topo:
        init_topo = create_topology_from_file(options.src_topo)
        vertices = init_topo.v
    else:
        vertices = list(range(0, options.size))
        init_topo = generate_tree(size=len(vertices), max_child=options.topo_max_child, seed=options.seed)
    # create target topology
    if options.dest_topo:
        target_topo = create_topology_from_file(options.dest_topo)
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
    print(plan)
    print('time:', time)


if __name__ == '__main__':
    main()
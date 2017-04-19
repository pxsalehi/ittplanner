from collections import OrderedDict
import os
import random

import jsonpickle

from heuristics import bfs_planner
import config.planner_config as pconf
from topology.generator import generate_tree, read_topology_from_json, change_links
from topology.topology import Topology


__author__ = 'pxsalehi'


def write_results(out_dir, plan, planning_time, seed):
    if not planning_time:
        planning_time = float('nan')
    plan_steps, plan_ops = plan_stat(plan)
    with open(os.path.join(out_dir, 'plan_stat' + str(seed) + '.txt'), 'w') as out:
        out.write('plan_time:{} \n'.format(planning_time))
        out.write('plan_steps:{} \n'.format(plan_steps))
        out.write('plan_ops:{} \n'.format(plan_ops))
    # write plan to separate file
    with open(os.path.join(out_dir, 'plan' + str(seed) + '.txt'), 'w') as out:
        if plan:
            ordered_plan = OrderedDict(sorted(plan.items()))
            out.write(jsonpickle.encode(ordered_plan))


def plan_stat(plan):
    plan_steps = plan_ops = 0
    if plan is None:
        plan_steps = plan_ops = float('nan')
    else:
        for step in plan.values():
            if len(step) > 0:
                plan_steps += 1
            plan_ops += len(step)
    return plan_steps, plan_ops


def main():
    (options, args) = pconf.parse_cl()
    print('Running with', sorted(options.__dict__.items()))
    options.stat_file = open(options.stat_filename, 'a')
    random.seed(options.seed)
    # create initial topology
    if options.src_topo:
        init_topo = read_topology_from_json(options.src_topo)
        vertices = init_topo.vertices
    else:
        vertices = list(range(0, options.size))
        init_topo = generate_tree(size=len(vertices), max_child=options.topo_max_child, seed=options.seed)
    # create target topology
    if options.dest_topo:
        target_topo = read_topology_from_json(options.dest_topo)
    else:
        target_topo = change_links(init_topo, options.change_rate, options.seed)
    # make sure init and target topo have the same broker set
    assert len(init_topo.vertices) == len(target_topo.vertices)
    if options.planner == pconf.Planner.IP:
        import ILP_optiplan.itt_optiplan as optiplan
        planner = optiplan.calculate_plan
    elif options.planner == pconf.Planner.BFS:
        planner = bfs_planner.calculate_plan
    else:
        raise RuntimeError("Only ip and bfs planner are supported!")
    print('init:', init_topo)
    print('target:', target_topo)
    plan, time = planner(init_topo, target_topo, options)
    if options.output_dir:
        write_results(options.output_dir, plan, time, options.seed)
    else:
        plan_steps, plan_ops = plan_stat(plan)
        print('best plan steps: {}'.format(plan_steps))
        print('best plan ops: {}'.format(plan_ops))
        print('plan', plan)
        print('time:', time)


if __name__ == '__main__':
    main()


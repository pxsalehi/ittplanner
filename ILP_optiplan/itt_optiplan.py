#!/usr/bin/env python3
import copy

from gurobipy import *
from datetime import datetime
from ILP_optiplan.itt_operator import *
import os
import sys
from timeit import default_timer as timer
import re
import config.planner_config as pconf
from ILP_optiplan.ilp_model import create_model
from ILP_optiplan.cost_model import *
from ILP_optiplan.runtime_params import *
from topology.generator import convert_topo_to_networkx, generate_tree, change_links
import networkx as nx
from topology.reader import create_topology_from_file
from topology.topology import Topology

__author__ = 'pxsalehi'

deployer = None


def extract_sort_actions(action_vars, plan_steps):
    actions = {}
    for t in plan_steps:
        actions[t] = list()
    for a in action_vars.values():
        if a.x > 0:
            toks = a.getAttr('VarName').split('_')
            step = int(toks[2])
            op = toks[1]
            actions[step].append(op)
    return actions


def remove_empty_steps(actions):
    step = 1;
    trimmed = dict()
    for s in actions.keys():
        if actions[s]:  # not empty
            trimmed[step] = actions[s]
            step += 1
    return trimmed


def gen_plan_lens(plan_len_start, plan_len_increment, max_len):
    return list(range(plan_len_start, max_len + 1, plan_len_increment))


def generate_propositions(actions, options, vertices):
    all_props = set()
    if options.gen_props_from_actions:
        for a in actions:
            all_props.update(a.get_pre())
            all_props.update(a.get_add())
            all_props.update(a.get_del())
    else:
        # create all propositions
        conn_perm = itertools.permutations(vertices, 2)
        for c in conn_perm:
            all_props.add(Conn(c[0], c[1]))
        rem_perm = itertools.permutations(vertices, 2)
        for r in rem_perm:
            all_props.add(Rem(r[0], r[1]))
    return all_props


def gen_shift_acts(initial_topo, target_topo, options):
    search_space_type = pconf.IPSearchSpaceType(options.search_space.upper())
    if search_space_type == pconf.IPSearchSpaceType.ALL:
        shift_perm = itertools.permutations(initial_topo.vertices, 3)
        shift_actions = [Operator.Shift(target_topo, sh[0], sh[1], sh[2]) for sh in shift_perm]
    elif search_space_type == pconf.IPSearchSpaceType.ALL_PATHS:
        new_edges = {e for e in target_topo.edges if e not in initial_topo.edges}
        initial_topo_nx = convert_topo_to_networkx(initial_topo)
        reconf_nodes = set()
        for e in new_edges:
            reconf_nodes.update(nx.shortest_path(initial_topo_nx, e[0], e[1]))
        print('**** reconf nodes size: %d' % len(reconf_nodes))
        shift_perm = itertools.permutations(reconf_nodes, 3)
        shift_actions = [Operator.Shift(target_topo, sh[0], sh[1], sh[2]) for sh in shift_perm]
    elif search_space_type == pconf.IPSearchSpaceType.EACH_PATH:
        new_edges = {e for e in target_topo.edges if e not in initial_topo.edges}
        initial_topo_nx = convert_topo_to_networkx(initial_topo)
        shift_actions = []
        for e in new_edges:
            reconf_nodes = nx.shortest_path(initial_topo_nx, e[0], e[1])
            shift_perm = itertools.permutations(reconf_nodes, 3)
            shift_actions.extend([Operator.Shift(target_topo, sh[0], sh[1], sh[2]) for sh in shift_perm])
    # elif search_space_type == heuristic:
    else:
        raise RuntimeError('Invalid local-search value %d' % search_space_type)
    return shift_actions


def parse_action(act_str):
    nodes = [int(s) for s in re.findall(r"[\w']+", act_str) if s.isdigit()]
    return Operator.Shift()


# returns a list of where list[i] is the set of actions in step i
def extract_move_ops(init_props, actions, target_topo, cost_model):
    max_move_cost = 100
    result_acts = list()
    # create list of shifts per step
    for alist in actions.values():
        step_actions = list()
        for act_str in alist:
            nodes = [int(s) for s in re.findall(r"[\w']+", act_str) if s.isdigit()]
            step_actions.append(Operator.Shift(target_topo, nodes[0], nodes[1], nodes[2]))
        result_acts.append(step_actions)
    # extract moves
    actions_per_step = copy.deepcopy(result_acts)
    moves_per_step = list()
    for step in range(len(actions_per_step) - 1):  # from first step to one before the end
        step_moves = list()
        for action in actions_per_step[step]:  # take one action in that step and find ones that follow it
            move = [action]
            move_cost = cost_model[action.tuple, step + 1]
            rem_produced = (action.vj, action.vk)
            try:
                for next_step in range(step + 1, len(actions_per_step)):
                # for step_actions in actions_per_step[step + 1:]:
                    step_actions = actions_per_step[next_step]
                    # find a dependant action
                    for act in step_actions:
                        rem_needed = (act.vi, act.vj)
                        if (rem_needed[0] == rem_produced[0] and rem_needed[1] == rem_produced[1]) or \
                           (rem_needed[0] == rem_produced[1] and rem_needed[1] == rem_produced[0]):
                            if move_cost + cost_model[act.tuple, next_step + 1] <= max_move_cost:
                                move_cost += cost_model[act.tuple, next_step + 1]
                                move.append(act)
                                rem_produced = (act.vj, act.vk)
                                step_actions.remove(act)
                            else:
                                # reached move cost upper bound
                                raise StopIteration
            except StopIteration:
                pass
            step_moves.append((move, move_cost))
            # print('move:', [str(a) for a in move], 'cost:', str(move_cost))
        if len(step_moves) > 0:
            moves_per_step.append(step_moves)
    last_step_moves = list()
    for act in actions_per_step[-1]:
        # print('move: ', str(act))
        last_step_moves.append(([act], cost_model[act.tuple, len(actions_per_step)]))
    if len(last_step_moves) > 0:
        moves_per_step.append(last_step_moves)
    print('Final actions:')
    for step, move_step in enumerate(moves_per_step):
        print('step %d:' % step, end='')
        for move, cost in move_step:
            print('M(%s)(%d)  ' % ([str(a) for a in move], cost), end='')
        print()
    return result_acts

# m.setParam("TimeLimit", 100.0); and kwargs can make adding new params a breeze!
def set_solver_params(model, options):
    model.params.threads = options.threads
    model.params.prepasses = options.prepasses
    model.params.presolve = options.presolve
    model.params.mipgap = 0.1
    if options.time_limit:
        model.params.timelimit = options.time_limit

def tune_model(model, options):
    tune_file = './itt_%d_%.2f.prm' % (options.size, options.change_rate)
    if os.path.exists(tune_file):
        # read it back
        print('   reading tuning results from', tune_file)
        model.read(tune_file)
    else:
        print('   No tune file found (%s). Tuning...' % tune_file)
        model.tune()
        if model.tuneResultCount > 0:
            model.getTuneResult(0)
            print('   writing tune results to %s' % tune_file)
            model.write(tune_file)
        else:
            print('   No tune result found! Writing empty tune result to %s' % tune_file)
            model.write(tune_file)


# returns model and actions (can be None)
def plan_itt(initial_topo, target_topo, runtime_samples, plan_len, options):
    start_model = timer()
    # create shift actions
    shift_acts = gen_shift_acts(initial_topo, target_topo, options)
    print('number of shift actions:', len(shift_acts))
    all_acts = shift_acts
    # generate propositions
    all_props = generate_propositions(all_acts, options, initial_topo.vertices)
    # print('Number of conns:', len([f for f in all_props if isinstance(f, Conn)]))
    # assign cost to each action
    # cost_model = create_cost_model(all_acts, list(range(plan_len + 1)), options)
    init_props, model, action_vars = \
        create_model(initial_topo, target_topo, runtime_samples, plan_len, all_props, all_acts, options)
    # model.setParam('TimeLimit', 2*60)
    end_model = timer()
    plan_steps = list(range(0, plan_len + 1))
    vars = model.getVars()
    print('   number of variables: %d' % len(vars))
    constrs = model.getConstrs()
    print('   number of constraints: %d' % len(constrs))
    if options.no_solve:
        sys.exit()
    set_solver_params(model, options)
    if options.tune:
        tune_model(model, options)
    start_solve = timer()
    model.optimize()
    end_solve = timer()
    actions = None
    # check results
    if model.status == GRB.OPTIMAL:
        actions = extract_sort_actions(action_vars, plan_steps[1:])
        print('Calculated shift actions:')
        for t in actions.keys():
            print('%d: %s' % (t, str(actions[t])))
        end = timer()
        rem_edges = [e for e in initial_topo.edges if e not in target_topo.edges]
        goal_edges = [e for e in target_topo.edges if e not in initial_topo.edges]
        print('* removable edges:', rem_edges)
        print('* goal edges:', goal_edges)
        print('*** total time =', end - start_model)
        print('*** time to create feasible model =', (end_model - start_model))
        print('*** time to solve model =', (end_solve - start_solve))
    else:
        print('Model not feasible.')
    return model, actions


# returns calculated plan and the time it took to calculate (ms)
def calculate_plan(initial_topo, target_topo, options):
    # make sure init and target topo have the same broker set
    assert len(initial_topo.vertices) == len(target_topo.vertices)
    options.size = len(initial_topo.vertices)
    # compute change rate
    u = initial_topo.edges.intersection(target_topo.edges)
    eff_change_rate = 1 - (float(len(u))/len(initial_topo.edges))
    print('effective change rate =', str(eff_change_rate))
    # create a dictionary for runtime parameters
    runtim_samples = dict()
    if options.action_cost_model.upper() == CostModelType.RUNTIME.name:
        print('Using %s action cost model' % CostModelType.RUNTIME)
        runtim_samples = get_samples(initial_topo.v)
    for plan_len in gen_plan_lens(options.plan_len_start, options.plan_len_inc, options.max_steps):
        planning_start_time = timer()
        model, actions = plan_itt(initial_topo, target_topo, runtim_samples, plan_len, options)
        planning_end_time = timer()
        if model.status == GRB.OPTIMAL:
            assert actions is not None
            if options.write_model:
                model.write('itt_%d_%.2f_%d.lp' % (options.size, eff_change_rate, options.seed))
            return actions, planning_end_time - planning_start_time
        elif model.status == GRB.INFEASIBLE:
            print('Increasing steps to %d' % (plan_len + options.plan_len_inc))
        elif model.status == GRB.TIME_LIMIT or model.status == GRB.INTERRUPTED:
            print('model status:', model.status)
            break;
    # after trying with up to max_steps length, report as infeasible
    if model.status == GRB.INFEASIBLE:
        print('Infeasible model with size=%d change_rate=%.2f max_steps=%d seed=%d'
              % (options.size, options.change_rate, options.max_steps, options.seed))
        if options.compute_iis:
            model.computeIIS()
            model.write('itt_iis_%d_%.2f_%d.ilp' % (options.size, options.change_rate, options.seed))
    return None, None


def main():
    (options, args) = pconf.parse_cl()
    print('Running with', sorted(sys.argv[1:]))
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
    # example
    init_topo = Topology(5)
    init_topo.set_edges([(0, 1),(1,2),(1,3),(2,4)])
    target_topo = Topology(5)
    target_topo.set_edges([(0, 1), (1,2),(1,3),(3,4)])
    # make sure init and target topo have the same broker set
    assert len(init_topo.vertices) == len(target_topo.vertices)
    print('init:', init_topo)
    print('target:', target_topo)
    plan_actions, calc_len = calculate_plan(init_topo, target_topo, options)
    if plan_actions is None:
        print('Planner could not find a plan!')
    else:
        print(plan_actions)

if __name__ == '__main__':
    main()




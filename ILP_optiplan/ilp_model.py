from gurobipy import *
from timeit import default_timer as timer
import itertools
from ILP_optiplan.itt_proposition import *
from ILP_optiplan.cost_model import *

__author__ = 'pxsalehi'


def create_model(initial_topo, target_topo, runtime_samples, plan_len, all_props, all_acts, options):
    create_model_start = timer()
    assert plan_len > 0
    vertices = initial_topo.vertices
    # decision variables
    action_vars = {}
    state_vars_maintain = {}
    state_vars_preadd = {}
    state_vars_predel = {}
    state_vars_add = {}
    state_vars_del = {}
    # 0, 1, 2, ..., plan_len
    plan_steps = list(range(0, plan_len + 1))
    # create set of initial and target propositions
    init_f = set()
    for e in initial_topo.edges:
        init_f.add(Conn(e[0], e[1]))
        if e not in target_topo.edges:
            init_f.add(Rem(e[0], e[1]))
    target_f = set()
    for e in target_topo.edges:
        target_f.add(Conn(e[0], e[1]))
    # update set of props with init and target props
    all_props.update(init_f)
    all_props.update(target_f)
    print('number of props:', len(all_props))
    # sets pre_f, add_f, del_f: f -> set of actions
    gen_basic_sets_start = timer()
    pre_f = {f.tuple: set() for f in all_props}
    add_f = {f.tuple: set() for f in all_props}
    del_f = {f.tuple: set() for f in all_props}
    for a in all_acts:
        for f in a.get_pre():
            pre_f[f.tuple].add(a)
        for f in a.get_add():
            add_f[f.tuple].add(a)
        for f in a.get_del():
            del_f[f.tuple].add(a)
    gen_basic_sets_end = timer()
    print('--- time to generate basic sets:', gen_basic_sets_end - gen_basic_sets_start)
    gen_subt_sets_start = timer()
     # sets add_f_subt_pre_f, del_f_subt_pre_f, pre_f_subt_del_f
    add_f_subt_pre_f = {f.tuple: set([a for a in add_f[f.tuple] if a not in pre_f[f.tuple]]) for f in all_props}
    del_f_subt_pre_f = {f.tuple: set([a for a in del_f[f.tuple] if a not in pre_f[f.tuple]]) for f in all_props}
    pre_f_subt_del_f = {f.tuple: set([a for a in pre_f[f.tuple] if a not in del_f[f.tuple]]) for f in all_props}
    gen_subt_sets_end = timer()
    print('--- time to generate subt sets:', gen_subt_sets_end - gen_subt_sets_start)
    # print del_f / pre_f
    for f in all_props:
        actions = del_f_subt_pre_f[f.tuple]
        if actions:
            print(f.tuple, ':', end='')
            print('%s' % [str(a) for a in actions])
    create_vars_start = timer()
    # create model
    m = Model('itt')
    cost_model = ActionCostModel(m, options.action_cost_model)
    # create variables
    for a in all_acts:
        for t in plan_steps[1:]:
            action_vars[a.tuple, t] = m.addVar(vtype=GRB.BINARY, name='x_%s_%d' % (str(a), t))
    for f in all_props:
        for t in plan_steps:
            state_vars_maintain[f.tuple, t] = m.addVar(vtype=GRB.BINARY)
            state_vars_preadd[f.tuple, t] = m.addVar(vtype=GRB.BINARY)
            state_vars_predel[f.tuple, t] = m.addVar(vtype=GRB.BINARY)
            state_vars_add[f.tuple, t] = m.addVar(vtype=GRB.BINARY)
            state_vars_del[f.tuple, t] = m.addVar(vtype=GRB.BINARY)
    # create cost model variables, if any
    cost_model.add_cost_decision_variables(all_props, plan_steps)
    create_vars_end = timer()
    update_vars_start = timer()
    m.update()
    update_vars_end = timer()
    # generate constraints defining cost model, if any
    cost_model.add_cost_constrs(all_acts, action_vars, all_props, plan_steps, runtime_samples)
    # create objective
    # get cost per action
    cost_per_action = cost_model.get_cost_per_action(all_acts, action_vars, plan_steps)
    objLinExpr = LinExpr()
    for a in all_acts:
        for t in plan_steps[1:]:
            objLinExpr += cost_per_action[a.tuple, t] * action_vars[a.tuple, t]
    m.setObjective(objLinExpr, GRB.MINIMIZE)
    #m.setObjective(quicksum(action_vars.values()), GRB.MINIMIZE)
    # create constraints, separate loops are used to make IIS output more readable
    create_constrs_start = timer()
    constr_start = timer()
    for f in init_f:
        m.addConstr(state_vars_add[f.tuple, 0], GRB.EQUAL, 1)
    constr_end = timer()
    print('--- time to create constr 1:', constr_end - constr_start)
    constr_start = timer()
    for f in all_props:
        if f not in init_f:
            c = m.addConstr(
                state_vars_add[f.tuple, 0] + state_vars_maintain[f.tuple, 0] + state_vars_preadd[f.tuple, 0],
                GRB.EQUAL, 0)
    constr_end = timer()
    print('--- time to create constr 2:', constr_end - constr_start)
    constr_start = timer()
    for f in target_f:
        m.addConstr(state_vars_add[f.tuple, plan_len] + state_vars_maintain[f.tuple, plan_len]
                    + state_vars_preadd[f.tuple, plan_len],
                    GRB.GREATER_EQUAL, 1)
    constr_end = timer()
    print('--- time to create constr 3:', constr_end - constr_start)
    constr_start = timer()
    for f in all_props:
        for t in plan_steps[1:]:
            m.addConstr(quicksum([action_vars[a.tuple, t] for a in add_f_subt_pre_f.get(f.tuple, [])]),
                        GRB.GREATER_EQUAL, state_vars_add[f.tuple, t])
    constr_end = timer()
    print('--- time to create constr 4:', constr_end - constr_start)
    gen_constr_start = timer()
    constr_5_list = list()
    for f in all_props:
        for t in plan_steps[1:]:
            for a in add_f_subt_pre_f[f.tuple]:
                constr_5_list.append(action_vars[a.tuple, t] <= state_vars_add[f.tuple, t])
                # m.addConstr(action_vars[a.tuple, t] <= state_vars_add[f.tuple, t])
    gen_constr_end = timer()
    add_constr_start = timer()
    for tconstr in constr_5_list:
        m.addConstr(tconstr)
    add_constr_end = timer()
    print('--- time to gen / add constr 5:', gen_constr_end - gen_constr_start, add_constr_end - add_constr_start)
    # print('--- time to create constr 5:', constr_end - constr_start)
    constr_start = timer()
    for f in all_props:
        for t in plan_steps[1:]:
            m.addConstr(quicksum([action_vars[a.tuple, t] for a in del_f_subt_pre_f[f.tuple]]),
                        GRB.GREATER_EQUAL, state_vars_del[f.tuple, t])
    constr_end = timer()
    print('--- time to create constr 6:', constr_end - constr_start)
    constr_start = timer()
    for f in all_props:
        for t in plan_steps[1:]:
            for a in del_f_subt_pre_f[f.tuple]:
                m.addConstr(action_vars[a.tuple, t] <= state_vars_del[f.tuple, t])
    constr_end = timer()
    print('--- time to create constr 7:', constr_end - constr_start)
    constr_start = timer()
    for f in all_props:
        for t in plan_steps[1:]:
            m.addConstr(quicksum([action_vars[a.tuple, t] for a in pre_f_subt_del_f[f.tuple]]),
                        GRB.GREATER_EQUAL, state_vars_preadd[f.tuple, t])
    constr_end = timer()
    print('--- time to create constr 8:', constr_end - constr_start)
    constr_9_list = list()
    gen_constr_start = timer()
    for f in all_props:
        for t in plan_steps[1:]:
            for a in pre_f_subt_del_f[f.tuple]:
                constr_9_list.append(action_vars[a.tuple, t] <= state_vars_preadd[f.tuple, t])
                # m.addConstr(action_vars[a.tuple, t] <= state_vars_preadd[f.tuple, t])
    gen_constr_end = timer()
    add_constr_start = timer()
    for tconstr in constr_9_list:
        m.addConstr(tconstr)
    add_constr_end = timer()
    print('--- time to gen constr 9:', gen_constr_end - gen_constr_start)
    print('--- time to add constr 9:', add_constr_end - add_constr_start)
    # print('--- time to create constr 9:', constr_end - constr_start)
    constr_start = timer()
    for f in all_props:
        for t in plan_steps[1:]:
            m.addConstr(quicksum([action_vars[a.tuple, t] for a in pre_f[f.tuple] if a in del_f[f.tuple]]),
                        GRB.EQUAL, state_vars_predel[f.tuple, t])
    constr_end = timer()
    print('--- time to create constr 10:', constr_end - constr_start)
    constr_start = timer()
    for f in all_props:
        for t in plan_steps:
            m.addConstr(state_vars_add[f.tuple, t] + state_vars_maintain[f.tuple, t] + state_vars_del[f.tuple, t]
                        + state_vars_predel[f.tuple, t] <= 1)
    constr_end = timer()
    print('--- time to create constr 11:', constr_end - constr_start)
    constr_start = timer()
    for f in all_props:
        for t in plan_steps[1:]:
            m.addConstr(state_vars_preadd[f.tuple, t] + state_vars_maintain[f.tuple, t] + state_vars_del[f.tuple, t]
                        + state_vars_predel[f.tuple, t] <= 1)
    constr_end = timer()
    print('--- time to create constr 12:', constr_end - constr_start)
    gen_constr_start = timer()
    constr13_list = list()
    for f in all_props:
        for t in plan_steps[1:]:
            constr13_list.append(state_vars_preadd[f.tuple, t] + state_vars_maintain[f.tuple, t] +
                                 state_vars_predel[f.tuple, t] - state_vars_preadd[f.tuple, t - 1] -
                                 state_vars_add[f.tuple, t - 1] - state_vars_maintain[f.tuple, t - 1] <= 0)
            # m.addConstr(state_vars_preadd[f.tuple, t] + state_vars_maintain[f.tuple, t]
            #             + state_vars_predel[f.tuple, t] - state_vars_preadd[f.tuple, t - 1]
            #             - state_vars_add[f.tuple, t - 1] - state_vars_maintain[f.tuple, t - 1] <= 0)
    gen_constr_end = timer()
    add_constr_start = timer()
    for tconstr in constr13_list:
        m.addConstr(tconstr)
    add_constr_end = timer()
    print('--- time to gen constr 13:', gen_constr_end - gen_constr_start)
    print('--- time to add constr 13:', add_constr_end - add_constr_start)
    # print('--- time to create constr 13:', constr_end - constr_start)
    create_constrs_end = timer()
    update_constrs_start = timer()
    m.update()
    update_constrs_end = timer()
    create_model_end = timer()
    print('--- time to create vars:', create_vars_end - create_vars_start)
    print('--- time to update vars:', update_vars_end - update_vars_start)
    print('--- time to create constrs:', create_constrs_end - create_constrs_start)
    print('--- time to update constrs:', update_constrs_end - update_constrs_start)
    print('--- total time to create model:', create_model_end - create_model_start)
    return init_f, m, action_vars



import random

from ILP_optiplan.itt_proposition import *
from ILP_optiplan.runtime_params import RuntimeParam
from config.planner_config import CostModelType


__author__ = 'pxsalehi'


class ActionCostModel():
    def __init__(self, model, cost_model_type):
        self.m = model
        self.cost_model_type = CostModelType[cost_model_type.upper()]
        self.cost_per_action = dict()  # act.tuple, step -> action cost
        # only for RUNTIME cost model
        if self.cost_model_type == CostModelType.RUNTIME:
            # runtime_param, tuple, step ->  model decision var
            # e.g. MSG_RATE, i, j, 3
            self.cost_vars = dict()

    def add_cost_decision_variables(self, all_props, plan_steps):
        if self.cost_model_type != CostModelType.RUNTIME:
            return
        for conn in all_props:
            if isinstance(conn, Conn):
                for t in plan_steps:
                    self.cost_vars[RuntimeParam.MSG_RATE, conn.vi, conn.vj, t] = \
                        self.m.addVar(vtype=GRB.INTEGER, lb=0,name='flow_%d_%d_%d' % (conn.vi, conn.vj, t))
                    self.cost_vars[RuntimeParam.MSG_RATE, conn.vj, conn.vi, t] = \
                        self.m.addVar(vtype=GRB.INTEGER, lb=0, name='flow_%d_%d_%d' % (conn.vj, conn.vi, t))

    def add_cost_constrs(self, all_acts, action_vars, all_props, plan_steps, runtime_samples):
        if self.cost_model_type != CostModelType.RUNTIME:
            return
        for conn in all_props:
            if isinstance(conn, Conn):
                x = conn.vi
                y = conn.vj
                # initial value for msg rates
                self.m.addConstr(self.cost_vars[RuntimeParam.MSG_RATE, x, y, 0] ==
                                 runtime_samples[RuntimeParam.MSG_RATE, x, y])
                for t in plan_steps[1:]:
                    self.m.addQConstr(self.cost_vars[RuntimeParam.MSG_RATE, x, y, t], GRB.GREATER_EQUAL,
                        self.cost_vars[RuntimeParam.MSG_RATE, x, y, t - 1]
                        + quicksum([action_vars[a.tuple, t] * self.cost_vars[RuntimeParam.MSG_RATE, x, a.vj, t - 1]
                                  for a in all_acts if a.vi == x and a.vk == y])
                        + self.cost_vars[RuntimeParam.MSG_RATE, x, y, t - 1]
                        - quicksum([action_vars[a.tuple, t] * self.cost_vars[RuntimeParam.MSG_RATE, x, a.vj, t - 1]
                                    for a in all_acts if a.vi == y and a.vk == x])
                        + quicksum([action_vars[a.tuple, t] * self.cost_vars[RuntimeParam.MSG_RATE, x, a.vk, t - 1]
                                    for a in all_acts if a.vj == x and a.vk == y])
                        + quicksum([action_vars[a.tuple, t] * self.cost_vars[RuntimeParam.MSG_RATE, x, y, t - 1]
                                    for a in all_acts if a.vj == y and a.vk == x])
                        )
                y = conn.vi
                x = conn.vj
                self.m.addConstr(self.cost_vars[RuntimeParam.MSG_RATE, x, y, 0] ==
                                 runtime_samples[RuntimeParam.MSG_RATE, x, y])
                for t in plan_steps[1:]:
                    self.m.addQConstr(self.cost_vars[RuntimeParam.MSG_RATE, x, y, t], GRB.GREATER_EQUAL,
                        self.cost_vars[RuntimeParam.MSG_RATE, x, y, t - 1]
                        + quicksum([action_vars[a.tuple, t] * self.cost_vars[RuntimeParam.MSG_RATE, x, a.vj, t - 1]
                                  for a in all_acts if a.vi == x and a.vk == y])
                        + self.cost_vars[RuntimeParam.MSG_RATE, x, y, t - 1]
                        - quicksum([action_vars[a.tuple, t] * self.cost_vars[RuntimeParam.MSG_RATE, x, a.vj, t - 1]
                                    for a in all_acts if a.vi == y and a.vk == x])
                        + quicksum([action_vars[a.tuple, t] * self.cost_vars[RuntimeParam.MSG_RATE, x, a.vk, t - 1]
                                    for a in all_acts if a.vj == x and a.vk == y])
                        + quicksum([action_vars[a.tuple, t] * self.cost_vars[RuntimeParam.MSG_RATE, x, y, t - 1]
                                    for a in all_acts if a.vj == y and a.vk == x])
                        )

    def get_cost_per_action(self, all_acts, action_vars, plan_steps):
        if self.cost_model_type == CostModelType.FIXED:
            for a in all_acts:
                for t in plan_steps[1:]:
                    self.cost_per_action[a.tuple, t] = 1
        elif self.cost_model_type == CostModelType.RANDOM:
            for a in all_acts:
                for t in plan_steps[1:]:
                    self.cost_per_action[a.tuple, t] = random.randint(1, 10)
        elif self.cost_model_type == CostModelType.RUNTIME:
            for a in all_acts:
                for t in plan_steps[1:]:
                    self.cost_per_action[a.tuple, t] = \
                        self.cost_vars[RuntimeParam.MSG_RATE, a.vi, a.vj, t - 1] \
                        + self.cost_vars[RuntimeParam.MSG_RATE, a.vj, a.vi, t - 1] \
                        + self.cost_vars[RuntimeParam.MSG_RATE, a.vk, a.vj, t - 1]
        return self.cost_per_action


    # TODO: repair cost from file
    # depending on chosen cost model, return a dictionary that maps each (actions, step) to a cost
    # cost can be a constant integer or another variable
    # def get_cost_per_action_from_file(all_acts, plan_steps, options):
    #     cost_model = pconf.ActionCostModel[options.action_cost_model.upper()]
    #     action_costs = dict()
    #     for act in all_acts:
    #         for ps in plan_steps:
    #             action_costs[act.tuple, ps] = generate_action_cost(act, ps, cost_model)
    #     # if there is a cost file, override the values with it
    #     if options.cost_file:
    #         with open(options.cost_file, 'r') as f:
    #             lines = f.read().splitlines()
    #             lines = [l.strip() for l in lines]
    #             lines = [l for l in lines if l != '' and l[0] != '#']
    #         tuples = [ast.literal_eval(tuple_str) for tuple_str in lines]
    #         for t in tuples:  # list of tuples (action, step, cost)
    #             action_costs[t[0], t[1]] = t[2]
    #     return action_costs
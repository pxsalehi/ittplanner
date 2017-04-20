# Incremental Topology Transformation Planner

Requires Python3, Gurobi runtime library, `gurobipy`, `networkx` and `jsonpickle`.

Check options with `planner.sh`

Usage: `planner.sh [options]`

Example: `./planner.sh --planner=ip --size=30 --change-rate=0.1 --seed=254987 --threads=8 --plan-len-start=5 --max-steps=10 --prepasses=1 --presolve=1 --search-space=all --action-cost=fixed`
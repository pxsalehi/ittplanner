#!/usr/bin/env bash
# source activate py3
#python3 planner.py --size=30 --change-rate=0.1 --seed=254987 --threads=8 --plan-len-start=5 --max-steps=10 --prepasses=1 --presolve=1 --search-space=all --action-cost=fixed

if [ $# -eq 0 ]; then
    python3 planner.py --help
else
    python3 planner.py $@
fi
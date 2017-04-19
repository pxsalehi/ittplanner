import itertools
from enum import Enum
import random

__author__ = 'pxsalehi'


class RuntimeParam(Enum):
    MSG_RATE = 0


# runtime_param,x,y -> value
# e.g. MSG_RATE,x,y -> msg rate of flow from x to y
def get_samples(nodes):
    samples = dict()
    for x, y in itertools.permutations(nodes, 2):
        samples[RuntimeParam.MSG_RATE, x, y] = random.randint(1, 20)
    return samples

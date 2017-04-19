from .itt_proposition import *

__author__ = 'pxsalehi'


class Operator:

    # shift(i,j,k): removes link between i and j and establishes a link between i and k
    # pre = {conn(i, j), conn(j, k), rem(i, j)}
    # add = {conn(i, k), if conn(i,k) not in G: rem(i, k)}
    # del = {conn(i, j), rem(i, j)}
    class Shift:
        def __init__(self, target_topo, vi, vj, vk):
            self.vi = vi
            self.vj = vj
            self.vk = vk
            self.target_topo = target_topo
            self.preconds = self.add_effects = self.del_effects = None
            self.tuple = ('shift', self.vi, self.vj, self.vk)

        # return set of preconditions
        def get_pre(self):
            if self.preconds is None:
                self.preconds = {
                    Conn(self.vi, self.vj),
                    Conn(self.vj, self.vk),
                    Rem (self.vi, self.vj),
                }
            return self.preconds

        # return set of add effects
        def get_add(self):
            if self.add_effects is None:
                self.add_effects = {Conn(self.vi, self.vk)}
                if not self.target_topo.contains_edge((self.vi, self.vk)):
                    self.add_effects.add(Rem(self.vi, self.vk))
            return self.add_effects

        # return set of del effects
        def get_del(self):
            if self.del_effects is None:
                self.del_effects = {
                    Conn(self.vi, self.vj),
                    Rem(self.vi, self.vj)
                }
            return self.del_effects

        def __str__(self):
            return 'shift(%d,%d,%d)' % (self.vi, self.vj, self.vk)

        # def tuple(self):
        #     return 'shift', self.vi, self.vj, self.vk

        def __hash__(self):
            return hash(self.tuple)

        def __eq__(self, other):
            if isinstance(other, self.__class__):
                return self.tuple == other.tuple
            else:
                return False


    # move(i, j, k, l): directly replaces removable edge (i, j) with goal edge (k, l)
    # pre = {conn(i, l), conn(j, k), conn(i, j), rem(i, j)
    # add = {conn(k, l), if (k, l) not in G: rem(k, l)}
    # del = {conn(i, j), rem(i, j)}
    class Move:
        def __init__(self, target_topo, vi, vj, vk, vl):
            self.vi = vi
            self.vj = vj
            self.vk = vk
            self.vl = vl
            self.target_topo = target_topo
            self.preconds = self.add_effects = self.del_effects = None
            self.tuple = ('move', self.vi, self.vj, self.vk, self.vl)

        # return set of preconditions
        def get_pre(self):
            if self.preconds is None:
                self.preconds = {
                    Conn(self.vi, self.vl),
                    Conn(self.vj, self.vk),
                    Conn(self.vi, self.vj),
                    Rem (self.vi, self.vj),
                }
            return self.preconds

        # return set of add effects
        def get_add(self):
            if self.add_effects is None:
                self.add_effects = {Conn(self.vk, self.vl)}
                if not self.target_topo.contains_edge((self.vk, self.vl)):
                    self.add_effects.add(Rem(self.vk, self.vl))
            return self.add_effects

        # return set of del effects
        def get_del(self):
            if self.del_effects is None:
                self.del_effects = {
                    Conn(self.vi, self.vj),
                    Rem(self.vi, self.vj)
                }
            return self.del_effects

        def __str__(self):
            return 'move(%d,%d,%d,%d)' % (self.vi, self.vj, self.vk, self.vl)

        # def tuple(self):
        #     return 'move', self.vi, self.vj, self.vk, self.vl

            

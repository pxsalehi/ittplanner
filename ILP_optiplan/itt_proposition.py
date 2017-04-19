__author__ = 'pxsalehi'

# propositions
# they should all provide a __bool__ interface that evaluates their truth based on their value


class Proposition:
    def __hash__(self):
        return hash(self.tuple)

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return self.tuple == other.tuple
        else:
            return False


class Conn(Proposition):
    def __init__(self, vi, vj):
        if vi <= vj:
            self.vi = vi
            self.vj = vj
        else:
            self.vi = vj
            self.vj = vi
        self.tuple = ('conn', self.vi, self.vj)

    def __str__(self):
        return 'conn(%d,%d)' % (self.vi, self.vj)

    # def tuple(self):
    #     return 'conn', self.vi, self.vj

    def __hash__(self):
        return hash(self.tuple)


class Rem(Proposition):
    def __init__(self, vi, vj):
        if vi <= vj:
            self.vi = vi
            self.vj = vj
        else:
            self.vi = vj
            self.vj = vi
        self.tuple = ('rem', self.vi, self.vj)

    def __str__(self):
        return 'rem(%d,%d)' % (self.vi, self.vj)

    # def tuple(self):
    #     return 'rem', self.vi, self.vj

    def __hash__(self):
        return hash(self.tuple)

if __name__ == '__main__':
    test = set()
    test.add(Conn(2, 3))
    c1 = Conn(3, 2)
    print(c1 in test)
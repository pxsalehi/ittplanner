__author__ = 'pxsalehi'


class Topology:
    def __init__(self, size=0):
        self.root = None
        self.leaves = []
        self.size = size
        self.vertices = {i for i in range(0, size)}
        self.edges = set()  # set of tuples
        self.uris = dict()
        self.ids = dict()
        self.ips = dict()
        self.link_latencies = dict()  # tuple (from edges) to int

    def set_vertices(self, vertices):
        if self.vertices != vertices:
            self.vertices.clear()
            self.uris.clear()
            self.edges.clear()
            self.link_latencies.clear()
            self.vertices.update(vertices)
            self.size = len(vertices)

    def set_uris(self, uris):
        self.uris = uris.copy()

    def get_latency(self, e):
        if (e[0], e[1]) not in self.edges:
            e = (e[1], e[0])
        # decoded topologies have string instead of tuple
        if type(next(iter(self.link_latencies.keys()))) is str:
            return self.link_latencies[str(e)]
        return self.link_latencies[e]

    def add_edge(self, vi, vj):
        if vi != vj and (vi, vj) not in self.vertices and (vj, vi) not in self.vertices \
                and vi in self.vertices and vj in self.vertices:
            self.edges.add((vi, vj))

    def set_edges(self, edges):  # takes list of tuples
        self.edges.clear()
        for e in edges:
            if e[0] not in self.vertices or e[1] not in self.vertices:
                raise RuntimeError('Both edge nodes must be in vertices! ' + str(e))
            self.add_edge(e[0], e[1])

    def rem_edge(self, vi, vj):
        if (vi, vj) in self.edges:
            self.edges.remove((vi, vj))
        elif (vj, vi) in self.edges:
            self.edges.remove((vj, vi))

    def contains_edge(self, e):
        if (e[0], e[1]) in self.edges or (e[1], e[0]) in self.edges:
            return True
        return False

    def __str__(self):
        res = 'Vertices: '
        for v in sorted(self.vertices):
            res += str(v) + ','
        res += '\nEdges: '
        for e in sorted(self.edges):
            res += str(e) + ','
        return res

    def equals(self, other):
        for e in self.edges:
            if not other.contains_edge(e):
                return False
        for e in other.edges:
            if not self.contains_edge(e):
                return False
        return True

if __name__ == '__main__':
    topo = Topology(4)
    topo.set_edges([(0, 1), (0, 2), (1, 3)])
    assert topo.contains_edge((0, 1))
    assert topo.contains_edge((1, 0))
    assert not topo.contains_edge((2, 3))
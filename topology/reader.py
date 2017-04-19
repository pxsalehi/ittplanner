from topology.topology import Topology
import ast

__author__ = 'pxsalehi'


def create_topology_from_str(vertices_list, edges_str):
    t = Topology()
    uris = dict()
    for v in vertices_list:
        # split by space
        toks = v.split(' ')
        uris[int(toks[0].strip())] = toks[1].strip()
    t.set_uris(uris)
    t.set_vertices(uris.keys())
    # parse edges
    edges_str = edges_str.strip()
    if edges_str[0] != '[':
        edges_str = '[' + edges_str
    if edges_str[-1] != ']':
        edges_str += ']'
    edges = ast.literal_eval(edges_str)
    t.set_edges(edges)
    return t


# the file can have only two sections, one for vertices, one for edges
def create_topology_from_file(filename):
    vertices = []  # one line per broker
    edges = None  # all in one line
    with open(filename, 'r') as f:
        lines = f.read().splitlines()
        lines = [l.strip() for l in lines]
        lines = [l for l in lines if l != '' and l[0] != '#']
        # after cleaning up, all lines up to last are vertices, last is edges
        edges = lines[-1]
        vertices = lines[0:-1]
    return create_topology_from_str(vertices, edges)




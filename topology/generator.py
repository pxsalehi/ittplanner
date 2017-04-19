import random
import sys
import copy
import jsonpickle
import networkx as nx
import itertools
from topology.topology import Topology

__author__ = 'pxsalehi'


def random_gaussian(x, y):
    mu = (x + y) / 2.0
    sigma = abs(x - y) / 6.0
    while True:
        n = round(random.gauss(mu, sigma))
        if x <= n <= y:
            return n


def generate_tree(size, min_lat=10, max_lat=40, max_child=3, balanced=True, seed=random.randint):
    tree = Topology(size)
    # generate random tree nodes
    random.seed(seed)
    cur_row = [0]  # first row has node 0
    tree.root = 0
    next_link_id = 0
    next_node_id = 1
    while cur_row:
        cur_row_has_child = False
        next_node_id = max(cur_row) + 1
        next_row = []
        for node in cur_row:
            min_child = 1 if balanced else 0
            # if this is the last node in this row and there has been no child yet and there are
            # nodes left to place in the tree have at least one child!
            if not balanced and node == cur_row[-1] and not cur_row_has_child:
                min_child = 1
            if balanced:  # always have at least one child and use gaussian distribution
                child_count = random_gaussian(min_child, max_child)
            else:  # unbalanced
                # if possible to not have a child, 33% of the time have no child!
                if min_child == 0 and random.randint(0, 2) == 0:
                    child_count = 0
                else:
                    child_count = random.randint(min_child, max_child)  # - tree.node_degree[node]
            if child_count == 0 or next_node_id >= size:
                tree.leaves.append(node)
            for i in range(0, child_count):
                if next_node_id < size:
                    cur_row_has_child = True
                    child = next_node_id
                    latency = random.randint(min_lat, max_lat)
                    tree.add_edge(node, child)
                    tree.link_latencies[node, child] = latency
                    next_node_id += 1
                    next_link_id += 1
                    next_row.append(child)
        cur_row = next_row
    return tree


def read_topology_from_json(filename):
    with open(filename) as infile:
        return jsonpickle.decode(infile.read())


def write_topology_to_json(topology, filename):
    with open(filename, 'w') as outfile:
        json_str = jsonpickle.encode(topology)
        outfile.write(json_str)

def change_links(topo, change_rate, seed=None):
    if seed:
        random.seed(seed)
    new_topo = copy.deepcopy(topo)
    count = round(len(new_topo.edges) * change_rate)
    edge_list = list(new_topo.edges)
    to_remove = set()
    while len(to_remove) < count:
        i = random.randint(0, len(edge_list) - 1)
        to_remove.add(edge_list[i])
    for e in to_remove:
        new_topo.edges.remove(e)
    # add 'count' new edges that don't create cycle
    possible_edges = get_all_possible_edges(topo)
    nx_tree = convert_topo_to_networkx(new_topo)
    while nx_tree.size() < len(topo.edges):
        i = random.randint(0, len(possible_edges) - 1)
        edge = possible_edges[i]
        nx_tree.add_edge(edge[0], edge[1])
        # if there is a cycle, undo
        if not nx.is_forest(nx_tree):
            nx_tree.remove_edge(edge[0], edge[1])
        possible_edges.remove(edge)
    # Adding new edges such that edge_count = node_count - 1 and there is no cycle, results in a tree
    assert nx.is_tree(nx_tree)
    assert len(nx_tree.edges()) == len(topo.edges)
    new_topo = convert_from_networkx(nx_tree)
    return new_topo


def get_all_possible_edges(topo):
    edges = set()
    for e in itertools.permutations(topo.vertices, 2):
        if not topo.contains_edge(e):
            edges.add(e)
    return list(edges)


def convert_topo_to_networkx(topo):
    nxg = nx.Graph()
    nxg.add_nodes_from(topo.vertices)
    nxg.add_edges_from(topo.edges)
    return nxg


def convert_edges_to_networkx(edges):
    g = nx.Graph()
    g.add_edges_from(edges)
    return g


def convert_from_networkx(nx_topo):
    topo = Topology()
    topo.set_vertices(nx_topo.nodes())
    topo.set_edges(nx_topo.edges())
    return topo


if __name__ == '__main__':
    if len(sys.argv) <= 6:
        print(sys.argv[0], ' size seed max_child balanced|unbalanced min_latency max_latency [output_file]')
        exit()
    jsonpickle.set_encoder_options('simplejson', sort_keys=True, indent=4, compactly=False)
    # collect all parameters
    size = int(sys.argv[1])
    seed = int(sys.argv[2])
    max_child = int(sys.argv[3])
    balanced = True if sys.argv[4] == 'balanced' else False
    min_latency = int(sys.argv[5])
    max_latency = int(sys.argv[6])
    output_dir = None if len(sys.argv) == 7 else sys.argv[7]
    print('Generating topology with', str(sys.argv))
    tree = generate_tree(size, min_latency, max_latency, max_child, balanced, seed)
    print('root:', tree.root)
    print('leaves:', tree.leaves)
    if output_dir:
        write_topology_to_json(tree, "topo.json")
        print('saved json to', output_dir)

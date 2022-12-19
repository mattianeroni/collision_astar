import math 
import functools 
import itertools
import matplotlib.pyplot as plt 

import networkx as nx
from networkx.algorithms.graph_hashing import weisfeiler_lehman_graph_hash



def euclidean (G, source, target):
    """ Euclidean distance between two nodes """
    x1, y1 = G.nodes[source]['pos']
    x2, y2 = G.nodes[target]['pos']
    return math.sqrt( math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2) )


@functools.lru_cache(maxsize=None)
def cache_euclidean (G, source, target):
    """ Euclidean distance caching the results """
    return euclidean(G, source, target)



def make_hashable (cls):
    """ Make a Graph class hashable to use it in cache_euclidean. 
    Can be used also for DiGraph, MultiGraph, and MultiDiGraph """
    def __hash__(self):
        return hash(weisfeiler_lehman_graph_hash(self, node_attr='pos'))
    setattr(cls, "__hash__", __hash__)


def plot_path (G, path, color='red'):
    """ Method to plot a graph by highlighting the provided path """
    pos = {i: node['pos'] for i, node in G.nodes.items()}
    path_edges = tuple(zip(path[:-1], path[1:]))
    nx.draw(G, pos=pos, with_labels=True, node_size=100, font_size=6, font_weight="bold")
    nx.draw_networkx_edges(G, pos=pos, edgelist=path_edges, edge_color=color)
    plt.show()
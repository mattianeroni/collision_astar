from heapq import heappop, heappush
from itertools import count

import networkx as nx
from networkx.algorithms.shortest_paths.weighted import _weight_function


def collision_astar():
    """ An implementation of A* algorithm that consider additional aspects
    design to avoid collisions and risks """
    pass
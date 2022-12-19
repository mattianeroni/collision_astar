import networkx as nx 
import time

from src.utils import euclidean, cache_euclidean, make_hashable, plot_path
from src.collision_astar import collision_astar


if __name__ == '__main__':

    #make_hashable(nx.Graph)

    G = nx.Graph()
    G.add_node(0, pos=(0, 0))
    G.add_node(1, pos=(10, 10))
    G.add_node(2, pos=(10, -5))
    G.add_node(3, pos=(20, 0))
    G.add_edge(0, 1, weight=euclidean(G, 0, 1), risk=10)
    G.add_edge(0, 2, weight=euclidean(G, 0, 2), risk=5)
    G.add_edge(1, 3, weight=euclidean(G, 1, 3), risk=10)
    G.add_edge(2, 3, weight=euclidean(G, 2, 3), risk=20)
    

    path = collision_astar(G, 0, 3, heuristic=euclidean, alpha=1000.0, weight='weight', risk='risk')
    print(path)

    plot_path(G, path)    
    



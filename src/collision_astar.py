from heapq import heappop, heappush
from itertools import count

import networkx as nx
from networkx.algorithms.shortest_paths.weighted import _weight_function



def _risk_function(G, risk):
    """ Returns a function that returns the risk of an edge.
    It works similarly to networkx _weight_function. """
    # If risk is already a function...
    if callable(risk):
        return risk

    # If the weight keyword argument is not callable, we assume it is a
    # string representing the edge attribute containing the weight of
    # the edge.
    if G.is_multigraph():
        return lambda u, v, d: min(attr.get(risk, 0) for attr in d.values())

    return lambda u, v, data: data.get(risk, 0)



def _time_windows_function (G, time_windows):
    """ Returns a function that returns the busy time windows of an edge.
    It works similarly to networkx _weight_function. """
    if G.is_multigraph():
        return lambda u, v, d: min( 
            iterable=(attr.get(time_windows, []) for attr in d.values()), 
            key=lambda i: len(i.get(time_windows, []))
        )

    return lambda u, v, data: data.get(time_windows, tuple())



def collision_astar (G, source, target, heuristic=None, alpha=0.0,
    weight="weight", risk="risk", time_windows="time_windows"):
    
    """ 
    An implementation of A* (A-star) algorithm that consider additional aspects
    design to avoid collisions and risks 
    
    Returns a list of nodes in a shortest path between source and target.
    There may be more than one shortest path.  This returns only one.
    
    :param G: The graph (should be hashable if hashable heuristic is provided)
    :param source: Id of the source node
    :param target: Id of the target node 
    :param heuristic: The heuristic function used to estimate the quality of a path
    :param alpha: The weight given to the risk (default is zero - same as no risk)

    :param weight: String or function describing the edges weight. If this is a string, 
        then edge weights will be accessed via the edge attribute with this key (that 
        is, the weight of the edge joining `u` to `v` will be ``G.edges[u, v][weight]``). 
        If no such edge attribute exists, the weight of the edge is assumed to be one.
        If this is a function, the weight of an edge is the value returned by the function. 
        The function must accept exactly three positional arguments: the two endpoints of 
        an edge and the dictionary of edge attributes for that edge. The function must
        return a number or None to indicate a hidden edge.

    :param risk: String or function describing the edges risk. If this is a string, 
        then edge risks will be accessed via the edge attribute with this key (that 
        is, the risk of the edge joining `u` to `v` will be ``G.edges[u, v][risk]``). 
        If no such edge attribute exists, the risk of the edge is assumed to be zero.
        If this is a function, the risk of an edge is the value returned by the function. 
        The function must accept exactly three positional arguments: the two endpoints of 
        an edge and the dictionary of edge attributes for that edge. The function must
        return a number.

    :param time_windows: String saying which attribute of the edges is a list of time windows

    :return: A list of nodes id belonging to the shortest path.    
    """
    if source not in G or target not in G:
        msg = f"Either source {source} or target {target} is not in G"
        raise nx.NodeNotFound(msg)

    if heuristic is None:
        # The default heuristic is h=0 - same as Dijkstra's algorithm
        def heuristic(u, v):
            return 0

    push = heappush
    pop = heappop
    get_weight = _weight_function(G, weight)
    get_risk = _risk_function(G, risk)
    get_time_windows = _time_windows_function(G, time_windows)

    # The queue stores priority, counter, node, cost to reach node, risk to reach node, and parent.
    # Uses Python heapq to keep in priority order.
    # Add a counter to the queue to prevent the underlying heap from
    # attempting to compare the nodes themselves. The hash breaks ties in the
    # priority and is guaranteed unique for all nodes in the graph.
    c = count()
    queue = [(0, next(c), source, 0, 0, None)]

    # Maps enqueued nodes to distance of discovered paths and the
    # computed heuristics to target. 
    # We avoid computing the heuristics more than once and inserting 
    # the node into the queue too many times.
    enqueued = {}
    # Maps explored nodes to parent closest to the source.
    explored = {}

    while queue:
        # Pop the smallest item from queue.
        _, __, curnode, totalcost, totalrisk, parent = pop(queue)

        # Reach the target
        if curnode == target:
            path = [curnode]
            node = parent
            while node is not None:
                path.append(node)
                node = explored[node]
            path.reverse()
            return path

        if curnode in explored:
            # Do not override the parent of starting node
            if explored[curnode] is None:
                continue

            # Skip bad paths that were enqueued before finding a better one
            qcost, h, r = enqueued[curnode]
            if qcost < totalcost:
                continue

        explored[curnode] = parent

        for neighbor, w in G[curnode].items():

            # Get cost and risk associated to new edge
            cost = get_weight(curnode, neighbor, w)
            risk = get_risk(curnode, neighbor, w)
            
            # If cost is None this arc cannot be covered
            if cost is None:
                continue

            ncost = totalcost + cost
            nrisk = totalrisk + risk
            n_cost_and_risk = ncost + alpha * nrisk

            if neighbor in enqueued:
                qcost, h, r = enqueued[neighbor]
                # If the weighted 
                # Therefore, we won't attempt to push this neighbor
                # to the queue.
                if qcost + alpha * r <= n_cost_and_risk:
                    continue
            else:
                h = heuristic(G, neighbor, target)
                
            enqueued[neighbor] = (ncost, h, nrisk)
            push(queue, (n_cost_and_risk + h, next(c), neighbor, ncost, nrisk, curnode))

    raise nx.NetworkXNoPath(f"Node {target} not reachable from {source}")

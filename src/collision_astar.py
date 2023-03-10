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


def _time_function (G, time, weight_function):
    """ Returns a function that returns the travel time of an edge.
    It workd similarly to networkx _weight_function. """
    # If risk is already a function...
    if callable(time):
        return time

    # If the weight keyword argument is not callable, we assume it is a
    # string representing the edge attribute containing the weight of
    # the edge.
    if G.is_multigraph():
        return lambda u, v, d: min(
            iterable=(attr.get(time, weight_function(u, v, d)) for attr in d.values()),
            key=weight_function(u, v, d)
        )

    return lambda u, v, data: data.get(time, weight_function(u, v, data))



def _time_windows_function (G, time_windows):
    """ Returns a function that returns the busy time windows of an edge.
    It works similarly to networkx _weight_function. """
    if G.is_multigraph():
        return lambda u, v, d: min( 
            iterable=(attr.get(time_windows, []) for attr in d.values()), 
            key=lambda i: len(i.get(time_windows, []))
        )

    return lambda u, v, data: data.get(time_windows, tuple())



def interested_time_window (ctime, time_windows):
    """ Check through a set of time windows if one of them
    interests the current travel """
    return next((i for i in time_windows if i[0] <= ctime < i[1]), None)



def build_path (curnode, tree, time_windows):
    """ 
    Method to reconstruct the path starting from the current node.
    :param target: The target node id 
    :param tree: The dict of explored nodes --i.e., tree[child] = parent
    :param time_windows: The dictionary of time windows associated with 
                         current exploration --i.e., tw[edge] = time_window
    :return: The path from source to target as a list of nodes ids, and
            the time windows associated with edges.
    """
    path = [curnode]
    node = tree[curnode]
    windows = {(node, curnode) : time_windows[node, curnode]}
    while node is not None:
        path.append(node)
        parent = tree[node]
        if parent is not None:
            windows[parent, node] = time_windows[parent, node]
        node = parent
    path.reverse()
    return path, windows



def collision_astar (G, source, target, heuristic=None, alpha=0.0,
    weight="weight", risk="risk", time="time", time_windows="time_windows"):
    
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

    :param time: String of function describing the edges travel time. If this is a string, 
        then edge risks will be accessed via the edge attribute with this key (that 
        is, the risk of the edge joining `u` to `v` will be ``G.edges[u, v][risk]``). 
        If no such edge attribute exists, the risk of the edge is assumed to be equal 
        to the weight. If this is a function, the risk of an edge is the value returned 
        by the function. The function must accept exactly three positional arguments: 
        the two endpoints of an edge and the dictionary of edge attributes for that edge. 
        The function must return a number.

    :param time_windows: String saying which attribute of the edges is a list of time windows

    :return: A list of nodes id belonging to the shortest path.    
    """
    if source not in G or target not in G:
        raise nx.NodeNotFound(f"Either source {source} or target {target} is not in G")

    if heuristic is None:
        # The default heuristic is h=0 - same as Dijkstra's algorithm
        def heuristic(u, v):
            return 0

    push = heappush
    pop = heappop
    get_weight = _weight_function(G, weight)
    get_risk = _risk_function(G, risk)
    get_time = _time_function(G, time, get_weight)
    get_time_windows = _time_windows_function(G, time_windows)

    # The queue stores priority, counter, node, cost to reach node, risk 
    # to reach node, time to reach node and parent.
    # Uses Python heapq to keep in priority order.
    # Add a counter to the queue to prevent the underlying heap from
    # attempting to compare the nodes themselves. The hash breaks ties in the
    # priority and is guaranteed unique for all nodes in the graph.
    c = count()
    queue = [(0, next(c), source, 0, 0, 0, None)]

    # Maps enqueued nodes to (distance + alpha * risk) and heuristic estimation.
    # In this way, we avoid computing the heuristics more than once.
    enqueued = {}
    # Maps explored nodes to parent closest to the source.
    explored = {}
    # Maps each visited edge to a time window in which the current trip will keep it busy.
    windows = {}

    while queue:
        # Pop the smallest item from queue.
        # (priority, counter, node, cumulate distance, cumulate risk, cumulate time, parent)
        _, __, curnode, cdist, crisk, ctime, parent = pop(queue)

        # Reach the target
        if curnode == target:
            explored[curnode] = parent
            return build_path(target, explored, windows)

        if curnode in explored:
            # Do not override the parent of starting node
            if explored[curnode] is None:
                continue

            # Skip bad paths that were enqueued before finding a better one
            cost, h, r = enqueued[curnode]
            if cost + alpha * r < cdist + alpha * crisk:
                continue

        explored[curnode] = parent

        for neighbor, w in G[curnode].items():

            # Get cost (if cost is None this arc cannot be covered)
            cost = get_weight(curnode, neighbor, w)
            if cost is None:
                continue

            # Get also risk, travel time, and time windows associated with edge
            risk = get_risk(curnode, neighbor, w)
            time = get_time(curnode, neighbor, w)
            time_windows_list = get_time_windows(curnode, neighbor, w)

            # Compute cost, risk, and time if that edge is covered
            new_cost = cdist + cost
            new_risk = crisk + risk
            new_value = new_cost + alpha * new_risk
            # Time when edge (curnode, neighbor) is entered and exited
            entry_t, exit_t = 0, 0   

            # Adjust time windows associated with considered arcs
            if (tw := interested_time_window(ctime, time_windows_list)) is None:
                new_time = ctime + time
                entry_t, exit_t = ctime, new_time
            else:
                tw_init, tw_end = tw
                new_time = tw_end + time
                windows[parent, curnode][1] = tw_end
                entry_t, exit_t = tw_end, new_time
            
            # Update time window when edge (curnode, neighbor) is covered
            windows[curnode, neighbor] = [entry_t, exit_t]

            if neighbor in enqueued:
                cost, h, r = enqueued[neighbor]
                # If the weighted value --i.e., distance + alpha * risk -- does 
                # not provide any significant improvement, we won't attempt to 
                # push this neighbor to the queue.
                if cost + alpha * r <= new_value:
                    continue
            else:
                h = heuristic(G, neighbor, target)
                
            enqueued[neighbor] = (new_cost, h, new_risk)
            push(queue, (new_value + h, next(c), neighbor, new_cost, new_risk, new_time, curnode))

    raise nx.NetworkXNoPath(f"Node {target} not reachable from {source}")

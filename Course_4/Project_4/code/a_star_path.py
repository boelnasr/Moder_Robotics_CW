import heapq
import os


def read_graph(node_fn, edge_fn): ## this libarrary to read elements of the given Graph
    
    nodes = []
    with open(node_fn) as fd:
        for line in fd:
            if line.startswith("#"):
                continue
            split = line.split(",")
            node_id = int(split[0])
            x,y,h_cost = map(float, split[1:])
            nodes.append((node_id, x, y, h_cost))

    edges = []
    with open(edge_fn) as fd:
        for line in fd:
            if line.startswith("#"):
                continue
            split = line.split(",")
            n1,n2 = map(int, split[:2])
            cost = float(split[2])
            edges.append(((n1,n2), cost))
            edges.append(((n2,n1), cost))
    return nodes, dict(edges)

def edges_from(node, edges):
    return [((n1,n2),c) for ((n1, n2), c) in edges.items() if n1 == node]

def get_h_cost(n_id, nodes):
    for node in nodes:
        if node[0] == n_id:
            return node[-1]

def astar_search(nodes, edges, start, end):#### for the A* search algo
    assert start in [n[0] for n in nodes], "Invalid start"
    assert end in [n[0] for n in nodes], "Invalid end"
    visited = set()
    open_list = [(get_h_cost(start, nodes), (start, None), 0)]
    path_costs = {}
    parents = {}
    while end not in visited:
        (_, (node, parent), path_cost) = heapq.heappop(open_list)
        if node in visited:
            continue 
        if parent: 
            parents[node] = parent 
        for ((_,n2), e_cost) in edges_from(node, edges):
            if n2 not in visited:
                heapq.heappush(open_list, ((e_cost + path_cost + get_h_cost(n2, nodes), (n2, node), (e_cost + path_cost))))
        visited.add(node)
    path = [end]
    node = end
    while node != start:
        path.append(parents[node])
        node = parents[node]
    return path[::-1]

def path_cost(path, edges):
    cost = 0
    for i,node in enumerate(path[1:]):
        prev = path[i]
        cost += edges[(prev,node)]
    return cost

if __name__ == "__main__":
    node_fn = os.path.join(os.getcwd(), "results", "nodes.csv")
    edge_fn = os.path.join(os.getcwd(), "results", "edges.csv")
    nodes, edges = read_graph(node_fn, edge_fn)
    start = 1
    end = 12
    path = astar_search(nodes, edges, start, end)
    with open(os.path.join(os.getcwd(), "results", "path.csv"), "w") as fd:
        fd.write(",".join(map(str, path)))
    print(f"Computed path, total_cost={path_cost(path, edges)}")
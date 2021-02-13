from collections import deque
import os
import numpy as np

# Define a simple TreeNode struct/record.
class TreeNode():
    def __init__(self, x, parent, children, id):
        self.x = x
        self.parent = parent
        self.children = children
        self.id = id

# Parameters
MAX_TREE_SIZE = 10000
GOAL_SAMPLE_PROB = 0.1

# This function follows Algorithm 10.3 in the textbook.
# The goal is a single node rather than a set.
def rrt(x_start, x_goal, obstacles):
    
    root = TreeNode(x_start, None, [], 1)
    # Get the nearest node by go through the whole tree and find the node with least distance
    def get_nearest_node(pos):
        best_node = root
        best_dist = (root.x[0]-pos[0])**2 + (root.x[1]-pos[1])**2
        que = deque([root])
        while len(que) > 0:
            node = que.popleft()
            dist = (node.x[0]-pos[0])**2 + (node.x[1]-pos[1])**2
            if dist < best_dist:
                best_dist = dist
                best_node = node
            que.extend(node.children)
        return best_node
    # Check whether the line segment from x_1 to x_2 collides with obstacles
    def collision_free(x_1, x_2):
        for ob in obstacles:
            R = ob[2]/2 # radius
            C = ob[:2]  # center
            # We solve the quadratic equation defined by |x_1*t+(1-t)*x_2-C|=R
            # There is collision if the equation has at least one real solution t where 0<=t<=1
            A = x_1 - x_2
            B = x_2 - C
            adota = A.dot(A)
            adotb = A.dot(B)
            bdotb = B.dot(B)
            a = adota
            b = 2*adotb
            c = bdotb-R**2
            if b**2 >= 4*a*c:
                t1 = (-b+np.sqrt(b**2-4*a*c))/(2*a)
                t2 = (-b-np.sqrt(b**2-4*a*c))/(2*a)
                if 0<=t1<=1 or 0<=t2<=1:
                    return False
        return True
    tree_size = 1
    # Return solution representations once goal node is added to the RRT tree
    def get_sol(goal_node):
        nodes = np.zeros([tree_size, 4])
        edges = np.zeros([tree_size-1, 3]) # graph theory guarantee that in trees #edges = #nodes - 1
        # fill nodes and edges by go through the tree
        que = deque([root])
        edge_index = 0
        while len(que) > 0:
            node = que.popleft()
            nodes[node.id-1] = [node.id, node.x[0], node.x[1], np.linalg.norm(node.x-x_goal)]
            que.extend(node.children)
            for child in node.children:
                edges[edge_index] = [node.id, child.id, np.linalg.norm(node.x-child.x)]
                edge_index += 1
        # get the path by follow parent pointer from the goal node
        path = []
        node = goal_node
        while node is not None:
            path.append(node.id)
            node = node.parent
        path.reverse()
        return nodes, edges, np.array(path)

    while tree_size < MAX_TREE_SIZE:
        # sample goal node with certain probability as specied
        if np.random.binomial(1, GOAL_SAMPLE_PROB) == 1:
            x_samp = x_goal
        else:
            x_samp = np.random.uniform(-0.5, 0.5, 2)
        node_nearest = get_nearest_node(x_samp)
        # since we use straight line path; local planning is trivial
        node_new = TreeNode(x_samp, None, [], None)
        if collision_free(node_nearest.x, node_new.x):
            # insert node_new under node_nearest
            tree_size += 1
            node_new.id = tree_size
            node_nearest.children.append(node_new)
            node_new.parent = node_nearest
            # numpy returns element-wise equality so add .all()
            if (node_new.x == x_goal).all():
                # SUCCESS build and return solution representations
                return get_sol(node_new)
    # FAILURE represents by an exception.
    raise Exception('No path to %s is found.' % x_goal)

    
obstacles =os.path.join(os.getcwd(), "results", "obstacles.csv")
node_fn = os.path.join(os.getcwd(), "results", "nodes.csv")
edge_fn = os.path.join(os.getcwd(), "results", "edges.csv") 
x_start = np.array([-0.5, -0.5])
x_goal = np.array([0.5, 0.5])
nodes, edges, path = rrt(x_start, x_goal, obstacles)
with open(os.path.join(os.getcwd(), "results", "path.csv"), "w") as fd:
        fd.write(",".join(map(str, path)))

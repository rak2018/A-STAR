<h1>ExpNo 4 : Implement A* search algorithm for a Graph</h1> 
<h3>Name: RAKSHITHA P      </h3>
<h3>Register Number: 212224060205          </h3>
<H3>Aim:</H3>
<p>To ImplementA * Search algorithm for a Graph using Python 3.</p>
<H3>Algorithm:</H3>

``````
// A* Search Algorithm
1.  Initialize the open list
2.  Initialize the closed list
    put the starting node on the open 
    list (you can leave its f at zero)

3.  while the open list is not empty
    a) find the node with the least f on 
       the open list, call it "q"

    b) pop q off the open list
  
    c) generate q's 8 successors and set their 
       parents to q
   
    d) for each successor
        i) if successor is the goal, stop search
        
        ii) else, compute both g and h for successor
          successor.g = q.g + distance between 
                              successor and q
          successor.h = distance from goal to 
          successor (This can be done using many 
          ways, we will discuss three heuristics- 
          Manhattan, Diagonal and Euclidean 
          Heuristics)
          
          successor.f = successor.g + successor.h

        iii) if a node with the same position as 
            successor is in the OPEN list which has a 
           lower f than successor, skip this successor

        iV) if a node with the same position as 
            successor  is in the CLOSED list which has
            a lower f than successor, skip this successor
            otherwise, add  the node to the open list
     end (for loop)
  
    e) push q on the closed list
    end (while loop)

``````

<hr>
<h2>Sample Graph I</h2>
<hr>

![image](https://github.com/natsaravanan/19AI405FUNDAMENTALSOFARTIFICIALINTELLIGENCE/assets/87870499/b1377c3f-011a-4c0f-a843-516842ae056a)

<hr>
<h2>Sample Input</h2>
<hr>
10 14 <br>
A B 6 <br>
A F 3 <br>
B D 2 <br>
B C 3 <br>
C D 1 <br>
C E 5 <br>
D E 8 <br>
E I 5 <br>
E J 5 <br>
F G 1 <br>
G I 3 <br>
I J 3 <br>
F H 7 <br>
I H 2 <br>
A 10 <br>
B 8 <br>
C 5 <br>
D 7 <br>
E 3 <br>
F 6 <br>
G 5 <br>
H 3 <br>
I 1 <br>
J 0 <br>
<hr>
<h1>PROGRAM</h1>
from collections import defaultdict

# Heuristic dictionary (stores estimated distance to goal)
H_dist = {}

def aStarAlgo(start_node, stop_node):
    """
    A* Search Algorithm implementation.
    :param start_node: Starting point
    :param stop_node: Goal node
    """
    open_set = set(start_node)
    closed_set = set()
    g = {}        # Actual cost from start node to current node
    parents = {}  # Stores the parent of each node for path reconstruction

    g[start_node] = 0
    parents[start_node] = start_node

    while len(open_set) > 0:
        n = None
        # Select node with lowest f(n) = g(n) + h(n)
        for v in open_set:
            if n is None or g[v] + heuristic(v) < g[n] + heuristic(n):
                n = v

        if n is None:
            print("Path does not exist!")
            return None

        # If goal is reached, reconstruct the path
        if n == stop_node:
            path = []
            while parents[n] != n:
                path.append(n)
                n = parents[n]
            path.append(start_node)
            path.reverse()

            print('✅ Path found:', path)
            return path

        # Explore neighbors of the current node
        for (m, weight) in get_neighbors(n):
            if m not in open_set and m not in closed_set:
                open_set.add(m)
                parents[m] = n
                g[m] = g[n] + weight
            else:
                # Update cost if a shorter path is found
                if g[m] > g[n] + weight:
                    g[m] = g[n] + weight
                    parents[m] = n
                    if m in closed_set:
                        closed_set.remove(m)
                        open_set.add(m)

        # Move n from open_set to closed_set
        open_set.remove(n)
        closed_set.add(n)

    print("❌ Path does not exist!")
    return None


def get_neighbors(v):
    """
    Returns all connected nodes (neighbors) of a given vertex.
    """
    if v in Graph_nodes:
        return Graph_nodes[v]
    else:
        return []


def heuristic(n):
    """
    Returns the heuristic distance for a given node.
    """
    return H_dist.get(n, 0)


# --------------------- MAIN EXECUTION ---------------------
if _name_ == "_main_":
    graph = defaultdict(list)

    print("Enter number of nodes and edges:")
    n, e = map(int, input().split())

    print("\nEnter edges in the format: <u> <v> <cost>")
    for i in range(e):
        u, v, cost = map(str, input().split())
        t = (v, int(cost))
        graph[u].append(t)
        t1 = (u, int(cost))
        graph[v].append(t1)

    print("\nEnter heuristic values for each node: <node> <h>")
    for i in range(n):
        node, h = map(str, input().split())
        H_dist[node] = int(h)

    Graph_nodes = graph

    print("\nEnter start and goal node:")
    start = input("Start node: ").strip()
    goal = input("Goal node: ").strip()

    print("\n--- Running A* Algorithm ---")
    aStarAlgo(start, goal)

<h2>Sample Output</h2>
<hr>
Path found: ['A', 'F', 'G', 'I', 'J']


<hr>
<h2>Sample Graph II</h2>
<hr>

![image](https://github.com/natsaravanan/19AI405FUNDAMENTALSOFARTIFICIALINTELLIGENCE/assets/87870499/acbb09cb-ed39-48e5-a59b-2f8d61b978a3)


<hr>
<h2>Sample Input</h2>
<hr>
6 6 <br>
A B 2 <br>
B C 1 <br>
A E 3 <br>
B G 9 <br>
E D 6 <br>
D G 1 <br>
A 11 <br>
B 6 <br>
C 99 <br>
E 7 <br>
D 1 <br>
G 0 <br>
<hr>
<h1>PROGRAM</h1>
"""from collections import defaultdict

# Heuristic dictionary
H_dist = {}

# ------------------- A* Algorithm Function -------------------
def aStarAlgo(start_node, stop_node):
    open_set = set(start_node)
    closed_set = set()
    g = {}          # Stores cost from start to current node
    parents = {}    # Stores the parent of each node

    g[start_node] = 0
    parents[start_node] = start_node

    while len(open_set) > 0:
        n = None

        # Find node with lowest f(n) = g(n) + h(n)
        for v in open_set:
            if n is None or g[v] + heuristic(v) < g[n] + heuristic(n):
                n = v

        if n is None:
            print("Path does not exist!")
            return None

        # If goal reached → reconstruct path
        if n == stop_node:
            path = []
            while parents[n] != n:
                path.append(n)
                n = parents[n]
            path.append(start_node)
            path.reverse()
            print("✅ Path found:", path)
            return path

        # Explore neighbors
        if n in Graph_nodes:
            for (m, weight) in get_neighbors(n):
                if m not in open_set and m not in closed_set:
                    open_set.add(m)
                    parents[m] = n
                    g[m] = g[n] + weight
                else:
                    if g[m] > g[n] + weight:
                        g[m] = g[n] + weight
                        parents[m] = n
                        if m in closed_set:
                            closed_set.remove(m)
                            open_set.add(m)

        open_set.remove(n)
        closed_set.add(n)

    print("❌ Path does not exist!")
    return None


# ------------------- Helper Functions -------------------
def get_neighbors(v):
    """Return all neighbors of a given vertex"""
    return Graph_nodes.get(v, [])


def heuristic(n):
    """Return the heuristic distance for a node"""
    return H_dist.get(n, 0)


# ------------------- Main Program -------------------
if __name__ == "__main__":
    graph = defaultdict(list)

    # Input section
    print("Enter number of nodes and edges:")
    n, e = map(int, input().split())

    print("\nEnter edges in the format: <u> <v> <cost>")
    for _ in range(e):
        u, v, cost = map(str, input().split())
        t = (v, int(cost))
        graph[u].append(t)
        t1 = (u, int(cost))
        graph[v].append(t1)

    print("\nEnter heuristic values for each node: <node> <h>")
    for _ in range(n):
        node, h = map(str, input().split())
        H_dist[node] = int(h)

    Graph_nodes = graph

    print("\nEnter start and goal nodes:")
    start = input("Start node: ").strip()
    goal = input("Goal node: ").strip()

    print("\n--- Running A* Algorithm ---")
    aStarAlgo(start, goal)

<h2>Sample Output</h2>
<hr>
Path found: ['A', 'E', 'D', 'G']

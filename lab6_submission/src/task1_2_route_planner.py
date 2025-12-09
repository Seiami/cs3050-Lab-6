#!/usr/bin/env python3
"""
Route Planner with Dijkstra, A*, and Bellman-Ford algorithms
"""
import copy
import sys
import csv
import heapq
import math
from typing import Dict, List, Tuple, Optional

EARTH_RADIUS = 6371.0  # km


class Node:
    """Represents a node in the graph"""
    def __init__(self, node_id: int, lat: float, lon: float):
        self.id = node_id
        self.lat = lat
        self.lon = lon


class Edge:
    """Represents an edge in the graph"""
    def __init__(self, to: int, weight: float):
        self.to = to
        self.weight = weight


class Graph:
    """Graph data structure with adjacency list"""
    def __init__(self):
        self.nodes: Dict[int, Node] = {}
        self.edges: List[Edge] = []
        self.adj_list: Dict[int, List[Edge]] = {}
    
    def add_node(self, node_id: int, lat: float, lon: float):
        """Add a node to the graph"""
        self.nodes[node_id] = Node(node_id, lat, lon)
        if node_id not in self.adj_list:
            self.adj_list[node_id] = []
    
    def add_edge(self, from_id: int, to_id: int, weight: float):
        """Add an edge to the graph"""
        if from_id not in self.adj_list:
            self.adj_list[from_id] = []
        self.adj_list[from_id].append(Edge(to_id, weight))
        self.edges.append(Edge(to_id, weight))


def haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Calculate haversine distance between two points"""
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)
    
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return EARTH_RADIUS * c

'''
A shortest-path finder helper function for my priority-route-planning dijkstra algorithm. I use it to find paths and distances between 2 nodes.
'''
def dijkstra_vanilla(graph: Graph, start: int, end: int) -> Tuple[Dict[int, float], Dict[int, Optional[int]], int]:
    """
    Dijkstra's algorithm for shortest path
    Returns: (distances, previous nodes, nodes explored)
    """
    dist = {node_id: float('inf') for node_id in graph.nodes}
    prev = {node_id: None for node_id in graph.nodes}
    dist[start] = 0
    
    pq = [(0, start)]
    nodes_explored = 0
    visited = set()
    
    while pq:
        current_dist, u = heapq.heappop(pq)
        
        if u in visited:
            continue
        
        visited.add(u)
        nodes_explored += 1
        
        if u == end:
            break
        
        if current_dist > dist[u]:
            continue
        
        for edge in graph.adj_list.get(u, []):
            v = edge.to
            alt = dist[u] + edge.weight
            
            if alt < dist[v]:
                dist[v] = alt
                prev[v] = u
                heapq.heappush(pq, (alt, v))
                
    
    return dist, prev, visited


def highest_priority(dests: List[Tuple[int, str]]):
    '''
        Helper function for dijkstra.
        Takes a to-visit destination list and returns a node id and the priority for the first node with highest priority in the list.
    '''
    priorities = ["HIGH", "MEDIUM", "LOW"]
    if dests:
        for p in priorities:
            for dest, priority in dests:  # Find the first node with the highest priority
                if priority == p:
                    return dest, priority
    return None, None


def find_priority(dests: List[Tuple[int, str]], id: int):
    '''
        Helper function for dijkstra-based priority route planner. Finds the priority associated with a given node id in the desintation list.
        Returns this priority and takes the destination list and node id.
    '''
    for dest, priority in dests:
        if dest == id:
            return priority
    return None # Node is not a destination
        
def get_shortest_path(graph: Graph, start: int, dests: List[Tuple[int, str]]):
    dests = dests.copy()
    total_dist = 0
    og_start = start
    
    while dests:        
        end, p = dests[0]
        # Ensure nodes in destination list exist on graph
        if end not in graph.nodes:
            print(f"Destination not on graph: {end}")
            return None, None, None
                
        # Run Dijkstra from the start node (or last node) to the next unvisited node in our destination list with the highest priority
        dist, prev, visited = dijkstra_vanilla(graph, start, end)
        if end in visited: # if path found to destination
            path = reconstruct_path(prev, start, end) # get path
            if start != og_start:
                path.pop(0) # Remove the first element from the path as it is a duplicate
            start = end # update start destination to node ended on
            total_dist += dist[end] # Update total distance
            dests.remove([end, p]) # remove previous end
            # loop through visit list and remove all nodes visited on recent path
            for node in path:
                if node in dests:
                    pri = find_priority(dests, node.id)
                    dests.remove([node.id, pri])
                    
    return total_dist


def dijkstra(graph: Graph, start: int, dests: List[Tuple[int, str]], threshold: float):
    '''
        I once more picked Dijkstra's algorithm for much the same reasons as in task 1.1: since we are simulating real travel on roads, distances cannot be negative.
        Therefore, I do not need the more complex A* or Bellman-Ford to handle negative edge weights, and Dijkstra's is the simplest and easiest to work with.
        I altered the function parameters to fit the input of a dictionary for priorities and a threshold for the priority-respecting path length.
        I decided that a dictionary was the best way to keep the destinations together with their priorities in a clearly related way.
    '''
            
    dests = dests.copy() # Don't want to destroy main values!
    total_dist = 0
    priority_violations = 0
    route = []
    og_start = start
    prev_p = None
    
    shortest_dist = get_shortest_path(graph, start, dests)
    print(f"Shortest possible distance: {shortest_dist:.2f} km\n")
    
    while dests:
        graph2 = copy.deepcopy(graph)
        end, p = highest_priority(dests)
        if prev_p != p:
            # Reweight edges to take both priority and threshold into account
            for key in graph2.adj_list.keys():
                for i in range(len(graph2.adj_list[key])):
                    edge = graph2.adj_list[key][i]
                    b = 0
                    if find_priority(dests, edge.to) == p:
                        b = 1
                    edge.weight = (1-threshold)*b + threshold*edge.weight
        
        
        # Ensure nodes in destination list exist on graph
        if end not in graph2.nodes:
            print(f"Destination not on graph: {end}")
            return None, None, None
                
        # Run Dijkstra from the start node (or last node) to the next unvisited node in our destination list with the highest priority
        dist, prev, visited = dijkstra_vanilla(graph2, start, end)
        if end in visited: # if path found to destination
            path = reconstruct_path(prev, start, end) # get path
            # Loop through edges and fix their weights for distance calculations; we messed with them earlier to apply a cost function (for thresholding)
            for i in range(len(path)-1):
                al = graph.adj_list[path[i]] # al = current adj_list for i (changed by cost function application)
                for edge in al:
                    if edge.to == path[i+1]: # Loop through all of the edges until we reach one used in our path; add its original weight to our total distance
                        total_dist += edge.weight
                        break
            if start != og_start:
                path.pop(0) # Remove the first element from the path as it is a duplicate
            start = end # update start destination to node ended on
            #total_dist += dist[end] # Update total distance
            dests.remove([end, p]) # remove previous end
            # loop through visit list and remove all nodes visited on recent path
            for node in path:
                node_bad = False
                for a, b in dests:
                    if node == a:
                        node_bad = True
                        break
                if node_bad:
                    pri = find_priority(dests, node)
                    if p != pri: # If visited node is a destination we need to visit and has a lower priority value than we were looking for, inc number of violations
                        priority_violations += 1
                    dests.remove([node, pri])
                # Update total route
                route.append(node)
        prev_p = p
            
    return total_dist, route, priority_violations
    


def astar(graph: Graph, start: int, end: int) -> Tuple[Dict[int, float], Dict[int, Optional[int]], int]:
    """
    A* algorithm for shortest path
    Returns: (distances, previous nodes, nodes explored)
    """
    dist = {node_id: float('inf') for node_id in graph.nodes}
    prev = {node_id: None for node_id in graph.nodes}
    dist[start] = 0
    
    end_node = graph.nodes[end]
    
    def heuristic(node_id: int) -> float:
        node = graph.nodes[node_id]
        return haversine(node.lat, node.lon, end_node.lat, end_node.lon)
    
    pq = [(heuristic(start), start)]
    nodes_explored = 0
    visited = set()
    
    while pq:
        _, u = heapq.heappop(pq)
        
        if u in visited:
            continue
        
        visited.add(u)
        nodes_explored += 1
        
        if u == end:
            break
        
        for edge in graph.adj_list.get(u, []):
            v = edge.to
            alt = dist[u] + edge.weight
            
            if alt < dist[v]:
                dist[v] = alt
                prev[v] = u
                f_score = alt + heuristic(v)
                heapq.heappush(pq, (f_score, v))
    
    return dist, prev, nodes_explored


def bellman_ford(graph: Graph, start: int, end: int) -> Tuple[Optional[Dict[int, float]], Optional[Dict[int, Optional[int]]], int]:
    """
    Bellman-Ford algorithm for shortest path
    Can handle negative weights and detect negative cycles
    Returns: (distances, previous nodes, nodes explored) or (None, None, 0) if negative cycle detected
    """
    dist = {node_id: float('inf') for node_id in graph.nodes}
    prev = {node_id: None for node_id in graph.nodes}
    dist[start] = 0
    
    nodes_explored = 0
    node_count = len(graph.nodes)
    
    # Relax edges |V| - 1 times
    for i in range(node_count - 1):
        updated = False
        for u in graph.nodes:
            if dist[u] == float('inf'):
                continue
            
            for edge in graph.adj_list.get(u, []):
                v = edge.to
                if dist[u] + edge.weight < dist[v]:
                    dist[v] = dist[u] + edge.weight
                    prev[v] = u
                    updated = True
        
        nodes_explored += 1
        if not updated:
            break
    
    # Check for negative cycles
    for u in graph.nodes:
        if dist[u] == float('inf'):
            continue
        
        for edge in graph.adj_list.get(u, []):
            v = edge.to
            if dist[u] + edge.weight < dist[v]:
                return None, None, 0  # Negative cycle detected
    
    return dist, prev, nodes_explored


def reconstruct_path(prev: Dict[int, Optional[int]], start: int, end: int) -> Optional[List[int]]:
    """Reconstruct path from start to end using previous nodes"""
    if prev[end] is None and start != end:
        return None
    
    path = []
    current = end
    while current is not None:
        path.append(current)
        current = prev[current]
    
    path.reverse()
    return path


def print_path(route: List, start: int, distance: float, priority_violations: int, threshold: float):
    """Print the path from start to end"""
        
    if route is None:
        print("No path found")
        return
    
    path_str = " -> ".join(str(node) for node in route)
    print(f"Path from {start}: {path_str}")
    print(f"Total distance: {distance:.2f} km")
    print(f"Threshold: {threshold}")
    print(f"Number of priority violations: {priority_violations}")


def load_graph(nodes_file: str, edges_file: str) -> Graph:
    """Load graph from CSV files"""
    graph = Graph()
    
    # Load nodes
    with open(nodes_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            node_id = int(row['id'])
            lat = float(row['lat'])
            lon = float(row['lon'])
            graph.add_node(node_id, lat, lon)
    
    # Load edges
    with open(edges_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            from_id = int(row['from'])
            to_id = int(row['to'])
            distance = float(row['distance'])
            graph.add_edge(from_id, to_id, distance)
    
    return graph


def main():
    if len(sys.argv) != 7:
        print(f"Usage: {sys.argv[0]} <nodes.csv> <edges.csv> <start_node> <destinations.csv> <threshold> <algorithm>")
        print("Algorithms: dijkstra, astar, bellman-ford")
        sys.exit(1)

    nodes_file = sys.argv[1]
    edges_file = sys.argv[2]
    start_node = int(sys.argv[3])
    destinations_file = sys.argv[4]
    threshold = float(sys.argv[5])
    algorithm = sys.argv[6]
        
    # Load graph
    graph = load_graph(nodes_file, edges_file)

    # Load destinations
    destinations = []

    with open(destinations_file, "r") as f:
        reader = csv.reader(f)
        next(reader)  # skip header row
        for row in reader:
            destinations.append([int(row[0]), row[1]])
    
    # Validate nodes
    if start_node not in graph.nodes:
        print("Invalid start or end node")
        sys.exit(1)
    
    # Run selected algorithm
    if algorithm == "dijkstra":
        print("\n=== Dijkstra's Algorithm ===")
        total_dist, route, priority_violations = dijkstra(graph, start_node, destinations, threshold)
    else:
        print(f"Unknown algorithm: {algorithm}")
        print("Available algorithms: dijkstra, astar, bellman-ford")
        sys.exit(1)
        
    # Print results
    print_path(route, start_node, total_dist, priority_violations, threshold)



if __name__ == "__main__":
    main()
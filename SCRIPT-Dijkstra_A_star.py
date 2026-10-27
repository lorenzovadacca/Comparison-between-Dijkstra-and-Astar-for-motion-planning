import osmnx as ox
import random
import heapq
import math
import matplotlib.pyplot as plt

def style_unvisited_edge(edge):        
    G.edges[edge]["color"] = "#d36206"
    G.edges[edge]["alpha"] = 1
    G.edges[edge]["linewidth"] = 0.2

def style_visited_edge(edge):
    G.edges[edge]["color"] = "green"
    G.edges[edge]["alpha"] = 1
    G.edges[edge]["linewidth"] = 1

def style_active_edge(edge):
    G.edges[edge]["color"] = "red"
    G.edges[edge]["alpha"] = 1
    G.edges[edge]["linewidth"] = 1

def style_path_edge(edge):
    G.edges[edge]["color"] = "white"
    G.edges[edge]["alpha"] = 1
    G.edges[edge]["linewidth"] = 5

def plot_graph():
    ox.plot_graph(
        G,
        node_size =  [ G.nodes[node]["size"] for node in G.nodes ],
        edge_color = [ G.edges[edge]["color"] for edge in G.edges ],
        edge_alpha = [ G.edges[edge]["alpha"] for edge in G.edges ],
        edge_linewidth = [ G.edges[edge]["linewidth"] for edge in G.edges ],
        node_color = "white",
        bgcolor = "#18080e"
    )

def heuristic(node1, node2):
    y1, x1 = G.nodes[node1]['y'], G.nodes[node1]['x']
    y2, x2 = G.nodes[node2]['y'], G.nodes[node2]['x']
    linear_distance = math.sqrt((x1 - x2)**2 + (y1 - y2)**2) # [linear_distance] = meters
    average_speed = 50/3.6 # [average_speed] = m/s
    time_travel = linear_distance / average_speed # [time_travel] = seconds
    return time_travel

def dijkstra(orig, dest, plot=False):
    for node in G.nodes:
        G.nodes[node]["visited"] = False
        G.nodes[node]["distance"] = float("inf")
        G.nodes[node]["previous"] = None
        G.nodes[node]["size"] = 0
    for edge in G.edges:
        style_unvisited_edge(edge)
    G.nodes[orig]["distance"] = 0
    G.nodes[orig]["size"] = 50
    G.nodes[dest]["size"] = 50
    pq = [(0, orig)]
    step = 0
    while pq:
        _, node = heapq.heappop(pq)
        if node == dest:
            print("Iterations:", step)
            return
        if G.nodes[node]["visited"]: continue
        G.nodes[node]["visited"] = True
        for edge in G.out_edges(node):
            style_visited_edge((edge[0], edge[1], 0))
            neighbor = edge[1]
            weight = G.edges[(edge[0], edge[1], 0)]["weight"]
            if G.nodes[neighbor]["distance"] > G.nodes[node]["distance"] + weight:
                G.nodes[neighbor]["distance"] = G.nodes[node]["distance"] + weight
                G.nodes[neighbor]["previous"] = node
                heapq.heappush(pq, (G.nodes[neighbor]["distance"], neighbor))
                for edge2 in G.out_edges(neighbor):
                    style_active_edge((edge2[0], edge2[1], 0))
        step += 1

def a_star(orig, dest, maxs, plot=False):
    for node in G.nodes:
        G.nodes[node]["visited"] = False
        G.nodes[node]["distance"] = float("inf")
        G.nodes[node]["f_score"] = float("inf")
        G.nodes[node]["previous"] = None
        G.nodes[node]["size"] = 0
    for edge in G.edges:
        style_unvisited_edge(edge)
    G.nodes[orig]["distance"] = 0
    G.nodes[orig]["f_score"] = heuristic(orig, dest)
    G.nodes[orig]["size"] = 50
    G.nodes[dest]["size"] = 50

    open_set = [(G.nodes[orig]["f_score"], orig)]
    step = 0
    while open_set:
        _, node = heapq.heappop(open_set)
        if node == dest:
            print("Iterations:", step)
            return
        if G.nodes[node]["visited"]: continue
        G.nodes[node]["visited"] = True
        for edge in G.out_edges(node):
            style_visited_edge((edge[0], edge[1], 0))
            neighbor = edge[1]
            weight = G.edges[(edge[0], edge[1], 0)]["weight"]
            tentative_g_score = G.nodes[node]["distance"] + weight
            if tentative_g_score < G.nodes[neighbor]["distance"]:
                G.nodes[neighbor]["previous"] = node
                G.nodes[neighbor]["distance"] = tentative_g_score
                G.nodes[neighbor]["f_score"] = tentative_g_score + heuristic(neighbor, dest)
                heapq.heappush(open_set, (G.nodes[neighbor]["f_score"], neighbor))
                for edge2 in G.out_edges(neighbor):
                    style_active_edge((edge2[0], edge2[1], 0))
        step += 1
        
def reconstruct_path(orig, dest, plot=False, algorithm=None):
    for edge in G.edges:
        style_unvisited_edge(edge)
    dist = 0
    speeds = []
    curr = dest
    while curr != orig:
        prev = G.nodes[curr]["previous"]
        dist += G.edges[(prev, curr, 0)]["length"]
        speeds.append(G.edges[(prev, curr, 0)]["maxspeed"])
        style_path_edge((prev, curr, 0))
        if algorithm:
            G.edges[(prev, curr, 0)][f"{algorithm}_uses"] = G.edges[(prev, curr, 0)].get(f"{algorithm}_uses", 0) + 1
        curr = prev
    dist /= 1000

def plot_heatmap(algorithm, ax):
    edge_colors = ox.plot.get_edge_colors_by_attr(G, f"{algorithm}_uses", cmap="hot")
    fig, _ = ox.plot_graph(
        G,
        ax=ax,
        show=False,
        close=False,
        node_size = [5 if G.nodes[node]["visited"] else 0 for node in G.nodes],
        node_color = ["green" if G.nodes[node]["visited"] else "none" for node in G.nodes],
        edge_color = [G.edges[edge]["color"] for edge in G.edges],
        edge_alpha = [G.edges[edge]["alpha"] for edge in G.edges],
        edge_linewidth = [G.edges[edge]["linewidth"] for edge in G.edges],
        bgcolor = "#18080e"
    )
    return fig

place_name = "Turin, Piedmont, Italy"
G = ox.graph_from_place(place_name, network_type="drive")
G = ox.project_graph(G)

for edge in G.edges:
    # Cleaning the "maxspeed" attribute, some values are lists, some are strings, some are None
    maxspeed = 50
    if "maxspeed" in G.edges[edge]:
        maxspeed = G.edges[edge]["maxspeed"]
        if type(maxspeed) == list:
            speeds = [ int(speed.strip(" mph")) for speed in maxspeed ]
            maxspeed = min(speeds)
        elif type(maxspeed) == str:
            maxspeed = maxspeed.strip(" mph") # [maxspeed] = km/h
    maxspeed = int(maxspeed)/3.6  # [maxspeed] = m/s
    G.edges[edge]["maxspeed"] = maxspeed
    # Adding the "weight" attribute (time = distance / speed)
    G.edges[edge]["weight"] = G.edges[edge]["length"] / maxspeed


for edge in G.edges:
    G.edges[edge]["dijkstra_uses"] = 0
    G.edges[edge]["a_star_uses"] = 0

start = random.choice(list(G.nodes))     #255281793
print("Origin ID:",start)
end = random.choice(list(G.nodes))       #728492344
print("Destination ID:",end)

fig, axes = plt.subplots(1, 2, figsize=(16, 9))

print("Running Dijkstra")
dijkstra(start, end)
reconstruct_path(start, end, algorithm="dijkstra", plot=True)
plot_heatmap("dijkstra", ax=axes[0])
axes[0].set_facecolor("#18080e")
axes[0].set_title("Dijkstra")

print("Running A*")
a_star(start, end, maxspeed)
reconstruct_path(start, end, algorithm="a_star", plot=True)
plot_heatmap("a_star", ax=axes[1])
axes[1].set_title("A*")
axes[1].set_facecolor("#18080e")

plt.tight_layout()
plt.show()
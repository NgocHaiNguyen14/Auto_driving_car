import osmnx as ox
import networkx as nx
import plotly.graph_objects as go
import numpy as np
import os
import geocoder
import matplotlib.pyplot as plt
import requests

def geocode_location(location):
    url = 'https://nominatim.openstreetmap.org/search'
    params = {
        'q': location,
        'format': 'json',
        'addressdetails': 1,
        'limit': 1
    }
    headers = {
        'User-Agent': 'autonomous-vehicle-mapping/1.0'
    }
    
    response = requests.get(url, params=params, headers=headers)
    
    if response.status_code == 200:
        data = response.json()
        if data:
            return float(data[0]['lat']), float(data[0]['lon'])
        else:
            return None
    else:
        raise Exception(f"Request failed with status code: {response.status_code}")

def node_list_to_path(G, node_list):
    edge_nodes = list(zip(node_list[:-1], node_list[1:]))
    lines = []
    for u, v in edge_nodes:
        # if there are parallel edges, select the shortest in length
        data = min(G.get_edge_data(u, v).values(), 
                   key=lambda x: x['length'])
        # if it has a geometry attribute
        if 'geometry' in data:
            # add them to the list of lines to plot
            xs, ys = data['geometry'].xy
            lines.append(list(zip(xs, ys)))
        else:
            # if it doesn't have a geometry attribute,
            # then the edge is a straight line from node to node
            x1 = G.nodes[u]['x']
            y1 = G.nodes[u]['y']
            x2 = G.nodes[v]['x']
            y2 = G.nodes[v]['y']
            line = [(x1, y1), (x2, y2)]
            lines.append(line)
    return lines

def plot_path(lat, long, origin_point, destination_point):

    # adding the lines joining the nodes
    fig = go.Figure(go.Scattermapbox(
        name = "Path",
        mode = "lines",
        lon = long,
        lat = lat,
        marker = {'size': 10},
        line = dict(width = 4.5, color = 'blue')))
    # adding source marker
    fig.add_trace(go.Scattermapbox(
        name = "Source",
        mode = "markers",
        lon = [origin_point[1]],
        lat = [origin_point[0]],
        marker = {'size': 12, 'color':"red"}))
     
    # adding destination marker
    fig.add_trace(go.Scattermapbox(
        name = "Destination",
        mode = "markers",
        lon = [destination_point[1]],
        lat = [destination_point[0]],
        marker = {'size': 12, 'color':'green'}))
    
    # getting center for plots:
    lat_center = np.mean(lat)
    long_center = np.mean(long)
    # defining the layout using mapbox_style
    fig.update_layout(mapbox_style="open-street-map",
                  mapbox_center_lat=21, mapbox_center_lon=105)

    fig.update_layout(margin={"r":0,"t":0,"l":0,"b":0},
                      mapbox = {
                          'center': {'lat': lat_center, 
                          'lon': long_center},
                          'zoom': 13})
    fig.show(config={"displayModeBar": False, "scrollZoom": False}, auto_open=False)


graph_file_path = "D:\Desktop\Projects\VAST\Map\hanoi_graph.graphml"

print("Checking if graph file exists...")
if os.path.exists(graph_file_path):
    print("File exists, loading the graph...")
    G = ox.load_graphml(filepath=graph_file_path)
    print("Graph loaded, drawing now...")
    ox.plot_graph(G, node_size=20, node_color='white', bgcolor='black')
    plt.show()
else:
    print("File doesn't exist, downloading map...")
    north, east, south, west = 21.2663, 105.9398, 20.7972, 105.3355
    G = ox.graph_from_bbox(north, south, east, west, network_type='drive')
    print("Map downloaded, saving to file...")
    ox.save_graphml(G, filepath=graph_file_path)
    print("Graph saved.")


# # Plotting the map graph 
# ox.plot_graph(G)
print("Done")

org_name = input('Điểm đi:')
origin_point = geocode_location(org_name)
print(origin_point)

dest_name = input('Điểm đến:')
destination_point = geocode_location(dest_name)
print(destination_point)

# get the nearest nodes to the locations 
origin_node = ox.nearest_nodes(G, origin_point[1], origin_point[0])
destination_node = ox.nearest_nodes(G, destination_point[1], destination_point[0])
# printing the closest node id to origin and destination points origin_node, destination_node

# Finding the optimal path 
route = nx.shortest_path(G, origin_node, destination_node, weight = 'length')

# getting the list of coordinates from the path 
# (which is a list of nodes)
lines = node_list_to_path(G, route)
long = []
lat = []
for i in range(len(lines)):
    z = list(lines[i])
    l1 = list(list(zip(*z))[0])
    l2 = list(list(zip(*z))[1])
    for j in range(len(l1)):
        long.append(l1[j])
        lat.append(l2[j])

plot_path(lat, long, origin_point, destination_point)
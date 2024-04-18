from PySide6.QtWidgets import QPushButton, QGridLayout, QWidget, QLabel, QLineEdit
import osmnx as ox
import networkx as nx
import plotly.graph_objects as go
import numpy as np
import os
import geocoder

class Map(QWidget):
    def __init__(self):
        super().__init__()
        graph_file_path = "D:/Desktop/Map/hanoi_graph.graphml"

        G = create_or_load_graph(graph_file_path)

        # # Plotting the map graph 
        # ox.plot_graph(G)

        org_label = QLabel('Điểm đi:')
        org_name = QLineEdit()

        dest_label = QLabel('Điểm đến:')
        dest_name = QLineEdit()

        grid_layout = QGridLayout()
        grid_layout.addWidget(org_label, 0, 0, 1, 1)
        grid_layout.addWidget(org_name, 0, 1, 1, 1)
        grid_layout.addWidget(dest_label, 1, 0, 1, 1)
        grid_layout.addWidget(dest_name, 1, 1, 1, 1)

        geocode_button = QPushButton('Tìm đường')
        geocode_button.clicked.connect(lambda:geocode_and_plot_path(G, org_name.text(), dest_name.text()))
        grid_layout.addWidget(geocode_button, 2, 0, 1, 2)

        self.setLayout(grid_layout)

def create_or_load_graph(graph_file_path):
        
        if os.path.exists(graph_file_path):
            # Load the graph from the GraphML file
            return ox.load_graphml(filepath=graph_file_path)
        else:
            # Defining the map boundaries 
            north, east, south, west = 21.2663, 105.9398, 20.7972, 105.3355
            
            # Download the map as a graph object 
            G = ox.graph_from_bbox(north, south, east, west, network_type='drive')

            # Save the graph to a GraphML file
            ox.save_graphml(G, filepath=graph_file_path)
            return G
    
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

def plot_path(lat, long, origin_point, destination_point, initial_zoom):
    # adding the lines joining the nodes
    fig = go.Figure(go.Scattermapbox(
        name="Path",
        mode="lines",
        lon=long,
        lat=lat,
        marker={'size': 10},
        line=dict(width=4.5, color='blue')))

    # adding source marker
    fig.add_trace(go.Scattermapbox(
        name="Source",
        mode="markers",
        lon=[origin_point[1]],
        lat=[origin_point[0]],
        marker={'size': 12, 'color': "red"}))

    # adding destination marker
    fig.add_trace(go.Scattermapbox(
        name="Destination",
        mode="markers",
        lon=[destination_point[1]],
        lat=[destination_point[0]],
        marker={'size': 12, 'color': 'green'}))

    # getting center for plots:
    lat_center = np.mean(lat)
    long_center = np.mean(long)

    # defining the layout using mapbox_style
    fig.update_layout(
        mapbox_style="open-street-map",
        mapbox_center_lat=lat_center,
        mapbox_center_lon=long_center,
        mapbox=dict(
            center=dict(lat=lat_center, lon=long_center),
            zoom=initial_zoom,  # Initial zoom level
        )
    )

    # # Add a slider for controlling zoom
    # sliders = [dict(
    #     active=initial_zoom,
    #     steps=[dict(label=str(i), method="relayout", args=[{"mapbox.zoom": i}])
    #            for i in range(1, 21)]  # Adjust the range as needed
    # )]

    # fig.update_layout(sliders=sliders)

    fig.update_layout(margin={"r": 0, "t": 0, "l": 0, "b": 0})

    # Save the figure to an HTML file
    fig.write_html("D:/Desktop/Qt GUI/path_map.html", config={"displayModeBar": True, "scrollZoom": True})

    # fig.show(config={"displayModeBar": False, "scrollZoom": False}, auto_open=False)
        
def geocode_and_plot_path(G, org_name, dest_name):
    origin_point = geocoder.osm(org_name).latlng
    destination_point = geocoder.osm(dest_name).latlng

    if origin_point is None or destination_point is None:
        # Handle the case when geocoding fails
        print("Error: Unable to geocode the origin or destination.")
    
    else:
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
        with open("D:/Desktop/coordinates.txt", "w") as file:
            file.write("Longitude, Latitude\n")
            for i in range(len(long)):
                file.write(f"{long[i]}, {lat[i]}\n")

        plot_path(lat, long, origin_point, destination_point, 15) 
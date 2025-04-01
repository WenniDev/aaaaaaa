from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import json
import sys
import osmnx as ox
from pyrosm import OSM
import time
import folium

def _create_data_model(filename):
    with open(filename) as f:
        data = json.load(f)

    # Extraction des données
    data['liste_lat_long'] = [(point['latitude'], point['longitude']) for point in data['point_de_vente']]
    data['service_times'] = [point['temps_de_service'] for point in data['point_de_vente']]
    data['time_windows'] = [point['horaires_d_ouverture'] for point in data['point_de_vente']]
    data['ids'] = [point['id'] for point in data['point_de_vente']]
    data['depot'] = 0
    data['vehicle_time_windows'] = data['point_de_vente'][0]['horaires_d_ouverture']
    data['num_vehicles'] = 1
    data['vehicle_max_times'] = [1000] * data['num_vehicles']
    data['slack_time'] = 0  # Temps d'attente autorisé

    return data

def travel_time_matrix(data):
    osm_place = 'paris'
    osm = OSM(osm_place + '.osm.pbf')

    # Load the OSM data
    G = None
    start_time = time.time()

    # if not os.path.exists(osm_place + '.graphml'):
    print("Graph not found, downloading and loading...")
    nodes, edges = osm.get_network(network_type='driving', nodes=True)
    G = osm.to_graph(nodes, edges, graph_type="networkx")
    ox.save_graphml(G, osm_place + ".graphml")
    # else:
    #    print("Graph already exists, loading from file...")
    G = ox.load_graphml(osm_place + ".graphml")

    print(f"OSM data loaded in {time.time() - start_time:.2f} seconds")

    G = ox.add_edge_speeds(G)
    G = ox.routing.add_edge_travel_times(G)

    orig_coords = [(p[0], p[1]) for p in data['liste_lat_long'] for _ in data['liste_lat_long']]
    dest_coords = [(p[0], p[1]) for _ in data['liste_lat_long'] for p in data['liste_lat_long']]


    orig = ox.nearest_nodes(G, X=[coord[1] for coord in orig_coords], Y=[coord[0] for coord in orig_coords])
    dest = ox.nearest_nodes(G, X=[coord[1] for coord in dest_coords], Y=[coord[0] for coord in dest_coords])

    shortest_routes = ox.routing.shortest_path(G, orig, dest, weight="travel_time", cpus=1)
    
    num_points = len(data['liste_lat_long'])
    travel_time_matrix = []
    polylines = []

    ligne = []
    polyline_ligne = []
    for elt in shortest_routes:
        if len(ligne) == num_points:
            travel_time_matrix.append(ligne)
            polylines.append(polyline_ligne)
            ligne = []
            polyline_ligne = []
        if len(elt) == 1:
            ligne.append(0)
            polyline_ligne.append([])
        else:
            route_gdf = ox.routing.route_to_gdf(G, elt, weight="length")
            ligne.append(int(sum(route_gdf["travel_time"])))
            polyline_ligne.append([list(line.coords) for line in route_gdf["geometry"]])
    
    travel_time_matrix.append(ligne)
    polylines.append(polyline_ligne)
    
    for i in range(len(travel_time_matrix)):
        for j in range(len(travel_time_matrix[i])):
            travel_time_matrix[i][j] = travel_time_matrix[i][j] // 5  

    data['weights'] = travel_time_matrix
    data['polylines'] = polylines

def _print_solution(data, manager, routing, solution):
    total_time = 0
    map_folium = folium.Map(location=[data['liste_lat_long'][0][0], data['liste_lat_long'][0][1]], zoom_start=12)
    
    colors = ["red", "blue", "green", "purple", "orange", "darkred", "lightred", "beige", "darkblue", "darkgreen"]

    for i, (lat, lon) in enumerate(data['liste_lat_long']):
        folium.Marker(
            location=[lat, lon],
            popup=f"Node {i}\nID: {data['ids'][i]}\nTime window: {data['time_windows'][i]}",
            icon=folium.Icon(color="blue", icon="info-sign")
        ).add_to(map_folium)

    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        nodes = []
        while not routing.IsEnd(index):
            nodes.append(manager.IndexToNode(index))
            index = solution.Value(routing.NextVar(index))

        for j in range(len(nodes) - 1):
            if nodes[j] < len(data['polylines']) and nodes[j + 1] < len(data['polylines'][nodes[j]]):
                coords = [[lat, lon] for lon, lat in data['polylines'][nodes[j]][nodes[j + 1]][0]]
                folium.PolyLine(coords, color=colors[vehicle_id % len(colors)], weight=5, opacity=0.7).add_to(map_folium)

        print(f"Route for vehicle {vehicle_id}: {nodes}")

    map_folium.save("./routes_map.html")
    print("Map saved as routes_map.html")

def main():
    data = _create_data_model(sys.argv[1])
    travel_time_matrix(data)

    manager = pywrapcp.RoutingIndexManager(len(data['weights']), data['num_vehicles'], data['depot'])
    routing = pywrapcp.RoutingModel(manager)

    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['weights'][from_node][to_node] + data['service_times'][from_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    routing.AddDimension(
        transit_callback_index,
        data['slack_time'],
        1800000,
        False,
        'Time'
    )

    time_dimension = routing.GetDimensionOrDie('Time')
    for location_idx, time_window in enumerate(data['time_windows']):
        if location_idx != data['depot']:
            index = manager.NodeToIndex(location_idx)
            time_dimension.CumulVar(index).SetRange(time_window[0][0], time_window[len(time_window)-1][1])

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_MOST_CONSTRAINED_ARC)

    solution = routing.SolveWithParameters(search_parameters)

    if solution:
        _print_solution(data, manager, routing, solution)
    else:
        print('No solution found')

if __name__ == '__main__':
    main()

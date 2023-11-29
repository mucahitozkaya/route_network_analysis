from __future__ import print_function
import osmnx as ox
import networkx as nx
import matplotlib.pyplot as plt
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

ox.settings.log_console=True
ox.settings.use_cache=True


# Müşteri noktalarınızın koordinatları (örnek veri)
customer_locations = {
    0: (40.730179, 30.393807),
    1: (40.744617, 30.408530),
    2: (40.743304, 30.395972),
    3: (40.741992, 30.381248),
    4: (40.752818, 30.371721),
    5: (40.754677, 30.389476),
    6: (40.755552, 30.407520),
    7: (40.728538, 30.369989),
    8: (40.719677, 30.374319),
    9: (40.774357, 30.401890),
    # Diğer müşteri noktalarını ekleyin...
}


def only_distance():
    

    # location where you want to find your route
    place = ['Sakarya, Turkey']
    # find shortest route based on the mode of travel
    mode = 'drive'        # 'drive', 'bike', 'walk'
    # find shortest path based on distance or time
    optimizer = 'length'        # 'length','time'

    graph = ox.graph_from_place(place, network_type = mode)

    # ox.save_graphml(graph, filepath='turkey.graphml')

    # graph = ox.load_graphml(filepath='turkey.graphml')

    data = {}
    data['distance_matrix'] = []

    for i in range(len(customer_locations)):
        row = []
        start_latlng = customer_locations[i]
        
        for j in customer_locations:
            end_latlng = customer_locations[j]

            orig_node = ox.distance.nearest_nodes(graph, start_latlng[1],start_latlng[0])

            dest_node = ox.distance.nearest_nodes(graph, end_latlng[1],end_latlng[0])

            shortest_route = nx.shortest_path(graph,orig_node,dest_node,weight=optimizer)

            total_distance = int(sum(ox.utils_graph.get_route_edge_attributes(graph, shortest_route, 'length')))
            row.append(total_distance)
            print("Toplam mesafe:", total_distance, "metre")
        data['distance_matrix'].append(row)

    data['num_vehicles'] = 1
    data['depot'] = 0
    print(data)
    return data


def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    max_route_distance = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_distance = 0
        while not routing.IsEnd(index):
            plan_output += ' {} -> '.format(manager.IndexToNode(index))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
        plan_output += '{}\n'.format(manager.IndexToNode(index))
        plan_output += 'Distance of the route: {}m\n'.format(route_distance)
        print(plan_output)
        max_route_distance = max(route_distance, max_route_distance)
    print('Maximum of the route distances: {}m'.format(max_route_distance))

def VRP():
    """Solve the CVRP problem."""
    # Instantiate the data problem.
    data = only_distance()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)


    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance constraint.
    dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        300000,  # vehicle maximum travel distance
        True,  # start cumul to zero
        dimension_name)
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(data, manager, routing, solution)

VRP()

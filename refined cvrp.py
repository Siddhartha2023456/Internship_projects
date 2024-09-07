import json
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import math

def transform_json_to_dict(file_path):
    with open(file_path, 'r') as file:
        json_data = json.load(file)
    
    # Create a mapping from loc_ids like "loc0" to indices 0, 1, 2, ...
    loc_id_to_index = {loc_id: index for index, loc_id in enumerate(json_data["loc_ids"])}
    
    # Set the depot as index 0 (assuming "loc0" is the depot)
    depot = loc_id_to_index.get("loc0", 0)  # Default to 0 if "loc0" isn't found
    
    # Replace "location_matrix" values (e.g., "loc1", "loc2") with their corresponding indices
    json_data["location_matrix"] = [loc_id_to_index[loc] for loc in json_data["location_matrix"]]

    # Add depot index to the dictionary
    json_data["depot"] = depot
    
    return json_data


file_path = 'C:/Users/Acer/Downloads/assignment_cvrp.json'
data = transform_json_to_dict(file_path)

def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    print(f"Objective: {solution.ObjectiveValue()}")
    total_distance = 0
    total_volume = 0
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        plan_output = f"Route for vehicle {vehicle_id}:\n"
        route_distance = 0
        route_load = 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            route_load += data["volume_matrix"][node_index]
            plan_output += f" {node_index} Volume({route_load}) -> "
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id
            )
        plan_output += f" {manager.IndexToNode(index)} Volume({route_load})\n"
        plan_output += f"Distance of the route: {route_distance}m\n"
        plan_output += f"Volume of the route: {route_load}\n"
        print(plan_output)
        total_distance += route_distance
        total_volume += route_load
    print(f"Total distance of all routes: {total_distance}m")
    print(f"Total volume of all routes: {total_volume}")

# Initialize routing manager and model
manager = pywrapcp.RoutingIndexManager(
    len(data["distance"]), data["num_vehicles"], data["depot"]
)
routing = pywrapcp.RoutingModel(manager)

# Vehicle cost callback
def vehicle_cost_callback(vehicle_id, from_index, to_index):
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    from_loc = data["location_matrix"][from_node]
    to_loc = data["location_matrix"][to_node]
    distance = math.ceil(data["distance"][from_node][to_node])
    hop_cost_val = 0
    hop_cost = 100  # Fixed hop cost

    # Apply hop cost if not starting from depot and moving between different locations
    if from_node != 0 and from_loc != to_loc:
        hop_cost_val += hop_cost
    
    return int(data["perKmCostPerVehicle"][vehicle_id] * distance) + hop_cost_val

# Register the vehicle cost callback and set vehicle costs
for vehicle_id in range(data["num_vehicles"]):
    vehicle_cost_callback_index = routing.RegisterTransitCallback(
        lambda from_index, to_index, vehicle_id=vehicle_id: 
        vehicle_cost_callback(vehicle_id, from_index, to_index)
    )
    routing.SetArcCostEvaluatorOfVehicle(vehicle_cost_callback_index, vehicle_id)
    vehicle_fixed_cost = math.ceil(data["fixedCostPerVehicle"][vehicle_id])
    routing.SetFixedCostOfVehicle(vehicle_fixed_cost, vehicle_id)

# Add Weight Capacity constraint.
def weight_callback(from_index):
    from_node = manager.IndexToNode(from_index)
    return data["weight_matrix"][from_node]

weight_callback_index = routing.RegisterUnaryTransitCallback(weight_callback)
routing.AddDimensionWithVehicleCapacity(
    weight_callback_index,
    0,  # null capacity slack
    data["max_weight"],  # vehicle maximum capacities
    True,  # start cumul to zero
    "Weight_Capacity",
)

# Add Volume Capacity constraint.
def volume_callback(from_index):
    from_node = manager.IndexToNode(from_index)
    return data["volume_matrix"][from_node]

volume_callback_index = routing.RegisterUnaryTransitCallback(volume_callback)
routing.AddDimensionWithVehicleCapacity(
    volume_callback_index,
    0,  # null capacity slack
    data["max_volume"],  # vehicle maximum capacities
    True,  # start cumul to zero
    "Volume_Capacity",
)

# Setting search parameters
search_parameters = pywrapcp.DefaultRoutingSearchParameters()
search_parameters.first_solution_strategy = (
    routing_enums_pb2.FirstSolutionStrategy.SAVINGS
)
search_parameters.local_search_metaheuristic = (
    routing_enums_pb2.LocalSearchMetaheuristic.SIMULATED_ANNEALING
)
search_parameters.time_limit.FromSeconds(1)

# Solve the problem
solution = routing.SolveWithParameters(search_parameters)

# Print solution on console
if solution:
    print_solution(data, manager, routing, solution)

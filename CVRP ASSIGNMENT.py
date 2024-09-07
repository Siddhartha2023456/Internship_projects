import json
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import math
from collections import defaultdict


def transform_json_to_dict(file_path):
    with open(file_path, 'r') as file:
        json_data = json.load(file)
    
    # Create a mapping from loc_ids like "loc0" to indices 0, 1, 2, ...
    loc_id_to_index = {loc_id: index for index, loc_id in enumerate(json_data["loc_ids"])}
    
    # Set the depot as index 0 (assuming "loc0" is the depot)
    depot = loc_id_to_index.get("loc0", 0)  # Default to 0 if "loc0" isn't found
    
    # Aggregate the orders for each location
    aggregated_weight = defaultdict(int)
    aggregated_volume = defaultdict(int)
    
    for loc, weight, volume in zip(json_data["location_matrix"], json_data["weight_matrix"], json_data["volume_matrix"]):
        index = loc_id_to_index[loc]
        aggregated_weight[index] += weight*1000  # Convert weight to integer
        aggregated_volume[index] += int(volume)  # Convert volume to integer
    
    # Update the weight_matrix and volume_matrix with aggregated integer values
    json_data["weight_matrix"] = [int(aggregated_weight[index]) for index in range(len(loc_id_to_index))]
    json_data["volume_matrix"] = [int(aggregated_volume[index]) for index in range(len(loc_id_to_index))]

    # Replace "location_matrix" values (e.g., "loc1", "loc2") with their corresponding indices
    json_data["location_matrix"] = list(range(len(loc_id_to_index)))

    # Add depot index to the dictionary
    json_data["depot"] = int(depot)  # Convert depot index to integer
    
    # Convert other numeric values in the dictionary to integers
    if "distance" in json_data:
        json_data["distance"] = [[int(value) for value in row] for row in json_data["distance"]]
    
    if "perKmCostPerVehicle" in json_data:
        json_data["perKmCostPerVehicle"] = [int(value) for value in json_data["perKmCostPerVehicle"]]


    return json_data


file_path = 'C:/Users/Acer/Downloads/assignment_cvrp.json'
data = transform_json_to_dict(file_path)
def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    print(f"Objective: {solution.ObjectiveValue()}")
    total_distance = 0
    total_volume = 0
    total_weight = 0


    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        plan_output = f"Route for vehicle {vehicle_id}:\n"
        route_distance = 0
        route_volume = 0
        route_weight = 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            route_volume += data["volume_matrix"][node_index]
            route_weight += data["weight_matrix"][node_index]
            plan_output += (
                f" {node_index} "
                f"Volume({route_volume}) "
                f"Weight({route_weight}) -> "
            )
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id
            )
        plan_output += (
            f" {manager.IndexToNode(index)} "
            f"Volume({route_volume}) "
            f"Weight({route_weight})\n"
        )
        plan_output += f"Cost of the route: {route_distance}\n"
        plan_output += f"Volume of the route: {route_volume}\n"
        plan_output += f"Weight of the route: {route_weight}\n"
        print(plan_output)
        total_distance += route_distance
        total_volume += route_volume
        total_weight += route_weight

        
    print(f"Total cost of all routes: {total_distance}")
    print(f"Total volume of all routes: {total_volume}")
    print(f"Total weight of all routes: {total_weight}")


# Initialize routing manager and model
manager = pywrapcp.RoutingIndexManager(
    len(data["distance"]), data["num_vehicles"], data["depot"]
)
routing = pywrapcp.RoutingModel(manager)

# Vehicle cost callback
def vehicle_cost_callback(vehicle_id, from_index, to_index):
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    distance = math.ceil(data["distance"][from_node][to_node])
    hop_cost_val = 0
    hop_cost = 100  # Fixed hop cost

    if from_node != 0:
        hop_cost_val += hop_cost
    
    return int(data["perKmCostPerVehicle"][vehicle_id] * distance)

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
    routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
)
search_parameters.local_search_metaheuristic = (
    routing_enums_pb2.LocalSearchMetaheuristic.SIMULATED_ANNEALING
)
search_parameters.time_limit.FromSeconds(1)
search_parameters.log_search=False

# Solve the problem
solution = routing.SolveWithParameters(search_parameters)
                                       
if solution:
    print_solution(data, manager, routing, solution)

def solution_to_json(data, manager, routing, solution):
    """Converts solution to a JSON structure."""
    solution_dict = {
        "objective": solution.ObjectiveValue(),
        "routes": []
    }

    total_distance = 0
    total_volume = 0
    total_weight = 0

    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        route = {
            "vehicle_id": vehicle_id,
            "route": [],
            "route_cost": 0,
            "route_volume": 0,
            "route_weight": 0,
        }
        route_distance = 0
        route_volume = 0
        route_weight = 0

        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            route_volume += data["volume_matrix"][node_index]
            route_weight += data["weight_matrix"][node_index]
            route["route"].append({
                "location": node_index,
                "cumulative_volume": route_volume,
                "cumulative_weight": route_weight
            })
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)

      
        route["route"].append({
            "location": manager.IndexToNode(index),
            "cumulative_volume": route_volume,
            "cumulative_weight": route_weight
        })

        route["route_cost"] = route_distance
        route["route_volume"] = route_volume
        route["route_weight"] = route_weight

        solution_dict["routes"].append(route)

        total_distance += route_distance
        total_volume += route_volume
        total_weight += route_weight

    # Add totals to the solution
    solution_dict["total_cost"] = total_distance
    solution_dict["total_volume"] = total_volume
    solution_dict["total_weight"] = total_weight

    return solution_dict

if solution:
    solution_json = solution_to_json(data, manager, routing, solution)

    # Output solution to a JSON file
    output_file = 'cvrp_solution.json'
    with open(output_file, 'w') as json_output:
        json.dump(solution_json, json_output, indent=4)

    print(f"Solution saved to {output_file}")


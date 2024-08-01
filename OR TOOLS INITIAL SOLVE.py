import pandas as pd
from haversine import haversine
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def read_data(file_path):
    df = pd.read_csv(file_path)
    places = df['Place_Name'].unique().tolist()
    coordinates = list(zip(df['Latitude'], df['Longitude']))
    return places, coordinates

def calculate_distance_matrix(coordinates):
    n = len(coordinates)
    distance_matrix = [[0.0] * n for _ in range(n)]
    for i in range(n):
        for j in range(n):
            if i != j:
                distance_matrix[i][j] = haversine(coordinates[i], coordinates[j])
    return distance_matrix

data_file_path = 'C:/Users/Acer/Downloads/tsp_input_70.csv'
places, coordinates = read_data(data_file_path)

def create_data_model():
    """Stores the data for the problem."""
    data = {}
    data["distance_matrix"] = calculate_distance_matrix(coordinates)
    data["num_vehicles"] = 1
    data["depot"] = 0
    return data

def print_solution(manager, routing, solution, places):
    """Prints solution on console and saves to CSV."""
    print(f"Objective: {solution.ObjectiveValue()/1000} kms")
    index = routing.Start(0)
    plan_output = "Route for vehicle 0:\n"
    route_distance = 0
    sequence = []
    while not routing.IsEnd(index):
        place_index = manager.IndexToNode(index)
        sequence.append((place_index, places[place_index]))
        plan_output += f" {place_index} ->"
        previous_index = index
        index = solution.Value(routing.NextVar(previous_index))
        route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
    place_index = manager.IndexToNode(index)
    sequence.append((place_index, places[place_index]))
    plan_output += f" {place_index}\n"
    plan_output += f"Route distance: {route_distance / 1000.0} kms\n"  # Convert meters to miles
    print(plan_output)

    # Save to CSV
    df_solution = pd.DataFrame(sequence, columns=["sequence", "place_name"])
    df_solution.to_csv('tsp_solution_702.csv', index=False)
    print("Solution saved to tsp_solution.csv")

def main():
    """Entry point of the program."""
    # Instantiate the data problem.
    data = create_data_model()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(data["distance_matrix"]), data["num_vehicles"], data["depot"]
    )

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return int(data["distance_matrix"][from_node][to_node] * 1000) 

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(manager, routing, solution, places)

if __name__ == '__main__':
    main()


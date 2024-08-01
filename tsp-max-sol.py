import pandas as pd
from haversine import haversine
import pulp
from pulp import GLPK, GUROBI
import folium

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

def read_tsp_solution(file_path):
    df_solution = pd.read_csv(file_path)
    sequence_dict = df_solution.set_index('place_name')['sequence'].to_dict()
    return sequence_dict

def build_model(places, distance_matrix, sequence_dict, max_distance=1000):
    # instantiate the problem - Python PuLP model
    prob = pulp.LpProblem("TSP", pulp.LpMinimize)

    # ***************************************************
    #   Defining decision variables
    # ***************************************************
    x = {}  # Binary: x_i,j:= 1 if I am visiting city j after city i; otherwise 0
    for i in range(len(places)):
        for j in range(len(places)):
            if i != j and distance_matrix[i][j] <= max_distance:
                x[(i, j)] = pulp.LpVariable("x_" + str(i) + '_' + str(j), cat='Binary')
    s = {}  # Integer: s_i is the sequence number when we are visiting city i
    for i in range(len(places)):
        s[i] = pulp.LpVariable("s_" + str(i), cat='Integer', lowBound=0, upBound=len(places) - 1)
        place_name = places[i]
        if place_name in sequence_dict:
            s[i].setInitialValue(sequence_dict[place_name])

    # ********************************************
    # Objective
    # ********************************************
    # Minimize total travel distance
    obj_val = 0
    for i in range(len(places)):
        for j in range(len(places)):
            if (i != j) and ((i, j) in x):
                obj_val += pulp.lpSum(x[(i, j)] * distance_matrix[i][j]) 
    prob += obj_val

    # Constraint 1: Each city is left exactly once
    for i in range(len(places)):
        aux_sum = 0
        for j in range(len(places)):
            if (i != j) and ((i, j) in x):
                aux_sum += x[(i, j)]
        prob += aux_sum == 1, 'Outgoing_sum_' + str(i)

    # Constraint 2: Each city is entered exactly once
    for j in range(len(places)):
        aux_sum = 0
        for i in range(len(places)):
            if (i != j) and ((i, j) in x):
                aux_sum += x[(i, j)]
        prob += aux_sum == 1, 'Incoming_sum_' + str(j)

    # Sub-tour elimination constraint
    for i in range(1, len(places)):
        for j in range(1, len(places)):
            if (i != j) and ((i, j) in x):
                prob += s[j] >= 1 + s[i] - (len(places)) * (1-x[(i, j)]), 'sub_tour_' + str(i) + '_' + str(j)

    return prob, x

def solve_tsp(prob, x, places):
    # Solve the problem
    solver = 'GUROBI'  # Solver choice; 'CBC', 'GUROBI', 'GLPK'
    print('-' * 50)
    print('Optimization solver', solver, 'called')
    prob.writeLP("C:/Users/Acer/Downloads/tsp_1023.lp")
    if solver == 'CBC':
        prob.solve(pulp.PULP_CBC_CMD())
    elif solver == 'GUROBI':
        prob.solve(GUROBI(warmStart=True))
    elif solver == 'GLPK':
        prob.solve(GLPK())
    else:
        print(solver, ' not available')
        exit()
    print(f'Status: {pulp.LpStatus[prob.status]}')

    if pulp.LpStatus[prob.status] == 'Optimal':
        n = len(places)
        optimal_route = [0]
        i = 0
        while len(optimal_route) < n:
            for j in range(n):
                if i != j and (i, j) in x and pulp.value(x[i, j]) == 1:
                    optimal_route.append(j)
                    i = j
                    break
        optimal_route.append(0)

        total_distance = pulp.value(prob.objective)

        print("Optimal Route:", " -> ".join(places[i] for i in optimal_route))
        print("Total Distance:", total_distance)

        return [places[i] for i in optimal_route], total_distance
    else:
        print("No optimal solution found.")
        return None, None

def plot_route(optimal_route, coordinates, places):
    # Create a map centered around the first place
    start_coord = coordinates[places.index(optimal_route[0])]
    m = folium.Map(location=start_coord, zoom_start=6)

    # Add markers for each place with tooltips
    for idx in optimal_route:
        folium.Marker(
            location=coordinates[places.index(idx)],
            popup=coordinates[places.index(idx)],
            tooltip=coordinates[places.index(idx)]
        ).add_to(m)

    # Add lines to show the route
    route_coords = [coordinates[places.index(i)] for i in optimal_route]
    folium.PolyLine(locations=route_coords, color='blue').add_to(m)

    # Save the map
    m.save('C:/Users/Acer/Downloads/tsp_route_401.html')


data_file_path = 'C:/Users/Acer/Downloads/tsp_input.csv'
places, coordinates = read_data(data_file_path)
distance_matrix = calculate_distance_matrix(coordinates)
solution_file_path = 'C:/Users/Acer/Downloads/tsp_solution_1001.csv'
sequence_dict = read_tsp_solution(solution_file_path)
problem, x = build_model(places, distance_matrix, sequence_dict, max_distance=2000)
optimal_route, total_distance = solve_tsp(problem, x, places)
if optimal_route:
    plot_route(optimal_route, coordinates, places)

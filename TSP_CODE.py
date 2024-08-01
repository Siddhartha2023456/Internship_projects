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


def build_model(places, distance_matrix):
    # instantiate the problem - Python PuLP model
    prob = pulp.LpProblem("TSP", pulp.LpMinimize)

    # ***************************************************
    #   Defining decision variables
    # ***************************************************
    x = {}  # Binary: x_i,j:= 1 if I am visiting city j after city i; otherwise 0
    for i in range(len(places)):
        for j in range(len(places)):
            if i != j:
                x[(i, j)] = pulp.LpVariable("x_" + str(i) + '_' + str(j), cat='Binary')
    s = {}  # Integer: s_i is the sequence number when we are visiting city i
    for i in range(len(places)):
        s[i] = pulp.LpVariable("s_" + str(i), cat='Integer', lowBound=0, upBound=len(places) - 1)

    # ********************************************
    # Objective
    # ********************************************
    # Minimize total travel distance
    obj_val = 0
    for i in range(len(places)):
        for j in range(len(places)):
            if i != j:
                obj_val+=(x[(i, j)] * distance_matrix[i][j]) 
    prob += obj_val

    # Constraint 1
    for i in range(len(places)):
        aux_sum = 0
        for j in range(len(places)):
            if i != j:
                aux_sum += x[(i, j)]
        prob += aux_sum == 1, 'Outgoing_sum_' + str(i)

    # Constraint 2
    for j in range(len(places)):
        aux_sum = 0
        for i in range(len(places)):
            if i != j:
                aux_sum += x[(i, j)]
        prob += aux_sum == 1, 'Incoming_sum_' + str(j)

    # Sub-tour elimination constraint
    for i in range(1, len(places)):
        for j in range(1, len(places)):
            if i != j:
                prob += s[j] >= 1 + s[i] - (len(places)) * (1-x[(i, j)]), 'sub_tour_' + str(i) + '_' + str(j)

    return prob, x


def solve_tsp(prob, x, places):
    # Solve the problem
    solver = 'CBC'  # Solver choice; 'CBC', 'GUROBI', 'GLPK'
    print('-' * 50)
    print('Optimization solver', solver, 'called')
    # prob.writeLP("../output/tsp.lp")
    if solver == 'CBC':
        prob.solve(pulp.PULP_CBC_CMD())
    elif solver == 'GUROBI':
        prob.solve(GUROBI())
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
                if i!= j and pulp.value(x[i, j]) == 1:
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
    m.save('C:/Users/Acer/Downloads/tsp_route_10.html')


data_file_path = 'C:/Users/Acer/Downloads/tsp_input.csv'
places, coordinates = read_data(data_file_path)
distance_matrix = calculate_distance_matrix(coordinates)
problem, x = build_model(places, distance_matrix)
optimal_route, total_distance = solve_tsp(problem, x, places)
if optimal_route:
    plot_route(optimal_route, coordinates, places)

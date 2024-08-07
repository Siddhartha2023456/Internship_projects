import pandas as pd
import pulp
from pulp import GUROBI

# Load data
locations_df = pd.read_csv('C:/Users/Acer/Downloads/locations.csv')
order_list_df = pd.read_excel('C:/Users/Acer/Downloads/order_list_1.xlsx')
travel_matrix_df = pd.read_csv('C:/Users/Acer/Downloads/travel_matrix.csv')
trucks_df = pd.read_csv('C:/Users/Acer/Downloads/trucks.csv')

# Convert loading/unloading windows to minutes with explicit format
locations_df['start_minutes'] = pd.to_datetime(locations_df['location_loading_unloading_window_start'], format='%H:%M').dt.hour * 60 + pd.to_datetime(locations_df['location_loading_unloading_window_start'], format='%H:%M').dt.minute
locations_df['end_minutes'] = pd.to_datetime(locations_df['location_loading_unloading_window_end'], format='%H:%M').dt.hour * 60 + pd.to_datetime(locations_df['location_loading_unloading_window_end'], format='%H:%M').dt.minute

# Extract relevant data
locations = locations_df['location_code'].tolist()
orders = order_list_df.to_dict(orient='records')
travel_matrix = travel_matrix_df.set_index(['source_location_code', 'destination_location_code']).to_dict(orient='index')
trucks = trucks_df.to_dict(orient='records')

# Constants
service_time_customer = 20  
service_time_depot = 60  

depot1 = "A123"
customers_int=order_list_df["Destination Code"].tolist()
invoice = order_list_df["Invoice No."].tolist()
customers_1 = [str(i) for i in customers_int]
customers=zip(invoice,customers_1)
# customers = locations[:len(locations)-1]
loc_without_depot = locations[: len(locations)-1]
# Initialize the problem
prob = pulp.LpProblem("CVRPTW", pulp.LpMinimize)

# Create decision variables
x = {}
t = {}
I = {}

for k in range(len(trucks)):
    I[k] = pulp.LpVariable(f'I_{k}', cat='Binary')
    for i in locations:
        for j in locations:
            x[(i, j, k)] = pulp.LpVariable(f'x_{i}_{j}_{k}',cat='Binary')
        t[(i, k)] = pulp.LpVariable(f't_{i}_{k}', lowBound=0, cat='Continuous')

# Objective function: Minimize total distance and fixed costs
prob += pulp.lpSum(
    travel_matrix.get((i, j), {}).get('travel_distance_in_km', 0) * x[(i, j, k)] * (20000 - int(truck['truck_max_weight']) / 1000)
    for k, truck in enumerate(trucks)
    for i in locations
    for j in locations
)+pulp.lpSum(
    int(truck['truck_max_weight']) * 2 * I[k]
    for k, truck in enumerate(trucks)
), "Minimize_Total_Cost"

# Flow balancing constraint
for u,i in customers:
    if i!=j:
        prob += pulp.lpSum(x[(i, j, k)] for j in locations for k in range(len(trucks)))==pulp.lpSum(x[(j, i, k)] for j in locations for k in range(len(trucks)))==1, f"Flow_Balancing_{u}"

# demand constraint
for k, truck in enumerate(trucks):
    truck_max_weight = int(truck['truck_max_weight'])
    prob += pulp.lpSum(order['Total Weight'] * pulp.lpSum(x[(str(order['Destination Code']), j, k)] for j in locations) for order in orders ) <= truck_max_weight * I[k], f"Demand_{k}"

# Each vehicle should leave the depot once
for k in range(len(trucks)):
    prob += pulp.lpSum(x[(depot1, j, k)] for j in loc_without_depot) == 1, f"Leave_Depot_{k}"

# # Each vehicle arrives at a customer should leave for another destination
# for h in customers:
#     for k in range(len(trucks)):
#         prob += (pulp.lpSum(x[(i, h, k)] for i in locations) - pulp.lpSum(x[(h, j, k)] for j in locations)) == 0, f"customer_source_to_dest_{h}_{k}"

# Each vehicle should arrive at the depot once
for k in range(len(trucks)):
    prob += pulp.lpSum(x[(i, depot1, k)] for i in loc_without_depot) == 1, f"Arrive_Depot_{k}"

# Time window constraints
for k in range(len(trucks)):
    for i in locations:
        start_window = locations_df.loc[locations_df['location_code'] == i, 'start_minutes'].values[0]
        end_window = locations_df.loc[locations_df['location_code'] == i, 'end_minutes'].values[0]
        prob += t[(i, k)] >= start_window, f"Start_Window_{i}_{k}"
        prob += t[(i, k)] <= end_window, f"End_Window_{i}_{k}"

# Service time and travel time constraints
for k in range(len(trucks)):
    for i in locations:
        for j in locations:
            if i != j and (i, j, k) in x:
                travel_time = travel_matrix.get((i, j), {}).get('travel_time_in_min', 0)
                service_time = service_time_customer if i != depot1 and j != depot1 else service_time_depot
                prob += t[(j, k)] >= t[(i, k)] + service_time + travel_time - 1e5 * (1 - x[(i, j, k)]), f"Service_Time_{i}_{j}_{k}"

# Allowed truck types constraint
for k, truck in enumerate(trucks):
    truck_type = truck['truck_type']
    for i in locations:
        allowed_trucks_i = eval(locations_df.loc[locations_df['location_code'] == i, 'trucks_allowed'].values[0])
        for j in locations:
            allowed_trucks_j = eval(locations_df.loc[locations_df['location_code'] == j, 'trucks_allowed'].values[0])
            if truck_type in allowed_trucks_i and allowed_trucks_j:
                prob += x[(i, j, k)] <= 1, f"Allowed_Truck_{i}_{j}_{k}"
# Linking constraint
for k in range(len(trucks)):
    for i in locations:
        for j in locations:
                prob += I[k] >= x[(i, j, k)], f"Linking_{i}_{j}_{k}"

# Solve the problem
prob.solve(GUROBI(Heuristics=0.5,MIPFocus=1,MIPGap=0.02))

# Extract solution
solution = {}
if pulp.LpStatus[prob.status] == 'Optimal':
    for k in range(len(trucks)):
        solution[trucks[k]['truck_id']] = []
        for i in locations:
            for j in locations:
                if pulp.value(x[(i, j, k)]) > 0:
                    solution[trucks[k]['truck_id']].append((i, j, pulp.value(t[(i, k)])))
else:
    solution = "No optimal solution found."

print(solution)
# Print solver status
print(f"Status: {pulp.LpStatus[prob.status]}")

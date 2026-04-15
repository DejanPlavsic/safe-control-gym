# AI AKNOWLEDGEMENT: Used Cursor AI to generate repeated lines of codes, and to complete lines of code via Cursor auto-complete function, no prompt given.
# Took inspiration from code in class notes
import math
import random
import numpy as np

class Node:
    def __init__(self, position_x, position_y, node_id, childern_node_id = None, cost=0):
        self.position_x = position_x
        self.position_y = position_y
        self.node_id = node_id
        self.childern_node_id = childern_node_id
        self.cost = cost

    def distance_to_point(self, point_x, point_y): # can give distance to a point, target, other node, etc. 
        x_distance = self.position_x - point_x
        y_distance = self.position_y - point_y
        dist = np.sqrt(x_distance**2 + y_distance**2)
        return dist



def distance_between_points(point_A_x, point_A_y, point_B_x, point_B_y):
    return np.sqrt((point_B_x - point_A_x)**2 + (point_B_y - point_A_y)**2)


def find_nearest_node(Nodes, random_point_x, random_point_y): # very inefficient should switch to vectorized operation not O(n^2)
    nearest_node = None # just initializing to None
    min_dist = 100 # can't be greater than 1.41 anyways
    for node in Nodes:
        dist = node.distance_to_point(random_point_x, random_point_y)
        if dist < min_dist:
            min_dist = dist
            nearest_node = node
    return nearest_node


def find_nearest_node_vectorized(Nodes, nodes_positions, random_point_x, random_point_y):
    nodes_positions_np = np.array(nodes_positions)
    dists_sqaured = (nodes_positions_np[:, 0] - random_point_x)**2 + (nodes_positions_np[:, 1] - random_point_y)**2 # don't need to take the square root since we are just finding the min value (saves computing)
    return Nodes[np.argmin(dists_sqaured)] # argmin returns the index of the min value (NodeID)


def steer(nearest_node, random_point_x, random_point_y, step_size):
    p_x = random_point_x - nearest_node.position_x # nearest node to random point vector x component
    p_y = random_point_y - nearest_node.position_y 
    dist = math.sqrt(p_x**2 + p_y**2)

    # normalizing the vector
    p_x /= dist
    p_y /= dist

    new_point_x = nearest_node.position_x + p_x * step_size
    new_point_y = nearest_node.position_y + p_y * step_size
    return new_point_x, new_point_y, p_x, p_y


def check_collision(point_A_x, point_A_y, point_B_x, point_B_y, simplified_obstacles, bounds):
    safety_margin = 0.2 # m
    path_vector = np.array([point_B_x - point_A_x, point_B_y - point_A_y])
    path_length = distance_between_points(point_A_x, point_A_y, point_B_x, point_B_y)

    # checking if point had a collision with an obstacle
    for obstacle in simplified_obstacles:
        obstacle_to_node_vector = np.array([obstacle[0] - point_A_x, obstacle[1] - point_A_y])
        projection = np.dot(path_vector, obstacle_to_node_vector) / path_length**2

        if projection >= 0 and projection <= path_length: # if this is true nearest point is on line segment
            distance_to_obstacle = np.linalg.norm(obstacle_to_node_vector - projection * path_vector)
            if distance_to_obstacle <= safety_margin: # means a collision has been detected in the path
                print("Collision detected along path")
                return True


        if distance_between_points(point_A_x, point_A_y, obstacle[0], obstacle[1]) <= safety_margin: # means nearest node is too close to an obstacle
            print("Collision detected in nearest node")
            return True

        if distance_between_points(point_B_x, point_B_y, obstacle[0], obstacle[1]) <= safety_margin: # means nearest node is too close to an obstacle
            print("Collision detected in end point")
            return True

    return False

       
def define_obstacle_boundaries(obstacles, gates, target_gate_id):
    #simplified_obstacles = np.zeros(3, len(obstacles)+2*len(gates)) # for each gate it has 2 poles which we will treat as going from ground to cieling as a simplification
    simplified_obstacles = []
    gate_width = 0.23 # m
    
    # defining obstacle boundaries
    for obstacle in obstacles:
        simplified_obstacles.append([obstacle[0], obstacle[1]]) # x, y

    # defining gate boundaries
    for i, gate in enumerate(gates):

        gate_center_x, gate_center_y, gate_angle = gate[0], gate[1], gate[5]
        gate_pole_1 = [gate_center_x + math.cos(gate_angle) * gate_width, gate_center_y + math.sin(gate_angle) * gate_width]
        gate_pole_2 = [gate_center_x - math.cos(gate_angle) * gate_width, gate_center_y - math.sin(gate_angle) * gate_width]
        simplified_obstacles.append(gate_pole_1)
        simplified_obstacles.append(gate_pole_2)

        if i != target_gate_id:
            gate_center = [gate_center_x, gate_center_y]
            simplified_obstacles.append(gate_center)

    return simplified_obstacles


def define_before_and_after_gates(gate_info, start_position):
    gate_center_x, gate_center_y, gate_angle = gate_info[0], gate_info[1], gate_info[5]
    distance_to_gate = 0.22 # m
    infront_gate_position = [gate_center_x + math.sin(gate_angle) * distance_to_gate, gate_center_y + math.cos(gate_angle) * distance_to_gate, 1]
    behind_gate_position = [gate_center_x - math.sin(gate_angle) * distance_to_gate, gate_center_y - math.cos(gate_angle) * distance_to_gate, 1]
    
    distance_to_front = distance_between_points(start_position[0], start_position[1], infront_gate_position[0], infront_gate_position[1])
    distance_to_back = distance_between_points(start_position[0], start_position[1], behind_gate_position[0], behind_gate_position[1])

    if distance_to_front < distance_to_back:
        return infront_gate_position, behind_gate_position
    else:
        return behind_gate_position, infront_gate_position


def find_path(Nodes, target_node): # this finds the optimal path from the node at the target to home
    path = [] # path to be returned
    child_node = target_node
    while child_node.node_id != 0:
        path.append(child_node.node_id)
        child_node = Nodes[child_node.childern_node_id]
    path.append(0)
    path.reverse()
    return path


def optimize_neighbors(Nodes, cost_vector, nodes_positions, simplified_obstacles, bounds, neighbor_search_radius=0.5):
    nodes_positions_np = np.array(nodes_positions)
    for node in Nodes:
        local_neighbors = []

        nodes_positions_np = np.array(nodes_positions)
        dists_sqaured = (nodes_positions_np[:, 0] - node.position_x)**2 + (nodes_positions_np[:, 1] - node.position_y)**2
        local_neigbor_ids = np.where(dists_sqaured <= neighbor_search_radius**2)[0]

        min_cost = 10000 # initialize to a large number
        local_collision_detected = False
        optimal_local_neighbor_id = None
        for local_neighbor_id in local_neigbor_ids:
            if local_neighbor_id != node.node_id and node.node_id != 0 and node.childern_node_id != 0: # making sure it doesn't rewire to itself or reqire the first node

                local_collision_detected = check_collision(nodes_positions[local_neighbor_id][0], nodes_positions[local_neighbor_id][1], node.position_x, node.position_y, simplified_obstacles, bounds)
                if local_collision_detected == False: # confirming new path doesn't have a collision
                    
                    #cost_to_local_neighbor = cost_vector[local_neighbor_id]
                    #cost_to_new_node = dists_sqaured[local_neighbor_id]**0.5
                    #candidate_cost = cost_to_local_neighbor + cost_to_new_node
                    candidate_cost = cost_calculator(node.position_x, node.position_y, Nodes[local_neighbor_id], cost_vector)

                    if candidate_cost < min_cost: # looping through to see what is the cheapest path
                        min_cost = candidate_cost
                        optimal_local_neighbor_id = local_neighbor_id
        
        if optimal_local_neighbor_id is not None:
            node.childern_node_id = optimal_local_neighbor_id
            node.cost = min_cost
            cost_vector[node.node_id] = min_cost


def cost_calculator(new_point_x, new_point_y, candidate_node, cost_vector):
    candidate_node_x, candidate_node_y, candidate_node_id = candidate_node.position_x, candidate_node.position_y, candidate_node.node_id
    distance = distance_between_points(new_point_x, new_point_y, candidate_node_x, candidate_node_y)
    cost = cost_vector[candidate_node_id] + distance
    return cost


def linearly_optimize_path(Nodes, nodes_positions, simplified_obstacles, bounds, target_position):
    # point of this function is to try and makie the path which the drone takes more linear and less zig-zaggy
    target_node = find_nearest_node_vectorized(Nodes, nodes_positions, target_position[0], target_position[1])
    path = find_path(Nodes, target_node)
    print(f"Path: {path}")

    fully_optimized = False
    while fully_optimized == False:
        for i in range(len(path)-2):
            node_a_id = path[i]
            node_b_id = path[i+1]
            node_c_id = path[i+2]
            node_a_x, node_a_y = nodes_positions[node_a_id][0], nodes_positions[node_a_id][1]
            node_c_x, node_c_y = nodes_positions[node_c_id][0], nodes_positions[node_c_id][1]

            if i >= len(path)-3: # if it reaches the target node the loop is completed
                fully_optimized = True

            if not check_collision(node_a_x, node_a_y, node_c_x, node_c_y, simplified_obstacles, bounds):
                Nodes[node_c_id].childern_node_id = node_a_id
                path.remove(node_b_id)
                break

        path = find_path(Nodes, target_node)
        print(f"Path: {path}")

    return path


def convert_path_to_waypoints(nodes_positions, path, target_position):
    waypoints = []
    for node_id in path:
        node_x, node_y = nodes_positions[node_id][0], nodes_positions[node_id][1]
        waypoints.append([node_x, node_y, 1])
    #waypoints.append([target_position[0], target_position[1], 1])
    return waypoints
        

def rrt_dejan(initial_position, target_position, obstacles, gates, bounds, target_gate_id):
    #start_time = time.time()
    Nodes = [] # initializing nodes list
    Nodes.append(Node(initial_position[0], initial_position[1], 0))
    step_size = 0.25
    simplified_obstacles = define_obstacle_boundaries(obstacles, gates, target_gate_id) # initializing obstacle boundaries (when code is running put this in planning not here)

    nodes_positions = [] # this is for the vectorized version of nearest nodes only
    nodes_positions.append([initial_position[0], initial_position[1]])

    cost_vector = [0] # this is only for RRT* (initialize first node with cost of 0)

    valid_path_found = False # initializing to False
    i = 0
    while i < 3000: # won't stop running until it finds a path between initial and target position

        valid_point_found = False # initializing to False
        while valid_point_found == False: # won't stop running until it finds a valid point

            random_point_x = random.uniform(bounds["x"][0], bounds["x"][1]) # random uniformly distributed point in bounds
            random_point_y = random.uniform(bounds["y"][0], bounds["y"][1])
            #if i > 100 and i % 5 == 0:
            #    random_point_x = target_position[0]
            #    random_point_y = target_position[1]
            

            #nearest_node = find_nearest_node(Nodes, random_point_x, random_point_y)
            nearest_node = find_nearest_node_vectorized(Nodes, nodes_positions, random_point_x, random_point_y)

            new_point_x, new_point_y, p_x, p_y = steer(nearest_node, random_point_x, random_point_y, step_size)

            collision_detected = check_collision(nearest_node.position_x, nearest_node.position_y, new_point_x, new_point_y, simplified_obstacles, bounds)

            if collision_detected == False:
                cost = cost_calculator(new_point_x, new_point_y, nearest_node, cost_vector) # caclulating cost up to new node
                cost_vector.append(cost)

                Nodes.append(Node(new_point_x, new_point_y, i+1, nearest_node.node_id, cost=cost))
                nodes_positions.append([new_point_x, new_point_y]) # this is for the vectorized version of nearest nodes only

                if distance_between_points(new_point_x, new_point_y, target_position[0], target_position[1]) <= 0.5:
                    print("Target position reached")
                    valid_path_found = True
                
                valid_point_found = True
                i += 1

    #end_time = time.time()
    #print(f"Time taken: {end_time - start_time} seconds")

    #plotting_testing(Nodes, simplified_obstacles, nodes_positions, bounds)

    # implementing RRT* Optimization
    optimize_neighbors(Nodes, cost_vector, nodes_positions, simplified_obstacles, bounds)
    #plotting_testing(Nodes, simplified_obstacles, nodes_positions, bounds)
    print(find_path(Nodes, find_nearest_node_vectorized(Nodes, nodes_positions, target_position[0], target_position[1])))

    # Making path straighter
    path = linearly_optimize_path(Nodes, nodes_positions, simplified_obstacles, bounds, target_position)
    #plotting_testing(Nodes, simplified_obstacles, nodes_positions, bounds)

    # Turning path into functions
    waypoints = convert_path_to_waypoints(nodes_positions, path, target_position)
    print(f"Waypoints: {waypoints}")

    return waypoints

'''
def plotting_testing(Nodes, simplified_obstacles, nodes_positions, bounds):
    # Drawing All Nodes
    # drawing nodes
    for node in Nodes:
        plt.scatter(node.position_x, node.position_y, color='blue')

    # drawing obstacles
    for obstacle in simplified_obstacles:
        plt.scatter(obstacle[0], obstacle[1], color='red')

    # drawing edges
    for node in Nodes:
        if node.childern_node_id is not None:
            child_node = Nodes[node.childern_node_id]
            plt.plot([node.position_x, child_node.position_x], [node.position_y, child_node.position_y])
    plt.xlim(bounds["x"][0], bounds["x"][1])
    plt.ylim(bounds["y"][0], bounds["y"][1])
    plt.show()

    # Drawing Only Optimal Path
    # drawing obstacles
    for obstacle in simplified_obstacles:
        plt.scatter(obstacle[0], obstacle[1], color='red')
    # drawing path target to base
    target_node = find_nearest_node_vectorized(Nodes, nodes_positions, target_position[0], target_position[1])
    path = find_path(Nodes, target_node)
    for node in path:
        plt.scatter(Nodes[node].position_x, Nodes[node].position_y, color='green')

    for i in range(len(path)-1):
        plt.plot([Nodes[path[i]].position_x, Nodes[path[i+1]].position_x], [Nodes[path[i]].position_y, Nodes[path[i+1]].position_y])

    plt.xlim(bounds["x"][0], bounds["x"][1])
    plt.ylim(bounds["y"][0], bounds["y"][1])
    plt.show()
'''


if __name__ == "__main__": # This is just for testing the code
    import matplotlib.pyplot as plt
    import math
    import random
    import numpy as np
    import time

    initial_position = (-1, -3, 0)
    target_position = (-0.5, 2, 0)
    #target_position = (0, 0, 0)

    obstacles= [  # x, y, z, r, p, y
      [ 1.5, -2.5, 0, 0, 0, 0],             # obstacle 1
      [ 0.5, -1.0, 0, 0, 0, 0],             # obstacle 2
      [ 1.5,    0, 0, 0, 0, 0],             # obstacle 3
      [-1.0,    0, 0, 0, 0, 0]              # obstacle 4
    ]

    gates= [  # x, y, z, r, p, y, type 
      [ 0.5, -2.5, 1, 0, 0, -1.57, 0],      # gate 1
      [ 2.0, -1.5, 1.2, 0, 0, 0,     0],      # gate 2
      [ 0.0,  0.5, 0.8, 0, 0, 1.57,  0],      # gate 3
      [-0.5,  1.5, 0.9, 0, 0, 0,     0]       # gate 4
    ]

    bounds = {"x": [-3.5, 3.5], "y": [-3.5, 3.5]}
    rrt_dejan(initial_position, target_position, obstacles, gates, bounds)

    

# AI AKNOWLEDGEMENT: Used Cursor AI to generate repeated lines of codes, and to complete lines of code via Cursor auto-complete function, no prompt given.
# Took inspiration from code in class notes

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
    safety_margin = 0.15 # m
    path_vector = np.array([point_B_x - point_A_x, point_B_y - point_A_y])
    path_length = distance_between_points(point_A_x, point_A_y, point_B_x, point_B_y)

    # checking if point had a collision with an obstacle
    for obstacle in simplified_obstacles:
        obstacle_to_node_vector = np.array([obstacle[0] - point_A_x, obstacle[1] - point_A_y])
        projection = np.dot(path_vector, obstacle_to_node_vector)

        if projection >= 0 and projection <= path_length: # if this is true nearest point is on line segment
            distance_to_obstacle = np.linalg.norm(obstacle_to_node_vector - projection * path_vector)
            if distance_to_obstacle <= safety_margin: # means a collision has been detected in the path
                print("Collision detected in path")
                return True


        if distance_between_points(point_A_x, point_A_y, obstacle[0], obstacle[1]) <= safety_margin: # means nearest node is too close to an obstacle
            print("Collision detected in nearest node")
            return True

        if distance_between_points(point_B_x, point_B_y, obstacle[0], obstacle[1]) <= safety_margin: # means nearest node is too close to an obstacle
            print("Collision detected in end point")
            return True

    return False

       
def define_obstacle_boundaries(obstacles, gates):
    #simplified_obstacles = np.zeros(3, len(obstacles)+2*len(gates)) # for each gate it has 2 poles which we will treat as going from ground to cieling as a simplification
    simplified_obstacles = []
    gate_width = 0.26 # m
    
    # defining obstacle boundaries
    for obstacle in obstacles:
        simplified_obstacles.append([obstacle[0], obstacle[1]]) # x, y

    # defining gate boundaries
    for gate in gates:
        gate_center_x, gate_center_y, gate_angle = gate[0], gate[1], gate[5]
        gate_pole_1 = [gate_center_x + math.cos(gate_angle) * gate_width, gate_center_y + math.sin(gate_angle) * gate_width]
        gate_pole_2 = [gate_center_x - math.cos(gate_angle) * gate_width, gate_center_y - math.sin(gate_angle) * gate_width]
        simplified_obstacles.append(gate_pole_1)
        simplified_obstacles.append(gate_pole_2)

    return simplified_obstacles


def rrt_dejan(initial_position, target_position, obstacles, gates, bounds):
    Nodes = [] # initializing nodes list
    Nodes.append(Node(initial_position[0], initial_position[1], 0))
    step_size = 0.25
    simplified_obstacles = define_obstacle_boundaries(obstacles, gates) # initializing obstacle boundaries (when code is running put this in planning not here)

    valid_path_found = False # initializing to False
    i = 0
    while valid_path_found == False and i < 500: # won't stop running until it finds a path between initial and target position

        valid_point_found = False # initializing to False
        while valid_point_found == False: # won't stop running until it finds a valid point

            random_point_x = random.uniform(bounds["x"][0], bounds["x"][1]) # random uniformly distributed point in bounds
            random_point_y = random.uniform(bounds["y"][0], bounds["y"][1])

            nearest_node = find_nearest_node(Nodes, random_point_x, random_point_y)

            new_point_x, new_point_y, p_x, p_y = steer(nearest_node, random_point_x, random_point_y, step_size)

            collision_detected = check_collision(nearest_node.position_x, nearest_node.position_y, new_point_x, new_point_y, simplified_obstacles, bounds)

            if collision_detected == False:
                Nodes.append(Node(new_point_x, new_point_y, i+1, nearest_node.node_id))

                if distance_between_points(new_point_x, new_point_y, target_position[0], target_position[1]) <= 0.5:
                    print("Target position reached")
                    valid_path_found = True
                
                valid_point_found = True
                i += 1

        
    plotting_testing(Nodes, simplified_obstacles, new_point_x, new_point_y)


def plotting_testing(Nodes, simplified_obstacles, new_point_x, new_point_y):
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
    plt.show()

    # drawing obstacles
    for obstacle in simplified_obstacles:
        plt.scatter(obstacle[0], obstacle[1], color='red')
    # drawing path target to base
    base_reached = False
    path = []
    child_node = Nodes[len(Nodes)-1]
    while base_reached == False:
        path.append(child_node.node_id)
        parent_node = child_node
        child_node = Nodes[parent_node.childern_node_id]
        if child_node.node_id == 0:
            base_reached = True
    path.reverse()
    for node in path:
        plt.scatter(Nodes[node].position_x, Nodes[node].position_y, color='green')

    for i in range(len(path)-1):
        plt.plot([Nodes[path[i]].position_x, Nodes[path[i+1]].position_x], [Nodes[path[i]].position_y, Nodes[path[i+1]].position_y])
    plt.show()
    




if __name__ == "__main__": # This is just for testing the code
    import matplotlib.pyplot as plt
    import math
    import random
    import numpy as np

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

    

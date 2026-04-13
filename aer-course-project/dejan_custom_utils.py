# AI AKNOWLEDGEMENT: Used Cursor AI to generate repeated lines of codes, and to complete lines of code via Cursor auto-complete function, no prompt given.
# Took inspiration from code in class notes

class Node:
    def __init__(self, position_x, position_y, node_id, childern_node_ids = [], cost=0):
        self.position_x = position_x
        self.position_y = position_y
        self.node_id = node_id
        self.childern_node_ids = childern_node_ids
        self.cost = cost

    def distance_to_point(self, point_x, point_y): # can give distance to a point, target, other node, etc. 
        x_distance = self.position_x - point_x
        y_distance = self.position_y - point_y
        dist = np.sqrt(x_distance**2 + y_distance**2)
        return dist





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
    return new_point_x, new_point_y




  

    


def rrt_dejan(initial_position, target_position, obstacles, gates, bounds):
    Nodes = [] # initializing nodes list
    Nodes.append(Node(initial_position[0], initial_position[1], 0))
    step_size = 0.25

    #valid_path_found = False # initializing to False
    #while valid_path_found == False: # won't stop running until it finds a path between initial and target position
    # i += 1
    for i in range(10): # just while testing delete this after

        valid_point_found = False # initializing to False
        while valid_point_found == False: # won't stop running until it finds a valid point

            random_point_x = random.uniform(bounds["x"][0], bounds["x"][1]) # random uniformly distributed point in bounds
            random_point_y = random.uniform(bounds["y"][0], bounds["y"][1])

            nearest_node = find_nearest_node(Nodes, random_point_x, random_point_y)

            new_point_x, new_point_y = steer(nearest_node, random_point_x, random_point_y, step_size)

            # collision_detected = check_collision(new_point, obstacles)

            Nodes.append(Node(new_point_x, new_point_y, i+1, [nearest_node.node_id]))

            valid_point_found = True

    plotting_testing(Nodes)



    """RRT* algorithm"""
    pass


def plotting_testing(Nodes):
    for node in Nodes:
        plt.scatter(node.position_x, node.position_y)
    plt.show()



if __name__ == "__main__": # This is just for testing the code
    import matplotlib.pyplot as plt
    import math
    import random
    import numpy as np

    initial_position = (5, 5, 5)
    target_position = (10, 10, 0)
    obstacles = [(1, 1), (2, 2), (3, 3)]
    gates = [(4, 4), (5, 5), (6, 6)]
    bounds = {"x": [0, 10], "y": [0, 10]}
    rrt_dejan(initial_position, target_position, obstacles, gates, bounds)

    

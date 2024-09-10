# -*- coding: utf-8 -*-

import random

import matplotlib.pyplot as plt
import numpy as np

# Node class definition
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0


def node_2_node_distance(node1, node2):
# This function [Assumes nodes are in 2D plane] defenition is to compute the euclidean distance between two nodes.
    return np.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)

def check_collision_free(new_node, obstacles, obstacle_radius):
# This is a collision checking function. By default it assumes True, ie. no collision.
    if new_node==None:
        return False
    for obstacle in obstacles:
        distance = node_2_node_distance(new_node, obstacle)
        if distance < obstacle_radius:
            return False  # Collision
    return True  # Default -> collision free

def move_node_2_node(from_node, to_node, max_distance):
    d = node_2_node_distance(from_node,to_node)
    if d>max_distance:
        new_x = from_node.x + max_distance*(to_node.x - from_node.x)/d
        new_y = from_node.y + max_distance*(to_node.y-from_node.y)/d
    else:
        new_x = to_node.x
        new_y = to_node.y

    return Node(new_x, new_y)

def rewire_tree(tree, new_node, max_distance, obstacles, obstacle_radius):
# complete the Function that rewires the tree to update the parent of nodes if a shorter path is found.

    for node in tree:
        if node == tree[0]:
            continue
        if is_descendant(new_node, node):
            continue

        if node_2_node_distance(node, new_node)< max_distance and check_collision_free(new_node,obstacles,obstacle_radius):
            cost= cost_fun(node)
            cost_new=cost_fun(new_node)+ node_2_node_distance(new_node,node)
            if cost_new<cost:
                node.parent=new_node
                node.cost=cost_fun(node)


def is_descendant(node, potential_ancestor):
    current_node = node
    while current_node is not None:
        if current_node == potential_ancestor:
            return True
        current_node = current_node.parent
    return False

def cost_fun(node):
    cost = 0
    current_node = node
    while current_node.parent is not None:
        cost +=node_2_node_distance(current_node,current_node.parent)
        current_node= current_node.parent

    return cost

def find_nearest_node(node,tree):
    near_dist=2000
    near_node=None
    for current_node in tree:
        d=node_2_node_distance(node,current_node)
        if d<near_dist:
            near_dist=d
            near_node=current_node
    return near_node

# Main RRT* algorithm
def rrt_star(start, goal, x_range, y_range, obstacles, max_iter=1000, max_distance=0.4, obstacle_radius=0.2):
    tree = [start]
    for i in range(0,max_iter):
        rand_node=None
        while(not check_collision_free(rand_node,obstacles,obstacle_radius)):
            x = random.randrange(x_range[0], x_range[1])
            y = random.randrange(y_range[0], y_range[1])
            rand_node=Node(x,y)


        near_node=find_nearest_node(rand_node,tree)
        rand_node=move_node_2_node(near_node,rand_node,max_distance)
        if rand_node is not tree[0]:
            rand_node.parent=near_node
            rand_node.cost=cost_fun(rand_node)
        tree.append(rand_node)
        rewire_tree(tree,rand_node,max_distance,obstacles,obstacle_radius)
        for node in tree:
            if node_2_node_distance(node,goal)<max_distance:
                goal.parent=node
                goal.cost=cost_fun(goal)
                tree.append(goal)

    current_node = goal if goal.parent is not None else None
    path = []
    while current_node is not None:
        path.append(current_node)
        current_node = current_node.parent










    return path


# Here I have set up the start and goal nodes, state space, obstacles and radius of obstacle(assumed circular).
start_node = Node(0, 0)
goal_node = Node(5, 5)
x_range = (-1, 6)
y_range = (-1, 6)
obstacle1 = Node(1, 1)
obstacle2 = Node(2, 0.5)
obstacle3 = Node(2, 2)
obstacle4 = Node(3, 4)
obstacle5 = Node(3, 0)
obstacle6 = Node(4, 1)
obstacle7 = Node(3, 3)
obstacle8 = Node(1.5, 3)
obstacle9 = Node(4, 4)
obstacle10 = Node(0, 1)
obstacle11 = Node(1.3, 2)
obstacle12 = Node(2.5, 1.3)
obstacle13 = Node(3.5, 1.5)
obstacle14 = Node(4, 2)
obstacle15 = Node(4.5, 3)
obstacle16 = Node(5, 4)
obstacles = [obstacle1, obstacle2, obstacle3, obstacle4, obstacle5, obstacle6, obstacle7, obstacle8, obstacle9, obstacle10, obstacle11, obstacle12, obstacle13, obstacle14, obstacle15, obstacle16]
obstacle_radius = 0.2

# Running the RRT* algorithm.
path = rrt_star(start_node, goal_node, x_range, y_range, obstacles)

# Plotting results for Visualization.
plt.scatter(start_node.x, start_node.y, color='green', marker='o', label='Start')
plt.scatter(goal_node.x, goal_node.y, color='red', marker='o', label='Goal')
plt.scatter(*zip(*[(obstacle.x, obstacle.y) for obstacle in obstacles]), color='black', marker='x', label='Obstacle')
plt.plot([node.x for node in path], [node.y for node in path], linestyle='-', marker='.', color='blue', label='Path')

plt.legend()
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.title('RRT* Algorithm')
plt.show()

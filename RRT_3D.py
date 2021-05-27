from typing import List
from common_modules.commom_class import Obstacle
import random
import numpy as np
import numpy.linalg as lg
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Node():
          """
          Class Node is to prensent a 3D point in Cartesian space
          """
          def __init__(self, x, y, z):
                    """
                    x, y, z : The coordinate of the Node
                    vector : The vector of the coordinate
                    parent : Mark the previous Node associated with itself
                    """
                    self.x = x
                    self.y = y
                    self.z = z
                    self.vector = np.array([x, y, z])
                    self.parent = None

class RRT_3D():
          def __init__(self, start:Node, goal:Node, obstacle_list:List[Obstacle], plan_area:List[List]):
                    """
                    start : The start Point, has 4 parameters: x, y, z and a vector of (x, y, z)
                    goal : The goal Point
                    obstacle_list : The list of cubic obstacle
                    plan_area : The area of generate random Point, is a 2*3 list
                    """
                    self.start = start
                    self.goal = goal
                    self.obstacle_list = obstacle_list
                    self.plan_area = plan_area

                    self.GoalSampleRate = 0.1     #Probability of selecting the goal as a node
                    self.NodeList = [self.start]            #A list to store node that meets requirements

                    #The step length is equal to 1/5 of the minimum size of the obstacle
                    self.StepLength = min([min([obstacle.L, obstacle.W, obstacle.H])for obstacle in self.obstacle_list])/5

          def generate_random_node(self):
                    """
                    This function is used to generate a random Point
                    """
                    random_x = random.uniform(self.plan_area[0][0], self.plan_area[1][0])
                    random_y = random.uniform(self.plan_area[0][1], self.plan_area[1][1])
                    random_z = random.uniform(self.plan_area[0][2], self.plan_area[1][2])
                    return Node(random_x, random_y, random_z)

          def get_nearest_Node_in_NodeList(self, random_node:Node):
                    """
                    This function is to get the nearest node to the random_node in NodeList
                    """
                    #Used the 2 norm to calculate the distence between random_node and each node in NodeList
                    distence_list = [ lg.norm(random_node.vector-node.vector) for node in self.NodeList]
                    #Get the index of minimum distence in distence_list
                    index = distence_list.index(min(distence_list))
                    #Return the nearest node and its index
                    return self.NodeList[index], index
          
          def collision_check(self, node:Node):
                    """
                    This function is to check whether the node collides with obstacle in obstacle_list
                    """
                    #If the node is in the obstacle, collision hapened
                    collision_hanpen = False      #Safe
                    for obstacle in self.obstacle_list:
                              x_min = obstacle.x - obstacle.W/2
                              x_max = obstacle.x + obstacle.W/2
                              y_min = obstacle.y - obstacle.L/2
                              y_max = obstacle.y + obstacle.L/2
                              z_min = obstacle.y - obstacle.H/2
                              z_max = obstacle.y + obstacle.H/2
                              if node.x > x_min and node.x < x_max and node.y > y_min and node.y < y_max and node.z > z_min and node.z < z_max:
                                        collision_hanpen = True       #Danger
                                        break

                    return collision_hanpen
                    
          def get_new_node(self, nearest_node:Node, random_node:Node):
                    """
                    This function is used to get the new node which direction is from nearest_node
                    to the random_node, and meet the step length requirement
                    """
                    new_node_vector = self.StepLength*(random_node.vector - nearest_node.vector)/lg.norm(random_node.vector - nearest_node.vector) + nearest_node.vector
                    return Node(new_node_vector[0], new_node_vector[1], new_node_vector[2])

          def palnning(self):
                    """
                    Planning path
                    """
                    while True:
                              #Generate a random node
                              if random.random() > self.GoalSampleRate:
                                        random_node = self.generate_random_node()
                              else:
                                        random_node = self.goal

                              #Get the nearest node in self.NodeList
                              nearest_node, index = self.get_nearest_Node_in_NodeList(random_node)

                              #Get the new node between nearest_node and random_node with a step length
                              new_node = self.get_new_node(nearest_node, random_node)

                              #Check the new_node whether collides with obstacle in obstacle_list
                              collision_hapen = self.collision_check(new_node)

                              #If collision occurred, regenerate the random_node
                              if collision_hapen:
                                        continue
                              
                              #The collision check is passed, and set the index of the nearest_node as the new_node.parent
                              new_node.parent = index

                              #Add the new_node into the self.NodeList
                              self.NodeList.append(new_node)
                              
                              #Check whether the exit loop condition is met
                              if lg.norm(self.goal.vector - new_node.vector) < self.StepLength:
                                        break
                                        
                    #When exit the loop, we can get the path through the node.parent
                    path = [self.goal]
                    index = len(self.NodeList) - 1
                    while self.NodeList[index].parent is not None:
                              path.append(self.NodeList[index])
                              index = self.NodeList[index].parent
                    path.append(self.start)
                    path.reverse()
                    return path

          def draw_picture(self, path:List[Node]):
                    """
                    This function is used to show the obstacle and path
                    """
                    fig = plt.figure()
                    ax = Axes3D(fig)

                    #Draw the obstacle
                    args = {"alpha":1, "color":"red"}
                    for obstacle in self.obstacle_list:
                              x_min = obstacle.x - obstacle.W/2
                              x_max = obstacle.x + obstacle.W/2
                              y_min = obstacle.y - obstacle.L/2
                              y_max = obstacle.y + obstacle.L/2
                              z_min = obstacle.z - obstacle.H/2
                              z_max = obstacle.z + obstacle.H/2
                              xx = [x_min, x_min, x_min, x_min, x_min]
                              yy = [y_min, y_max, y_max, y_min, y_min]
                              zz = [z_min, z_min, z_max, z_max, z_min]
                              ax.plot3D(xx, yy, zz, **args)
                              xx = [x_max, x_max, x_max, x_max, x_max]
                              yy = [y_min, y_max, y_max, y_min, y_min]
                              zz = [z_min, z_min, z_max, z_max, z_min]
                              ax.plot3D(xx, yy, zz, **args)
                              ax.plot3D([x_min,x_max], [y_min, y_min], [z_max, z_max], **args)
                              ax.plot3D([x_min,x_max], [y_min, y_min], [z_min, z_min], **args)
                              ax.plot3D([x_min,x_max], [y_max, y_max], [z_max, z_max], **args)
                              ax.plot3D([x_min,x_max], [y_max, y_max], [z_min, z_min], **args)
                    
                    #Draw the path
                    args = {"alpha":1, "color":"green"}
                    for i in range(len(path)-1):
                              ax.plot3D([path[i].x, path[i+1].x], [path[i].y, path[i+1].y], [path[i].z, path[i+1].z], **args)
                    plt.show()


#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import cv2
import math
import ast
from queue import PriorityQueue
from numpy import inf
import time

start_time = time.time()

#Map
width = 400
height = 250
clr = 5
img = np.zeros([height, width, 3], dtype=np.uint8)
class Node:
    def __init__(self, pos, cost, parent):
        self.pos = pos
        self.x = pos[0]
        self.y = pos[1]
        self.cost = cost
        self.parent = parent

#create obstacles
def obstacles():
    img_clr = img.copy()
    #circle
    cv2.circle(img, (300,65), 40, (255,255,255), -1)
    cv2.circle(img_clr, (300,65), 45, (255,255,255), -1)
    #hexagon
    hexagon = np.array([[200, 150-70/np.sqrt(3)], [235, 150-35/np.sqrt(3)], [235, 150+35/np.sqrt(3)],
                        [200, 150+70/np.sqrt(3)], [165, 150+35/np.sqrt(3)], [165, 150-35/np.sqrt(3)]], np.int32)
    cv2.fillPoly(img, [hexagon], (255,255,255), 1)
    #hexagon_clr = np.array([[200, 150-80/np.sqrt(3)], [240, 150-40/np.sqrt(3)], [240, 150+40/np.sqrt(3)],
    #                        [200, 150+80/np.sqrt(3)], [160, 150+40/np.sqrt(3)], [160, 150-40/np.sqrt(3)]], np.int32)
    cv2.fillPoly(img_clr, [hexagon], (255,255,255), 1)
    cv2.line(img_clr, (200, int(150-70/np.sqrt(3))), (235, int(150-35/np.sqrt(3))), (255,255,255), thickness=10)
    cv2.line(img_clr, (235, int(150-35/np.sqrt(3))), (235, int(150+35/np.sqrt(3))), (255,255,255), thickness=10)
    cv2.line(img_clr, (235, int(150+35/np.sqrt(3))), (200, int(150+70/np.sqrt(3))), (255,255,255), thickness=10)
    cv2.line(img_clr, (200, int(150+70/np.sqrt(3))), (165, int(150+35/np.sqrt(3))), (255,255,255), thickness=10)
    cv2.line(img_clr, (165, int(150+35/np.sqrt(3))), (165, int(150-35/np.sqrt(3))), (255,255,255), thickness=10)
    cv2.line(img_clr, (165, int(150-35/np.sqrt(3))), (200, int(150-70/np.sqrt(3))), (255,255,255), thickness=10)
    cv2.circle(img_clr, (200, int(150-70/np.sqrt(3))), 5, (255,255,255), -1)
    cv2.circle(img_clr, (200, int(150+70/np.sqrt(3))), 5, (255,255,255), -1)
    cv2.circle(img_clr, (235, int(150-35/np.sqrt(3))), 5, (255,255,255), -1)
    cv2.circle(img_clr, (235, int(150+35/np.sqrt(3))), 5, (255,255,255), -1)
    cv2.circle(img_clr, (165, int(150-35/np.sqrt(3))), 5, (255,255,255), -1)
    cv2.circle(img_clr, (165, int(150+35/np.sqrt(3))), 5, (255,255,255), -1)
    #other
    other = np.array([[115, 40],[80,70],[105,150],[36,65]], np.int32)
    cv2.fillPoly(img, [other], (255,255,255), 1)
    cv2.fillPoly(img_clr, [other], (255,255,255), 1)
    cv2.line(img_clr, (115,40), (80,70), (255,255,255), thickness=10)
    cv2.line(img_clr, (80,70), (105,150), (255,255,255), thickness=10)
    cv2.line(img_clr, (105,150), (36,65), (255,255,255), thickness=10)
    cv2.line(img_clr, (36,65), (115,40), (255,255,255), thickness=10)
    cv2.circle(img_clr, (115,40), 5, (255,255,255), -1)
    cv2.circle(img_clr, (80,70), 5, (255,255,255), -1)
    cv2.circle(img_clr, (105,150), 5, (255,255,255), -1)
    cv2.circle(img_clr, (36,65), 5, (255,255,255), -1)
    
    #conver img_clr
    gray = cv2.cvtColor(img_clr, cv2.COLOR_BGR2GRAY)
    (t, thresh) = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
    return img, thresh

#check node inside obstacles
def check_obstacle(y, x, img_clr):
    x = 250 - x
    if x >= 250 or x < 0 or y >=400 or y < 0:
        return True
    if img_clr[x][y] == 255:
        return True
    else:
        return False

#check goal node
def check_solution(cur_node, goal_node):
    if cur_node == goal_node:
        return True
    else:
        return False

def ask_input(start):
    if start:
        x, y = [int(x) for x in input("Enter start points x, y position: ").split()]
    else:
        x, y = [int(x) for x in input("Enter goal points x, y position: ").split()]
    return x, y

#get start and goal
def get_coords(img_clr):
    start = True
    start_x, start_y = ask_input(start)
    while start_x>width or start_y>height or start_x<=0 or start_y<=0:
        print("They are outside the boundary")
        start_x, start_y = ask_input(start)
    print("Start inside the boundary")

    #start points in obstacles
    while check_obstacle(start_x, start_y, img_clr):
        print("Inside the obstacle")
        start_x, start_y = ask_input(start)

    start_node = (start_x, start_y) 

    #Input goal
    start = False
    goal_x, goal_y = ask_input(start)
    while goal_x>width or goal_y>height or start_x<=0 or start_y<=0:
        print("They are outside boundary")
        goal_x, goal_y = ask_input(start)
    print("Goal inside the boundary")

    #goal points in obstacles
    while check_obstacle(goal_x, height-goal_y, img_clr):
        print("Inside the obstacle")
        goal_x, goal_y = ask_input(start)
        
    goal_node = (goal_x, goal_y)
    return start_x, start_y, goal_x, goal_y, start_node, goal_node

#moves
def move(node):
    i = node.x
    j = node.y
    possible_moves = [(i,j+1), (i+1,j), (i-1,j), (i,j-1),
                        (i+1,j+1), (i-1,j-1),(i-1,j+1),(i+1,j-1)]
    possible_paths = []
    for pos, path in enumerate(possible_moves):
        if not path[0]>=height or path[0]<0 or path[1]>=width or path[1]<0:
                cost = math.sqrt(2) if pos>3 else 1
                possible_paths.append([path,cost])
    return possible_paths

#Dijkstra
def dijkstra(start_node, goal_node, img_clr):
    solvable = True
    parent = {}
    total_cost = {}
    visited = set()
    visited_list = []
    q = PriorityQueue()
    for i in range(0, width):
        for j in range(0, height):
            total_cost[str([i,j])] = inf
    total_cost[str(start_node)] = 0
    visited.add(str(start_node))
    visited_list.append(str(start_node))
    node = Node(start_node, 0, None)
    parent[str(node.pos)] = node
    q.put([node.cost, node.pos])
    while q:
        if q.empty():
            print("Unknown error occurs")
            print("Fail to find the path")
            break
        cur_node = q.get()
        node = parent[str(cur_node[1])]
        if check_solution(cur_node[1], goal_node):
            print("at goal")
            print("Time cost: %s seconds" %(time.time() - start_time))
            parent[str(goal_node)] = Node(goal_node, cur_node[0], node)
            break
        for next_node, cost in move(node):
            if next_node[0]<width and next_node[0]<height and next_node[0]>0 and next_node[1]>0: 
                if not check_obstacle(next_node[0], next_node[1], img_clr):
                    if str(next_node) in visited:   
                        cur_cost = cost + total_cost[str(node.pos)]
                        if cur_cost<total_cost[str(next_node)]:
                            total_cost[str(next_node)] = cur_cost
                            parent[str(next_node)].parent = node
                    else:
                        visited.add(str(next_node))
                        visited_list.append(str(next_node))
                        absolute_cost = cost + total_cost[str(node.pos)]
                        total_cost[str(next_node)] = absolute_cost
                        new_node = Node(next_node, absolute_cost, parent[str(node.pos)])
                        parent[str(next_node)] = new_node
                        q.put([absolute_cost, new_node.pos])
    goal_node = parent[str(goal_node)]
    parent_node = goal_node.parent
    backtracked=[]
    while parent_node:
        backtracked.append(parent_node.pos)
        print("Position:", parent_node.pos, " Cost:", parent_node.cost)
        parent_node = parent_node.parent
    return backtracked, visited_list

#visualization
def visual(path, img, visited, out):
    imgcopy = img.copy()
    visited = [ast.literal_eval(x.strip()) for x in visited]
    for i in visited:
        if height-i[1] >= 249:
            val = 249
        elif height-i[1] < 249:
            val = height-i[1]
        imgcopy[val, i[0]] = (255,10,0)
        out.write(imgcopy)

    for i in path:
        imgcopy[250-i[1], i[0]] = (200,255,0)
        out.write(imgcopy)
    return imgcopy

def main():
    ori_map, img_clr = obstacles()
    start_x, start_y, goal_x, goal_y, start_node, goal_node = get_coords(img_clr)
    
    #dijkstra
    print("start dijksgra")
    shortest_path, visited = dijkstra(start_node, goal_node, img_clr)
    print("success")
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter('Dijkstra.avi', fourcc, 1000, (400, 250), isColor=True)
    
    #path on the graph
    imgcopy = visual(shortest_path, ori_map, visited, out)
    cv2.circle(imgcopy, (start_x, 250-start_y), radius=1, color=(0,255,0), thickness=3)
    cv2.circle(imgcopy, (goal_x, 250-goal_y), radius=1, color=(0,0,255), thickness=3)

    #show start and goal
    for i in range(1000):
        out.write(imgcopy)
    cv2.imwrite("ori_map.png", ori_map)
    cv2.imwrite("img_clr.png", img_clr)
    cv2.imwrite("path.png", imgcopy)
    cv2.imshow("visualization", imgcopy)

    #create shortest path txt and visited txt
    file = open("shortest_path.txt", "w+")
    for i in shortest_path:
        file.write(str(i) + "\n")
        file.write("--------------------" + "\n")
    file.close()
    file = open("exploration.txt", "w+")
    for i in visited:
        file.write(str(i) + "\n")
        file.write("--------------------" + "\n")
    file.close()

    if cv2.waitKey(1) == ord('q'):
        out.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

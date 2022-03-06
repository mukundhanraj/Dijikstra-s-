import heapq as heap
import numpy as np
import cv2

# To create a list of valid points on the map


def valid_points(m_length, m_breadth, clearance):

    p_valid = []

# Defining Hexagon
    h_x1 = 200
    h_x2 = 235
    h_x3 = 235
    h_x4 = 200
    h_x5 = 165
    h_x6 = 165
    h_y1 = 140.4
    h_y2 = 120.2
    h_y3 = 79.8
    h_y4 = 59.6
    h_y5 = 79.8
    h_y6 = 120.2

# Polygon
    p_x1 = 36
    p_x2 = 115
    p_x3 = 80
    p_x4 = 105
    p_y1 = 185
    p_y2 = 210
    p_y3 = 180
    p_y4 = 100

# Circle
    c_x = 300
    c_y = 185
    c_r = 40

# Polygon line slopes
    s_21 = (p_y2 - p_y1) / (p_x2 - p_x1)
    s_32 = (p_y3 - p_y2) / (p_x3 - p_x2)
    s_43 = (p_y4 - p_y3) / (p_x4 - p_x3)
    s_14 = (p_y1 - p_y4) / (p_x1 - p_x4)

# Hexagon line slopes
    q_21 = (h_y2 - h_y1) / (h_x2 - h_x1)
    q_43 = (h_y4 - h_y3) / (h_x4 - h_x3)
    q_54 = (h_y5 - h_y4) / (h_x5 - h_x4)
    q_16 = (h_y1 - h_y6) / (h_x1 - h_x6)

    for x in range(m_length + 1):
        for y in range(m_breadth + 1):
            if ((x - c_x)**2 + (y - c_y)**2) <= c_r**2:
                continue
            if (y-p_y1) <= (s_21*(x-p_x1)) and (y-p_y2) >= (s_32*(x-p_x2)) and \
               (y-p_y4) >= (s_14*(x-p_x4)):
                continue
            if (y-p_y1) <= (s_21*(x-p_x1)) and (y-p_y3) <= (s_43*(x-p_x3)) and \
               (y-p_y4) >= (s_14*(x-p_x4)):
                continue
            if x <= h_x2 and x >= h_x5 and y >= h_y5 and y <= h_y2:
                continue
            if y >= h_y2 and (y-h_y1) <= (q_21*(x-h_x1)) and (y-h_y6) <= (q_16*(x-h_x6)):
                continue
            if y <= h_y5 and (y-h_y3) >= (q_43*(x-h_x3)) and (y-h_y4) >= (q_54*(x-h_x4)):
                continue
            p_valid.append((x, y))
    return p_valid

#Point Validity Checking Function


def validity_check(point, p_valid, clearance):
    # p_valid: All valid points, Cleareance: Minimum distance from the obstacle
    for i in range(-clearance, clearance):
        for j in range(-clearance, clearance):
            if not (point[0] + i, point[1] + j) in p_valid:
                return False
    return True

#Function for generating adjacent nodes to the given node


def adjacent_nodes(n_current, p_valid, clearance):
    adjNodes = []

    moves = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1), (1, 1, 1.4),
             (1, -1, 1.4), (-1, 1, 1.4), (-1, -1, 1.4)]
    flag = True
    for move in moves:
        if (n_current[0] + move[0], n_current[1] + move[1]) in p_valid:
            for i in range(clearance):
                if not (n_current[0] + move[0] + (i * move[0]), n_current[1]
                        + move[1] + (i * move[1])) in p_valid:
                    flag = False
                    break
                if not flag:
                    break
            if flag:
                adjNodes.append(((n_current[0] + move[0],
                                n_current[1] + move[1]), move[2]))
    return adjNodes

#Function to update the nodes depending on the cost of nodes


def node_updt(n_new, n_current, n_cost, queue, map_parent, cost, goal):

    cost_new = n_cost[n_current] + cost
    cost_temp = n_cost.get(n_new)
    if not cost_temp or (cost_temp > cost_new):
        n_cost[n_new] = cost_new
        map_parent[n_new] = n_current
        heap.heappush(queue, (cost_new, n_new))
    if n_new == goal:
        return True, n_cost, queue, map_parent
    return False, n_cost, queue, map_parent

#function for dijikstras_algorithm


def dijkstras_algorithm(start, goal, p_valid, clearance):
    closed = []
    queue = []
    n_cost = {}
    map_parent = {}
    reached = False

    n_cost[start] = 0
    heap.heappush(queue, (0, start))

    if goal == start:
        reached = True
        map_parent[goal] = start

    while not reached and queue:
        curc_rost, n_current = heap.heappop(queue)
        closed.append(n_current)
        adjNodes = adjacent_nodes(n_current, p_valid, clearance)
        for n_new, cost in adjNodes:
            if n_new in closed:
                continue
            print('checking for node: ', n_new)
            flag, n_cost, queue, map_parent = node_updt(
                n_new, n_current, n_cost, queue, map_parent, cost, goal)
            if flag:
                reached = True
                break
    return reached, map_parent, closed

#Backtracking function


def getPath(map_parent, start, goal):
    n_current = goal
    parent_node = map_parent[goal]
    path = [n_current]
    while not parent_node == start:
        n_current = parent_node
        parent_node = map_parent[n_current]
        path.append(n_current)
    path.append(start)
    return path[::-1]

#Function to generate animation of the generated path


def animate(m_length, m_breadth, p_valid, closed, path):
    m_frame = np.zeros((m_breadth + 1, m_length + 1, 3))
    delay = 5
    cnt = 0
    resize = (800, 500)
    for point in p_valid:
        m_frame[m_breadth - point[1], point[0]] = [255, 255, 255]
    for point in closed:
        m_frame[m_breadth - point[1], point[0]] = [130, 0, 0]
        cv2.imshow('map_frame', cv2.resize(m_frame, resize))
        cnt = cnt + 1
        if cnt == delay:
            cnt = 0
            cv2.waitKey(1)
    for point in path:
        m_frame[m_breadth - point[1], point[0]] = [0, 0, 130]
        cv2.imshow('map_frame', cv2.resize(m_frame, resize))
        cv2.waitKey(1)

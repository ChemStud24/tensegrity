import heapq
import numpy as np
import random
from close_node import is_point_within_distance
from util_heuristic import wave_heuristic, dist_heuristic, l2_dist, coll_det


def rel_mov(x, y, theta):
    # Calculate the change in x and y using trigonometry
    dx = x * np.cos(theta) - y * np.sin(theta)
    dy = x * np.sin(theta) + y * np.cos(theta)
    return dx, dy

def angle_norm(x):
    return x % (2*np.pi)

def heuristic(a, b, obstacles, heur_type, k=0, grid_step=0.01):
    if heur_type == "wave":
        return wave_heuristic(a, b, obstacles=obstacles, grid_step=grid_step)
    else:
        return dist_heuristic(a, b, obstacles, k)





# A* algorithm implementation
def astar(start, goal, gaits, obstacles=(),tolerance = 0.1, rot_tol = np.pi/4, repeat_tol = 0.07, single_push = False, stochastic = True,\
          heur_type = "dist", boundary = (-1,1,-1,1), obstacle_dims=(0.4,0.27)):

    # Initialize open and closed lists
    open_list = []
    heapq.heappush(open_list, (0, start, -1))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal, obstacles, heur_type)}
    closed_list = [] #only used to check if already visited using kd tree

    while open_list:
        # print(open_list)
        # Get the node with the lowest f_score value
        node = heapq.heappop(open_list)
        current = node[1]
        # print(current)
        # print(current)
        # print(current)

        if coll_det(current, obstacles, boundary=boundary, obstacle_dims=obstacle_dims) or (len(closed_list)>0 and is_point_within_distance(current[:2], closed_list, repeat_tol)):
                # print("Too close", neighbor)
                closed_list.append(current[:2])
                continue

        print(str(current), str(g_score[current]), str(heuristic(current[:2], goal, obstacles, heur_type)), str(f_score[current]))
        if not single_push:
            closed_list.append(current[:2])
        # ang_switch = np.pi/4
        # if current[2] < ang_switch:
        #     closed_list.append((current[0],current[1], 2*np.pi - current[2]))
        # elif current[2] > 2*np.pi - ang_switch:
        #     closed_list.append((current[0],current[1], 2*np.pi - current[2]))
        # print(current)

        # If the goal is reached, reconstruct and return the path
        if l2_dist(current[:2], goal[:2])<= tolerance and min(abs(angle_norm(current[2]) - goal[2]), np.pi - abs(angle_norm(current[2]) - goal[2])) <= rot_tol :
            # print(current)
            path = []
            movements = []
            while current in came_from:
                prev = came_from[current][0]
                move = came_from[current][1]
                path.append(current)
                movements.append(move)
                current = prev
            path.append(start)
            return path[::-1], movements[::-1], gaits

            # while current in came_from:
            #     path.append(current)
            #     current = came_from[current][0]
            # path.append(start)
            # return path[::-1], gaits

        if single_push:
            if stochastic:
                each_neighbor = []
            else:
                best_neighbor = None
        # best_f = 0
        for k in range(len(gaits)):
            gait_num = gaits[k]
            dx , dy = rel_mov(gait_num[0], gait_num[1], current[2])
            neighbor = (current[0]+dx, current[1]+dy, angle_norm(current[2]+gait_num[3]))
            
            tentative_g_score = g_score[current] + np.sqrt(dx**2 + dy**2)
            if tentative_g_score < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = (current, k)
                g_score[neighbor] = tentative_g_score
                
                f_score[neighbor] = tentative_g_score + heuristic(neighbor[:2], goal, obstacles, heur_type)
                # if neighbor not in [i[1] for i in open_list]:
                    # print(neighbor)
                if single_push:
                    if stochastic:
                        each_neighbor.append((f_score[neighbor], neighbor, k))
                    else:
                        if best_neighbor == None or f_score[neighbor] < best_neighbor[0]:
                            best_neighbor = (f_score[neighbor], neighbor, k)
                else:
                    heapq.heappush(open_list, (f_score[neighbor], neighbor, k))

        if single_push:
            if stochastic:
                if len(each_neighbor)==0:
                    closed_list.append(current[:2])
                    continue
                elif len(each_neighbor)==1:
                    heapq.heappush(open_list, each_neighbor[0])
                beta = 1.0
                weights = [np.exp(-beta * node[0]) for node in each_neighbor]
                total_weight = sum(weights)
                probabilities = [weight / total_weight for weight in weights]
                
                # Select a node based on the probabilities
                best_neighbor = random.choices(each_neighbor, weights=probabilities, k=1)[0]
                heapq.heappush(open_list, best_neighbor)
                heapq.heappush(open_list, node)

            else:
                if best_neighbor != None:
                    heapq.heappush(open_list, best_neighbor)
                    heapq.heappush(open_list, node)
                else:
                    closed_list.append(current[:2])
        
              


    # Return empty path if no path found
    print("Can't Find Path")
    return [], [], gaits

# Example usage
# grid = [
#     [0, 1, 0, 0, 0],
#     [0, 1, 0, 1, 0],
#     [0, 0, 0, 1, 0],
#     [0, 1, 0, 0, 0],
#     [0, 0, 0, 0, 0],
# ]
# start = (0, 0)  # Start position
# goal = (4, 4)   # Goal position

# path = astar(grid, start, goal)
# print("Path found:", path)

import numpy as np
import heapq
import matplotlib.pyplot as plt
import time
# from shapely.geometry import Polygon, box

def get_rotated_corners(x, y, w, h, theta):
    dx = w / 2
    dy = h / 2

    corners = [
        (-dx, -dy),
        (dx, -dy),
        (dx, dy),
        (-dx, dy)
    ]

    rotated_corners = []
    for corner in corners:
        cx = corner[0] * np.cos(theta) - corner[1] * np.sin(theta)
        cy = corner[0] * np.sin(theta) + corner[1] * np.cos(theta)
        rotated_corners.append((x + cx, y + cy))

    return rotated_corners


def check_overlap(corners1, corners2):
    # Use Separating Axis Theorem (SAT) to check for overlap
    axes = get_axes(corners1) + get_axes(corners2)

    for axis in axes:
        if not projection_overlap(corners1, corners2, axis):
            return False
    return True

def get_axes(corners):
    axes = []
    for i in range(len(corners)):
        p1 = corners[i]
        p2 = corners[(i + 1) % len(corners)]
        edge = (p2[0] - p1[0], p2[1] - p1[1])
        axis = (-edge[1], edge[0])  # Perpendicular vector
        length = np.sqrt(axis[0] ** 2 + axis[1] ** 2)
        axes.append((axis[0] / length, axis[1] / length))
    return axes

def projection_overlap(corners1, corners2, axis):
    def project(corners, axis):
        return [corner[0] * axis[0] + corner[1] * axis[1] for corner in corners]

    p1 = project(corners1, axis)
    p2 = project(corners2, axis)

    if max(p1) < min(p2) or max(p2) < min(p1):
        return False
    return True

def coll_det(point, obstacles, robot_dims=(0.33, 0.15), obstacle_dims=(0.35,0.35), boundary = (-3,1,-1.4,0.4)):
    #(0.4, 0.27)
    #(-1.0, 1.0, -1.0, 4.0)):
    #boundary is (x_min, x_max, y_min, y_max)
    robot_x, robot_y, robot_theta = point
    robot_w, robot_h = robot_dims
    obs_w, obs_h = obstacle_dims

    # Calculate robot bounding box corners
    robot_corners = get_rotated_corners(robot_x, robot_y, robot_w, robot_h, robot_theta)
    # print(robot_corners)

    for corner in robot_corners:
        if boundary[0] > corner[0] or boundary[1] < corner[0] or boundary[2] > corner[1] or boundary[3] < corner[1]:
            # print("Border", str(corner))
            # print("Border")
            return True


    for obs_x, obs_y in obstacles:
        # Calculate obstacle bounding box corners (assuming no rotation)
        obstacle_corners = get_rotated_corners(obs_x, obs_y, obs_w, obs_h, 0)

        # Check if there is overlap between the robot and the obstacle
        if check_overlap(robot_corners, obstacle_corners):
            # print("obstacle")
            return True
    
    return False

# def make_robot_polygon(x, y, theta, robot_dims):
#     """
#     Create a Shapely Polygon for the robot.

#     Parameters
#     ----------
#     x, y : float
#         Center of the robot.
#     theta : float
#         Heading of the robot in radians.
#     robot_dims : (float, float)
#         (length, width) of the robot.
#     """
#     length, width = robot_dims
    
#     # Half dimensions
#     half_l = length / 2.0
#     half_w = width  / 2.0
    
#     # Define corners in local coordinates (centered at origin)
#     # Bottom-left, bottom-right, top-right, top-left
#     local_corners = [
#         (-half_l, -half_w),
#         ( half_l, -half_w),
#         ( half_l,  half_w),
#         (-half_l,  half_w)
#     ]
    
#     # Rotate and translate corners to global coordinates
#     global_corners = []
#     for (cx, cy) in local_corners:
#         # Rotate by theta
#         rx =  cx * np.cos(theta) - cy * np.sin(theta)
#         ry =  cx * np.sin(theta) + cy * np.cos(theta)
#         # Translate by (x, y)
#         gx, gy = (x + rx, y + ry)
#         global_corners.append((gx, gy))
    
#     # Create and return a Shapely polygon
#     return Polygon(global_corners)


# def coll_det(point, obstacles, robot_dims=(0.33, 0.15), obstacle_dims=(0.4, 0.27), boundary = (-1.0, 1.0, -1.0, 4.0)):
#     """
#     Check if the oriented rectangular robot collides with any axis-aligned rectangular obstacles.

#     Parameters
#     ----------
#     x, y : float
#         Center of the robot.
#     theta : float
#         Heading of the robot in radians.
#     robot_dims : (float, float)
#         (length, width) of the robot.
#     obstacles : list of (float, float)
#         List of (x, y) centers of obstacles.
#     obstacle_dims : (float, float)
#         (length, width) of each obstacle.

#     Returns
#     -------
#     bool
#         True if there is any collision; False otherwise.
#     """
#     x,y,theta = point
#     # Create the robot polygon
#     robot_poly = make_robot_polygon(x, y, theta, robot_dims)
    
#     # For each obstacle
#     for (ox, oy) in obstacles:
#         # Create an axis-aligned box for the obstacle
#         obs_length, obs_width = obstacle_dims
#         obs_box = box(ox - obs_length/2, 
#                       oy - obs_width/2,
#                       ox + obs_length/2, 
#                       oy + obs_width/2)
        
#         # Check collision
#         if robot_poly.intersects(obs_box):
#             return True  # Collision found
    
#     return False  # No collision found

def l2_dist(a,b):
    return np.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))

def dist_heuristic(a, b, obstacles=[], k=0):
    dist = l2_dist(a,b)
    if k==0 or len(obstacles)<1:
        return dist

    penalty = 0
    for obstacle in obstacles:
        distance_to_obstacle = l2_dist(a, obstacle)
        penalty += 1 / (1+distance_to_obstacle)


    return dist + k*penalty

# print(dist_heuristic((1.5982203824585355, -0.38906994686732577),(0.0,3.0)))

########
def snap_to_grid(point, grid_step):
    return ((round(point[0] / grid_step) * grid_step, round(point[1] / grid_step) * grid_step))

def simple_collision(point, obstacles, obstacle_size=(0.4, 0.27)):
    px, py = point
    half_w, half_h = obstacle_size[0] / 2, obstacle_size[1] / 2
    for ox, oy in obstacles:
        if (ox - half_w <= px <= ox + half_w) and (oy - half_h <= py <= oy + half_h):
            return True
    return False

def wave_heuristic(start, goal, grid_step=0.1, obstacles=[]):
    start = snap_to_grid(start, grid_step)#now in the format (x,y)
    goal = snap_to_grid(goal, grid_step)

    if simple_collision(start, obstacles):
        return 1000
    

    gaits = [[grid_step,0],[0,grid_step],[0,-grid_step],[-grid_step,0],\
             [grid_step,grid_step],[-grid_step,grid_step],[grid_step,-grid_step],[-grid_step,-grid_step]]

    open_list = []
    
    # came_from = {}
    g_score = {start: 0}
    f_score = {start: dist_heuristic(start, goal)}
    heapq.heappush(open_list, (f_score[start], start))
    closed_list = set() #only used to check if already visited using kd tree

    # best = f_score[start]

    while open_list:
        # print(open_list)
        # Get the node with the lowest f_score value
        node = heapq.heappop(open_list)
        # print(node[0])
        current = node[1]

        closed_list.add(current)
        # if f_score[current] < best:
        #     print(f_score[current])
        # print(closed_list)

        if simple_collision(current, obstacles):
                continue

        
        # If the goal is reached, reconstruct and return the path
        if current==goal:
            # print(g_score[current])
            return g_score[current]
            # # print(current)
            # path = []
            # # movements = []
            # cath_cost
            # while current in came_from:
            #     prev = came_from[current][0]
            #     move = came_from[current][1]
            #     path.append(current)
            #     movements.append(move)
            #     current = prev
            # path.append(start)
            # return path[::-1], movements[::-1], gaits

            # while current in came_from:
            #     path.append(current)
            #     current = came_from[current][0]
            # path.append(start)
            # return path[::-1], gaits

        for k in range(len(gaits)):
            gait_num = gaits[k]
            neighbor = snap_to_grid((current[0]+gait_num[0], current[1]+gait_num[1]),grid_step)

            if neighbor in closed_list:
                continue

            if k <4:
                cost = 1
            else:
                cost = np.sqrt(2)
            
            tentative_g_score = g_score[current] + cost
            if tentative_g_score < g_score.get(neighbor, float('inf')):
                # came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + dist_heuristic(neighbor[:2], goal, obstacles)
                heapq.heappush(open_list, (f_score[neighbor], neighbor))




# start = (0.05, 0.1)
# goal = (1.5, 1.5)
# obstacles = [(0.4, 0.4), (0.6, 0.6)]  # Smaller obstacles
# ts= time.time()
# path_length = wave_heuristic(start, goal, obstacles=obstacles,grid_step=0.1)
# te = time.time()
# print("Time:", str(te-ts))
# print("Path length:", path_length)
# plot_problem(start, goal, obstacles)

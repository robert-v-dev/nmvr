import rclpy
from rclpy.node                 import Node
from std_msgs.msg               import String
from tkinter                    import *
from math                       import cos, pow, atan2, sin, sqrt
import numpy                    as np
import time

robot = 0
grid = 0
r_pos_x = 0
r_pos_y = 0
r_theta = 0
goal_pose_x = 0
goal_pose_y = 0
distance_tolerance = 0
canvas = 0
root = 0
goal_grid_pose_x = 0
goal_grid_pose_y = 0

# D* Lite
map = 0
prev_map = 0
s_last = 0
k_m = 0  # accumulation
U = {}
rhs = {}
g = {}



for i in range(0,20):
    for y in range(0,20):
        rhs[(i,y)]=float("inf")
        g[(i,y)]=float("inf")

s_start = 0
s_goal = 0
cringe = []

origin_path = []
miro = False

def sub_callback(msg):
    global grid, map                                               # Globalna premenna mriezky
    grid = [int(char) for char in msg.data]                        # Formatovanie mriezky
    dim = int(sqrt(len(grid)))
    map = np.transpose(np.array(grid).reshape((dim,dim)))
    show_map()
    

def show_map():
    global robot, s_start, s_last, cringe, origin_path
    canvas.place(relx=0.5, rely=0.5, anchor=CENTER)
    for iter in range(0,400):                                      # Vykreslenie poli
    
        pos_y=int(iter / 20)
        pos_x=iter-(pos_y * 20)

        if grid[iter] == 2: color="yellow"

        elif grid[iter] == 1: color="green"

        else: color="grey"

        canvas.create_rectangle( pos_x*30,  pos_y*30, (pos_x*30)+30, (pos_y*30)+30, fill=color)

        if(grid[iter] == 2):
            robot_img = PhotoImage(file="/home/nmvr/Desktop/task1/src/sim/sim/robot.png")
            if r_pos_x == 0 and r_pos_y == 0:
                robot = canvas.create_image((pos_x*30),(pos_y*30), anchor=NW, image=robot_img)
                
                s_start = (np.where(map == 2)[0][0],np.where(map == 2)[1][0])                        # Init s_last to starting position
                s_last = (np.where(map == 2)[0][0],np.where(map == 2)[1][0])
                U[s_goal] = calculateKey(s_goal)
            else:
                robot = canvas.create_image(r_pos_x,r_pos_y, anchor=NW, image=robot_img)

    for e in origin_path:
        pos_x,pos_y = e
        canvas.create_rectangle( pos_x*30,  pos_y*30, (pos_x*30)+30, (pos_y*30)+30, fill="orange")

    for e in cringe:
        pos_x,pos_y = e
        canvas.create_rectangle( pos_x*30,  pos_y*30, (pos_x*30)+30, (pos_y*30)+30, fill="red")

    canvas.tag_raise(robot)
        
    """D* lite implementacia"""
    global k_m, miro, prev_map, g, rhs
    if not miro:

        computeShortestPath()
        origin_path = ExtractPath()
        prev_map = map
        miro = True
        time.sleep(2)


    if (s_start != s_goal):
        """s_start = argmin"""
        x,y = s_start
        g_list = {}
        s = None
        for q in neighbors(map,x,y):
            g_list[q] = g[q]
            s = min(g_list, key = g_list.get)

        s_start = s  

        """move to start"""
        x,y = s_start

        cringe.append(s_start)

        while euclidean_distance(x*30, y*30) >= distance_tolerance:
            move_robot(x*30,y*30)

        """if any wall added"""
        
        if not np.array_equal(map,prev_map):
            
            k_m = k_m + heuristic(s_last,s_start)
            s_last = s_start

            diff_map = np.where(prev_map != map)
            prev_map = map
            x = diff_map[0][0]
            y = diff_map[1][0]
            if map[diff_map] == 1:
                rhs[(x,y)] = float("inf")
                g[(x,y)] = float("inf")
            else:
                updateVertex((x,y))
            
            for s in neighbors(map,x,y):
                updateVertex(s)
            computeShortestPath()


    root.update()

def move_robot(pose_x,pose_y):
    global r_pos_x, r_pos_y, r_theta, root
    r_pos_x, r_pos_y = canvas.coords(robot)

    t = 0.1     #perióda vzorkovania
    l = 0.03    #šírka kolies

    """P - regulator"""
    if euclidean_distance(pose_x, pose_y) >= distance_tolerance:
        v = linear_vel(pose_x, pose_y)
        w = angular_vel(pose_x, pose_y)
        """Dolny priepust"""
        if v > 20: v = 20
        if w > 10: w = 10

    else:
        v = 0
        w = 0




    """Odometry"""
    dr = (v + (0.5 * l * w)) * t
    dl = (v - (0.5 * l * w)) * t

    fi = (dr - dl) / l
    d_center = (dl + dr) / 2
    r_theta = r_theta + fi

    new_r_pos_x = r_pos_x + (d_center*cos(r_theta))
    new_r_pos_y = r_pos_y + (d_center*sin(r_theta))


    move_x = new_r_pos_x - r_pos_x
    move_y = new_r_pos_y - r_pos_y


    canvas.move(robot, move_x, move_y)
    canvas.tag_raise(robot)
    
    

    root.update()
    r_pos_x, r_pos_y = canvas.coords(robot)





def euclidean_distance(x,y):
    """Euclidean distance between current pose and the goal."""
    return sqrt(pow((x - r_pos_x), 2) +
                pow((y - r_pos_y), 2))

def wrap(angle):
    while angle > np.pi:
        angle -= 2.0 * np.pi
    
    while angle< - np.pi:
        angle += 2.0 * np.pi
    return angle

def linear_vel(x,y,constant=1.5):
    return constant * euclidean_distance(x,y)

def steering_angle(x,y):
    return wrap(atan2(y - r_pos_y, x - r_pos_x))

def angular_vel(x,y,constant=6):
    return constant * wrap((steering_angle(x,y) - r_theta))


"""D* Lite functions"""

def heuristic(start, pos):
    return abs(start[0] - pos[0]) + abs(start[1] - pos[1])                          # Manhattan metric
    #return sqrt(pow((start[0] - pos[0]), 2) + pow((start[1] - pos[1]), 2))         # Euclidean metric
    

def calculateKey(pos):
    return [min(g[pos],rhs[pos]) + heuristic(s_start,pos) + k_m, min(g[pos],rhs[pos])]


def neighbors(arr,x,y):
    result = []
    #if map[x][y] == 1:

    if (x + 1) < len(arr):
        result.append((x + 1,y))

        if (y - 1) >= 0:
            result.append((x + 1,y - 1))
        
        if (y + 1) < len(arr[0]):
            result.append((x + 1,y + 1))

    if (x - 1) >= 0:
        result.append((x - 1,y))

        if (y - 1) >= 0:
            result.append((x - 1,y - 1))
        
        if (y + 1) < len(arr[0]):
            result.append((x - 1,y + 1))

    if (y + 1) < len(arr[0]):
        result.append((x,y + 1))

        if (x + 1) < len(arr):
            result.append((x + 1,y + 1))

        if (x - 1) >= 0:
            result.append((x - 1,y + 1))

    if (y - 1) >= 0:
        result.append((x,y - 1))

        if (x + 1) < len(arr):
            result.append((x + 1,y - 1))
            
        if (x - 1) >= 0:
            result.append((x - 1,y - 1))
        
    result = list(dict.fromkeys(result))    #remove duplicates
    buff = list(dict.fromkeys(result))

    for r in buff:
        x,y = r
        if map[x][y] == 1:
            result.remove(r)
    return result


def c(u, v):
    """
    calcuclate the cost between nodes
    :param u: from vertex
    :param v: to vertex
    :return: euclidean distance to traverse. inf if obstacle in path
    """
    #if not sensed_map.is_unoccupied(u) or not sensed_map.is_unoccupied(v):
    if map[u] == 1 or map[v] == 1:
        return float('inf')
    else:
        return heuristic(u, v)

def updateVertex(u):
    global U, rhs
    
    if u != s_goal:
        x,y = u
        neighbor = neighbors(map,x,y)
        rhs[u] = min([c(u,s) + g[s] for s in neighbor])

    if u in U:
        U.pop(u)
    
    if g[u] != rhs[u]:
        U[u] = calculateKey(u)
    
def ExtractPath():
    path = [s_start]
    s = s_start
    while True:
        g_list = {}
        x,y = s
        for q in neighbors(map,x,y):
            g_list[q] = g[q]
        s = min(g_list, key = g_list.get)
        path.append(s)

        if s == s_goal:
            break
    return list(path)

def computeShortestPath():
    global U, g

    while(topKey(U)[1] < calculateKey(s_start) or rhs[s_start] != g[s_start]):
        
        u, k_old = topKey(U)   # hodnota rhs a g
        U.pop(u)               # index bodu
        
        if k_old < calculateKey(u):
            U[u] = calculateKey(u)

        elif g[u] > rhs[u]:
            g[u] = rhs[u]
            x,y = u
            for neighbor in neighbors(map,x,y):
                updateVertex(neighbor)
        else:
            g[u] = float("inf")
            updateVertex(u)
            x,y = u
            for neighbor in neighbors(map,x,y):
                updateVertex(neighbor)
        


def topKey(U):
    """
    get point in map with lowest heuristic element
    """
    if len(U) > 0:
        s = min(U,key=U.get)
        return [s, U[s]]
    else:
        return [float("inf"),[float("inf"),float("inf")]]
    


def main():
    global root, canvas, goal_pose_x, goal_pose_y, goal_grid_pose_x, goal_grid_pose_y, distance_tolerance, rhs, U, s_goal
    
    """goal position and tolerance from user"""
    goal_grid_pose_x = int(input("Set your x goal: "))
    goal_grid_pose_y = int(input("Set your y goal: "))
   
    goal_pose_x = goal_grid_pose_x * 30
    goal_pose_y = goal_grid_pose_y * 30
    distance_tolerance = int(input("Set your tolerance: "))

    """D* lite"""
    s_goal = (goal_grid_pose_x, goal_grid_pose_y)
    rhs[s_goal] = 0

    """Tkinter window init"""
    root = Tk()
    root.geometry('700x700')
    canvas = Canvas(root, height=600, width=600, bg="blue")
    
    """ROS init"""
    rclpy.init()
    node=rclpy.create_node("simulator")
    sub = node.create_subscription(String, "csv_read", sub_callback,10)
    rclpy.spin(node)



if __name__== '__main__':
    main()

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tkinter import *
from math import cos, pow,atan2, sin,sqrt


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

def sub_callback(msg):
    global grid                                                  #globalna premenna mriezky
    grid=[int(char) for char in msg.data]                        #formatovanie mriezky
    show_map()

def show_map():
    global robot
    canvas.place(relx=0.5, rely=0.5, anchor=CENTER)
    for iter in range(0,400):                                   #vykreslenie poli
    
        pos_y=int(iter/20)
        pos_x=iter-(pos_y*20)

        if grid[iter]==2:
            color="yellow"

        elif grid[iter]==1:
            color="green"

        else:
            color="grey"

        canvas.create_rectangle( pos_x*30,  pos_y*30, (pos_x*30)+30, (pos_y*30)+30, fill=color)

        if(grid[iter]==2):
            robot_img = PhotoImage(file="/home/nmvr/Desktop/task1/src/sim/sim/robot.png")
            if r_pos_x==0 and r_pos_y==0:
                robot=canvas.create_image((pos_x*30),(pos_y*30), anchor=NW, image=robot_img)

            else:
                robot=canvas.create_image(r_pos_x,r_pos_y, anchor=NW, image=robot_img)

    
    canvas.tag_raise(robot)
    move_robot()
    root.update()

def move_robot():
    global r_pos_x, r_pos_y, r_theta, root
    r_pos_x, r_pos_y = canvas.coords(robot)


    t = 0.1                           #perióda vzorkovania
    l = 0.03                           #šírka kolies

    """P - regulator"""
    if euclidean_distance() >= distance_tolerance:
        v = linear_vel()
        w = angular_vel()
        """Dolny priepust"""
        if v > 20: v = 20
        if w > 10: w = 10

    else:
        v = 0
        w = 0

    #print("linear = " + str(v) + " angular = " + str(w) + "\n")



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
    #print("move_x = " + str(move_x) + " move_y = " + str(move_y) + "\n")
    

    root.update()
    r_pos_x, r_pos_y = canvas.coords(robot)
    print("r_pos_x = " + str(r_pos_x) + " r_pos_y = " + str(r_pos_y) + " r_theta = " + str(r_theta) +"\n")




def euclidean_distance():
    """Euclidean distance between current pose and the goal."""
    return sqrt(pow((goal_pose_x - r_pos_x), 2) +
                pow((goal_pose_y - r_pos_y), 2))

def linear_vel(constant=1.5):
    """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
    return constant * euclidean_distance()

def steering_angle():
    """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
    return atan2(goal_pose_y - r_pos_y, goal_pose_x - r_pos_x)

def angular_vel(constant=6):
    """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
    return constant * (steering_angle() - r_theta)




    
    
def main():
    global root, canvas, goal_pose_x, goal_pose_y, distance_tolerance
    
    """goal position and tolerance from user"""
    goal_pose_x = (int(input("Set your x goal: "))*30)
    goal_pose_y = (int(input("Set your y goal: "))*30)
    distance_tolerance = int(input("Set your tolerance: "))

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
    

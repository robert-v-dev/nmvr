import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tkinter import *



def sub_callback(msg):
   global grid                                                  #globalna premenna mriezky
   grid=[int(char) for char in msg.data]                        #formatovanie mriezky
   show_map()

def show_map():
    canvas = Canvas(root, height=600, width=600, bg="blue")
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
    root.update()
    
    
def main():
    rclpy.init()
    node=rclpy.create_node("simulator")
    sub = node.create_subscription(String, "csv_read", sub_callback,10)
    global root 
    root = Tk()
    root.geometry('900x900')
    rclpy.spin(node)


if __name__== '__main__':
    main()
    
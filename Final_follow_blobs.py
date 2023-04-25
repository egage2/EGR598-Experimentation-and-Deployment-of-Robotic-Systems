#EGR598:Experimentation and Deployment of Robotic Systems Spring 2023
#Final Project - Color Dependent Blob follower/tracker with filtered data
#Blob Subscriber, motion publisher, and live plotter code
#Ethan Gage
#Akshay Fulzele
#Daniel Espinoza-Pena

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from sensor_msgs.msg import Image
from std_msgs.msg import String 
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from time import time

import matplotlib.pyplot as plt 
from matplotlib.animation import FuncAnimation

MEMORY = 100

my_frame  = np.zeros((MEMORY,2))

class ImageSubscriber(Node):
  def __init__(self):
    self.past = time()
    super().__init__('subscriber')
    self.subscription = self.create_subscription(String,'blobs', self.listener_callback, 10)
    self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
    self.t0 = None
    self.history = None
    self.kalmanHistory = None
    self.fig  = None
    self.count = 1
    self.colorFound = ''
    self.current_time = None
    
    #P Controller coefficient for L/R twist
    self.angularKp = 1/10000 #derived via trial and error
    self.linearKp = 4/1000 #derived via trial and error
        
  def listener_callback(self, data):
    #self.get_logger().info('Receiving video frame')
    
    #if first loop, set past time for Kalman filter dt value
    if self.current_time == None:
    	self.past = time() - 0.03 #estimate dt value derived from /color/preview/image frequency = ~30hz
    else:
    	self.past = self.current_time #else derive dt from last time /blobs topic updated
   
    #time variables for Kalmann and Matplotlib live plot	
    self.current_time = time()
    if not self.t0:
        self.t0 = self.current_time

    t = self.current_time-self.t0
    
    #Read Published Message data, convert to str arr
    valArr = ['','','','']
    index = 0
    for i,j in enumerate(data.data):
        #coded to only read highest priority blob Red > Blue > Green
        if index > 3:
            break
        if j == '\n':
            index = index + 1;
            continue
        else:
            valArr[index] = valArr[index] + j            
        
              
    
    
    
    #Create new Twist message for publisher
    twist = Twist()
    
    #setup history if first loop
    if self.history is None:
        self.history = np.zeros((MEMORY,3))
    
    #If blob found:
    if valArr[1] != '':
        
        #convert blob x coordinate value from str to float
        x_value = float(valArr[1])
        
        #Pass found X_value to kalman filter with curent time, dt, and  live plot history
        dt = self.current_time - self.past
        self.history[:-1,:] = self.history[1:,:]
        filterOut = filter(x_value,self.count,dt)
        self.count = self.count+1
        self.history[-1,:] = np.array([t, x_value, filterOut[0][0]])
        self.update(1)
        
        #determine offset from midline. ASSUMING: 250x250px size /color/preview/image 
        delta =  (250/2) - filterOut[0][0]
        
              
        print(valArr)
        
        #make custom twist code based on color and size of blob found     
        if valArr[0] == 'red':         #Rotational and Linear Motion with Live Plotting
            self.colorFound = 'Red'
            #sets angular deadzone of +/-15px around midline, determined from delta
            if abs(delta) > 5:        #Just Rotational motion with live plotting
                twist.angular.z = self.angularKp * delta
            
            #Sets linear deadzone based on size of blob found 75-85px diameter
            if float(valArr[3]) > 62 or float(valArr[3]) < 58: 
                linear = self.linearKp * (80 - float(valArr[3]))
                #print(valArr[3])
                #Set min and max linear speed
                if linear > 0: #positive linear velocity
                    if linear < 0.1:
                        linear = 0
                    elif linear > 0.3:
                        linear = 0.3
                else:          #negative linear velocity
                    if linear < -0.3:
                        linear = -0.3
                    elif linear > -0.1:
                        linear = 0
                twist.linear.x = float(linear)
        elif valArr[0] == 'blue':
            self.colorFound = 'Blue'
            if abs(delta) > 5:
                twist.angular.z = self.angularKp * delta 
        elif valArr[0] == 'green':
    	    self.colorFound = 'Green'     
    
    if self.colorFound == 'Red' or self.colorFound == 'Blue':
        self.publisher_.publish(twist)
    cv2.waitKey(10)
    
  def update(self,i):

        if self.fig is None:
            first = True
            self.fig = plt.figure()
            self.ax = plt.subplot(111)
        else:
            first = False
        self.ax.cla()
        self.ax.plot(self.history[:,0],self.history[:,1], color='r', label='raw')
        self.ax.plot(self.history[:,0],self.history[:,2], color='b', label='Kalman')
        self.ax.set_xlabel('Time(s)')
        self.ax.set_ylabel(self.colorFound + 'Blob Coordinate - X(px)')
        self.ax.set_title('Kalman Filtered Livestreamed ' + self.colorFound + ' Blob Tracker/Follower')
        self.ax.legend(['Raw', 'Kalman'],loc = "upper left")
        self.ax.set_ylim(self.history[:,1].min(),self.history[:,1].max())    # plot memory
        self.fig.canvas.draw()
        if first:
            plt.ion()
            plt.show()

def main(args=None):
  previous = time()
  delta = 0
  rclpy.init(args=args)
  image_subscriber = ImageSubscriber()
  rclpy.spin(image_subscriber)
  image_subscriber.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()


#kalman Filter  
def filter(z, updateNumber,deltaT):
    
    #define dt based on 
    dt = deltaT
    
    # Initialize State
    if updateNumber == 1:
        filter.x = np.array([[0],
                            [20]])
        filter.P = np.array([[5, 0],
                                 [0, 5]])
        filter.A = np.array([[1, dt],
                             [0, 1]])
        filter.H = np.array([[1, 0]])
        filter.HT = np.array([[1],
                              [0]])
        filter.R = 5 #Found R value of 5 to be the best
        filter.Q = np.array([[1, 0],
                             [0, 5]])
    # Predict State Forward
    x_p = filter.A.dot(filter.x)
    # Predict Covariance Forward
    P_p = filter.A.dot(filter.P).dot(filter.A.T) + filter.Q
    # Compute Kalman Gain
    S = filter.H.dot(P_p).dot(filter.HT) + filter.R
    K = P_p.dot(filter.HT)*(1/S)
    # Estimate State
    residual = z - filter.H.dot(x_p)
    filter.x = x_p + K*residual
    # Estimate Covariance
    filter.P = P_p - K.dot(filter.H).dot(P_p)
    return [filter.x[0], filter.x[1], filter.P];
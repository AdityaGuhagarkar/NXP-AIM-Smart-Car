import rclpy
from rclpy.node import Node

from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
from nxp_cup_interfaces.msg import PixyVector
from time import sleep
from datetime import datetime
import numpy as np


class LineFollow(Node):

    def __init__(self):
        super().__init__('aim_line_follow')

        self.flag_data = Float64()
        
        self.laser_data = LaserScan()
        
        self.start_delay = 15.0 
        
        self.camera_vector_topic = "/cupcar0/PixyVector"
        
        self.linear_velocity = 0.98
        
        self.angular_velocity = -1.4
        
        self.single_line_steer_scale = 1.0 
        
        self.i = 0
     

        # Time to wait before running
        self.get_logger().info('Waiting to start for {:s}'.format(str(self.start_delay)))
        sleep(self.start_delay)
        self.get_logger().info('Started')

        self.start_time = datetime.now().timestamp()
        self.restart_time = True

        # Subscribers
        self.pixy_subscriber = self.create_subscription(
            PixyVector,
            self.camera_vector_topic,
            self.listener_callback,
            10)
            
            
        self.LaserScanSub = self.create_subscription(
            LaserScan,
            "/scan",
            self.listener_callback_laser,
            10)
            
        self.FlagSub = self.create_subscription(
            Float64,
            "/flags",
            self.listener_callback_flag,
            10)
            
        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cupcar0/cmd_vel', 10)

        self.speed_vector = Vector3()
        self.steer_vector = Vector3()
        self.cmd_vel = Twist()

        # Timer setup
        # timer_period = 0.5 #seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 1

    def get_num_vectors(self, msg):
        num_vectors = 0
        if(not(msg.m0_x0 == 0 and msg.m0_x1 == 0 and msg.m0_y0 == 0 and msg.m0_y1 == 0)):
            num_vectors = num_vectors + 1
        if(not(msg.m1_x0 == 0 and msg.m1_x1 == 0 and msg.m1_y0 == 0 and msg.m1_y1 == 0)):
            num_vectors = num_vectors + 1
        return num_vectors

    def listener_callback_laser(self, msg1):
        self.laser_data = msg1
        
    def listener_callback_flag(self, msg2):
        self.flag_data = msg2

    def listener_callback(self, msg):
        #TODO
        current_time = datetime.now().timestamp()
        frame_width = 79
        frame_height = 52
        window_center = (frame_width / 2)
        x = 0
        y = 0
        steer = 0
        speed = 0
        num_vectors = self.get_num_vectors(msg)
        
        counter = 0.0      

        if(num_vectors == 1):
            if not self.restart_time:
                self.start_time = datetime.now().timestamp()
                self.restart_time = True
            if(msg.m0_x1 > msg.m0_x0):
                x = (msg.m0_x1 - msg.m0_x0) / frame_width
                y = (msg.m0_y1 - msg.m0_y0) / frame_height
            else:
                x = (msg.m0_x0 - msg.m0_x1) / frame_width
                y = (msg.m0_y0 - msg.m0_y1) / frame_height
            if(msg.m0_x0 != msg.m0_x1 and y != 0):
                steer = (-self.angular_velocity) * (x / y) * self.single_line_steer_scale
                if (self.start_time+0.1) > current_time:
                    speed = self.linear_velocity * ((current_time-self.start_time)/0.1)
                if (self.start_time+0.1) <= current_time:
                    speed = self.linear_velocity
            else:
                steer = 0
                if (self.start_time+0.1) > current_time:
                    speed = self.linear_velocity * ((current_time-self.start_time)/0.1)*0.9
                if (self.start_time+0.1) <= current_time:
                    speed = self.linear_velocity*0.9

        if(num_vectors == 2):
            m_x1 = (msg.m0_x1 + msg.m1_x1) / 2.0
            m_x0 = (msg.m0_x0 + msg.m1_x0) / 2.0   #Added by me
            CTEf = float(m_x1 - window_center)
            Jp = self.angular_velocity
            #Jd = 0.000001
            #steer = Jp*(CTEf) + Jd * (CTEf - CTEi)/(timeF - timeI)           PD caused instability due to single track... so let's stick to P
            steer = Jp*(CTEf)
            CTEi = CTEf         #CTEi is a variable purely for testing purposes to reduce cross track error
            
                
            if not self.restart_time:
                self.start_time = datetime.now().timestamp()
                self.restart_time = True
            
            steer = self.angular_velocity*(m_x1 - window_center) / frame_width
            if (self.start_time+0.1) > current_time:
                speed = self.linear_velocity * ((current_time-self.start_time)/4.0)
            if (self.start_time+0.1) <= current_time:
                speed = self.linear_velocity

        self.speed_vector.x = float(speed)*float(1-min(np.abs(1.3*steer), 0.25))
        
        if(num_vectors == 0):
            if(self.i<800):
                self.speed_vector.x = 0.17
                steer = -0.9
            
            else:
                self.speed_vector.x = 0.17
                steer = steer - 0.16

        finale1=0
        finale2=0    
        r=0
        for r in range(99):
            z = -0.6 + r*(1.2/99.0)
            finale1 = finale1 + (z*z*z - 0.6*0.6*z)/((self.laser_data).ranges[r])
            finale2 = finale2 + (z*z-0.6*0.6)/((self.laser_data).ranges[r])
            
        if finale2 == 0 :
            finale = 0
        else : 
            finale = finale1/finale2
            
        steer = steer - finale/1.8
        self.i = self.i + 1
        print(self.i)
        if self.flag_data.data>0.0 or (self.i>1100 and self.i<1150):
            steer = steer - 0.5
            #self.speed_vector.x = 0.25
            print("YYYYYEEEESSSSSSS.............................")
                        
        self.steer_vector.z = float(steer)

        self.cmd_vel.linear = self.speed_vector
        self.cmd_vel.angular = self.steer_vector

        self.cmd_vel_publisher.publish(self.cmd_vel)

def main(args=None):
    rclpy.init(args=args)

    line_follow = LineFollow()

    rclpy.spin(line_follow)

    line_follow.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

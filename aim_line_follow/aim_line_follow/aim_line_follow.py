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

        self.starter = 0

        self.flag_data = Float64()
        
        self.laser_data = LaserScan()
        
        self.start_delay = 15.0 
        
        self.camera_vector_topic = "/cupcar0/PixyVector"
        
        self.linear_velocity = 0.95
        
        self.angular_velocity = -1.4
        
        self.single_line_steer_scale = 1.0 
        
        self.i = 0
        
        self.j = 0
        
        self.keep_moving_forward = 0
        
        self.delayer = 0
     
        self.delayer2 = 0
        
        self.delayer3 = 0

        self.brown_mode = 0
        
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
        
        if(num_vectors == 1 or num_vectors == 2):
            self.j = self.j + 1
              
        if(self.j > 100):
            self.keep_moving_forward = 0
        
        if(self.starter == 1):
            self.i = self.i + 1
            
        
        if(num_vectors == 1 and self.keep_moving_forward == 0):
            
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

        if(num_vectors == 2 and self.keep_moving_forward == 0):
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
            
        if(self.i>380 and self.i<390):               #😀️😀️😀️😀️😀️
            self.speed_vector.x = 0.10
        
            steer = 5.0
        
        if((int)((self.flag_data).data / 10.0) % 10 == 1 and self.i>400):
            self.delayer = self.delayer + 1
        
        if(self.i > 400 and self.delayer > 20 and self.delayer < 23):
            self.speed_vector.x = 0.18
            steer = -0.6
            steer = steer
        
        if(self.i == 220):
            self.linear_velocity = 0.85
        
        if(num_vectors == 0):
            self.starter = 1
            if(self.i<230):
                self.speed_vector.x = 0.17
                steer = -0.9
            
            elif(self.i<480):              
                self.speed_vector.x = 0.17
                steer = steer - 0.175  
            
            elif(self.keep_moving_forward == 1 or self.i<10000):
                self.speed_vector.x = 0.5
                steer = 0.0
                self.keep_moving_forward = 1
                self.j = 0
        
        finale1=0
        finale2=0
        r=0
        e=2.71828
        for r in range(99):
            z = -0.6 + r*(1.2/99.0)
            finale1 = finale1 + (e**(-np.abs(z))-e**(-0.6))*z/((self.laser_data).ranges[r])
            finale2 = finale2 + (e**(-np.abs(z))-e**(-0.6))/((self.laser_data).ranges[r])
            
        if finale2 == 0 :
            finale = 0
        else :
            finale = finale1/finale2
        
         
        #if(self.i>525 and self.i<535) or (self.flag_data).data == 10:
            #steer = -2.0
        
        #if(self.i>535 and self.i<555):
        #    steer = 0.0
        #    self.speed_vector.x = 0.9
        
        if((int)((self.flag_data).data) == 100 and self.i>500):
            self.speed_vector.x = 0.65
            self.brown_mode = 1
            
        if(self.brown_mode == 1 and (int)((self.flag_data).data / 10000.0) == 1):
            self.delayer3 = self.delayer3 + 1
            if (self.delayer3 > 20):
                steer = -3.0
                self.delayer2 = self.delayer2 + 1
                self.speed_vector.x=0.5
                
        if(self.brown_mode == 1 and (int)((self.flag_data).data / 10000.0) == 1 and self.delayer2 > 25):
            self.brown_mode = 0
            self.speed_vector.x = 0.9
            
        
        #if(self.i>555 and self.i<565):
            #steer = -2.0
            #self.speed_vector.x = 1.25
            
        #else:
        if(self.brown_mode == 1):
            steer = steer
        else:
            steer = steer - finale/1.8
        print(self.i)
            
            
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
#570
if __name__ == '__main__':
    main()

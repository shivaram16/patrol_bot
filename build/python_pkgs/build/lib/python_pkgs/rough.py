import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute,SetPen
import math

class DrawDrone(Node):
    def __init__(self):
        super().__init__("drone_image")

        self.teleport_client_ = self.create_client(TeleportAbsolute,'/turtle1/teleport_absolute')
        self.set_pen_client_ = self.create_client(SetPen,'/turtle1/set_pen')
        self.points_publisher_ = self.create_publisher(Twist,"/turtle1/cmd_vel",10)
        self.points_subscriber_ = self.create_subscription(Pose,"/turtle1/pose",self.PoseCallback,10)
        self.coordinates_ = Twist()
        self.pose_msg = Pose()
        self.coordinate_values_circle = [[2.0,3.0],[2.0,9.0],[8.0,9.0],[8.0,3.0]]
        self.coordinate_values_square = [[3.0,5.0],[5.0,7.0],[7.0,5.0],[5.0,3.0]]
        self.circle_co_val_ptr = 0
        self.line_count = 0
        self.square_line_count =0
        self.x_teleport_value = None
        self.y_teleport_value = None
        self.publish_flag = True
        self.allow_teleport = True
        self.timer_set_flag = False
        self.gp1 = None
        self.gp2 = None
        self.pen_state = True
        self.four_circles_timer_ = self.create_timer(0.1,self.FourCircles)

    
    def PenOffFunct(self):
        print(self.pose_msg.x - 4.0)
        print(self.pose_msg.y - 4.0)
        if(((self.pose_msg.x - 4.0) > 0 and (self.pose_msg.y - 4.0) > 0) and ((self.pose_msg.x - 4.0) < 2.0 and (self.pose_msg.y - 4.0) < 2.0)):
                    while not self.set_pen_client_.wait_for_service(timeout_sec=1.0):
                        self.get_logger().info("waiting for the SetPen service ")
                    self.SetPenState(off=True)
                    self.pen_state = False
        else : 
                if(not self.pen_state):
                    self.SetPenState(off=False)
                    self.pen_state = True   
        #                      
        # elif(abs(self.pose_msg.x - (self.coordinate_values_square[2][0] - 3.0 )) < 0.03 and abs(self.pose_msg.y - (self.coordinate_values_square[2][1] - 3.0)) < 0.03):
        #             while not self.set_pen_client_.wait_for_service(timeout_sec=1.0):
        #                 self.get_logger().info("waiting for the SetPen service ")
        #             self.SetPenState(off=True)
        #             self.pen_state = False
                    
        # else:
        #         if(not self.pen_state):
        #             self.SetPenState(off=False)
        #             self.pen_state = True
                         


    def DrawCross(self):
        # print("DrawCross called")
        # self.coordinates_.linear.x = 0.0
        # self.coordinates_.angular.z = -0.0
        # self.publish_flag=True
        # self.points_publisher_.publish(self.coordinates_)
        if(self.allow_teleport and self.line_count == 0):
            cir_co = self.coordinate_values_circle[0]
            x_p = cir_co[0]
            y_p = cir_co[1] - 1.0
            self.TeleportRequestFunct(x_p,y_p,angle = math.radians(45.0))
            self.allow_teleport = False
            self.gp1 = [self.coordinate_values_circle[2][0] ,self.coordinate_values_circle[2][1] - 1.0 ]
        elif(self.allow_teleport and self.line_count == 1):
            cir_co = self.coordinate_values_circle[1]
            x_p = cir_co[0]
            y_p = cir_co[1] - 1.0
            self.TeleportRequestFunct(x_p,y_p,angle = math.radians(-45.0))
            self.allow_teleport = False
            self.gp2 = [self.coordinate_values_circle[3][0] ,self.coordinate_values_circle[3][1] - 1.0 ]

        # if(abs(self.pose_msg.x - (self.coordinate_values_square[0][0] + 3.0 )) < 0.03 and abs(self.pose_msg.y - (self.coordinate_values_square[0][1] + 3.0)) < 0.03):
        #     while not self.set_pen_client_.wait_for_service(timeout_sec=1.0):
        #         self.get_logger().info("waiting for the SetPen service ")
        #     self.SetPenState(off=True)
        #     teleport_request = TeleportAbsolute.Request()
        #     resp = self.teleport_client_.call_async(teleport_request)
        #     resp.add_done_callback(self.TeleportCallBack)
        # elif(abs(self.pose_msg.x - (self.coordinate_values_square[2][0] - 3.0 )) < 0.03 and abs(self.pose_msg.y - (self.coordinate_values_square[2][1] - 3.0)) < 0.03):
        #     while not self.set_pen_client_.wait_for_service(timeout_sec=1.0):
        #         self.get_logger().info("waiting for the SetPen service ")
        #     self.SetPenState(off=True)
        #     teleport_request = TeleportAbsolute.Request()
        #     resp = self.teleport_client_.call_async(teleport_request)
        #     resp.add_done_callback(self.TeleportCallBack)
        # else:
        #     self.SetPenState(off=False)
        #     teleport_request = TeleportAbsolute.Request()
        #     resp = self.teleport_client_.call_async(teleport_request)
        self.PenOffFunct()
        self.StraightLine(goal1=self.gp1,goal2=self.gp2,allow_line_count=True) 

    def StraightLine(self,goal1=[0,0],goal2=[0,0],allow_line_count = True):
        if(self.line_count == 0):
            self.coordinates_.linear.x = 1.0
            self.coordinates_.angular.z = 0.0
            # print(abs(self.pose_msg.x - goal1[0]))
            # print(abs(self.pose_msg.y - goal1[1]))
            if (abs(self.pose_msg.x - goal1[0]) < 0.03 and abs(self.pose_msg.y - goal1[1]) < 0.03):
                self.coordinates_.linear.x = 0.0
                self.publish_flag = False
            if(self.publish_flag):
                self.points_publisher_.publish(self.coordinates_)
            else:
                self.allow_teleport = True
                self.publish_flag = True
                self.coordinates_.linear.x = 0.0
                self.points_publisher_.publish(self.coordinates_)
                # self.get_logger().info("Killing Cross Timer")
                if(self.square_line_count == 4):
                    self.square_line_count = 0
                    self.allow_teleport = True
                    self.publish_flag = True
                    self.coordinates_.linear.x = 0.0
                    self.points_publisher_.publish(self.coordinates_)
                    self.get_logger().info("Killing square Timer")
                    self.square_timer_.cancel()
                    self.cross_timer = self.create_timer(0.05,self.DrawCross)
                if(allow_line_count):
                    self.line_count+=1
                # self.cross_timer_.cancel()
        elif(self.line_count == 1):
            self.coordinates_.linear.x = 1.0
            self.coordinates_.angular.z = 0.0
            print(abs(self.pose_msg.x - goal2[0]))
            print(abs(self.pose_msg.y - goal2[1]))
            if (abs(self.pose_msg.x - goal2[0]) < 0.02 and abs(self.pose_msg.y - goal2[1]) < 0.02):
                self.coordinates_.linear.x = 0.0
                self.publish_flag = False
            if(self.publish_flag):
                self.points_publisher_.publish(self.coordinates_)
            else:
                self.allow_teleport = True
                self.publish_flag = True
                self.coordinates_.linear.x = 0.0
                self.points_publisher_.publish(self.coordinates_)
                self.get_logger().info("Killing Cross Timer")
                self.line_count+=1
                self.cross_timer.cancel()
                self.TeleportRequestFunct(5.0,5.0,angle= 0.0)
        
   

        

    def FourCircles(self):
            if(self.allow_teleport):
                cir_co = self.coordinate_values_circle[self.circle_co_val_ptr]
                x_p = cir_co[0]
                y_p = cir_co[1]
                self.TeleportRequestFunct(x_p,y_p,angle = math.radians(0.0))
                self.allow_teleport = False
            self.DrawCircle()


    def TeleportRequestFunct(self,x_p = 0.0,y_p = 0.0,angle = 55.0):

        self.x_teleport_value = x_p
        self.y_teleport_value = y_p

        while not self.set_pen_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("waiting for the SetPen service ")

        self.SetPenState(off=True)
         
        while not self.teleport_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for teleport service to be available...')

        teleport_request = TeleportAbsolute.Request()
        teleport_request.x = x_p
        teleport_request.y = y_p
        teleport_request.theta = angle


        resp = self.teleport_client_.call_async(teleport_request)
        self.pen_state = False
        resp.add_done_callback(self.TeleportCallBack)
        #self.get_logger().info('teleport coordinates are x: %f, y: %f'%(x_p,y_p))
        
    
    def TeleportCallBack(self,resp):
        try:
            response = resp.result()
            self.get_logger().info("teleport service called successfully")
            self.SetPenState(off=False)
            self.pen_state = True
        except:
            self.get_logger().info("unable to call teleport service")
    
    def SetPenState(self,off):
        pen_req = SetPen.Request()
        pen_req.off = int(off)
        resp = self.set_pen_client_.call_async(pen_req)

    def PoseCallback(self,msg):
        self.pose_msg = msg

    # def stop_timer(self):
    #     self.circle_timer_.cancel()  # Cancel the timer
    #     self.get_logger().info('Timer stopped!')
    def DrawSquare(self):
        if(self.allow_teleport):
            cir_co = self.coordinate_values_square[self.circle_co_val_ptr]
            x_p = cir_co[0]
            y_p = cir_co[1]
            if(self.circle_co_val_ptr == 0):
                teleport_angle = 45.0
            elif(self.circle_co_val_ptr == 1):
                teleport_angle = -45.0
            elif(self.circle_co_val_ptr == 2):
                teleport_angle = -135.0
            else:
                teleport_angle = 135.0
                self.circle_co_val_ptr = -1
            self.TeleportRequestFunct(x_p,y_p,angle = math.radians(teleport_angle))
            self.allow_teleport = False
            self.circle_co_val_ptr+=1
            self.square_line_count+=1
            self.gp1 = [self.coordinate_values_square[self.circle_co_val_ptr][0] ,self.coordinate_values_square[self.circle_co_val_ptr][1] ]
        self.StraightLine(self.gp1,allow_line_count=False)
        
        

    def DrawCircle(self):
            self.coordinates_.linear.x = 1.0
            self.coordinates_.angular.z = -1.0
            #print(self.pose_msg.x - self.x_teleport_value)
            if (abs(self.pose_msg.x - self.x_teleport_value) < 0.03 and abs(self.pose_msg.y - self.y_teleport_value)<0.03):
                self.coordinates_.linear.x = 0.0
                if(abs(self.pose_msg.theta) < math.radians(45.0)):
                    self.coordinates_.angular.z = 0.0
                    self.publish_flag=False
            
            if(self.publish_flag):
                self.points_publisher_.publish(self.coordinates_)
            else:
                self.get_logger().info("publishing flag down and turning up")
                self.publish_flag=True  
                self.allow_teleport = True
                if(self.circle_co_val_ptr > 2):
                    self.get_logger().info("Killing circle Timer")
                    self.four_circles_timer_.cancel()
                    self.circle_co_val_ptr = 0
                    self.coordinates_.linear.x = 0.0
                    self.coordinates_.angular.z = -0.0
                    self.allow_teleport = True
                    self.points_publisher_.publish(self.coordinates_)
                    self.square_timer_ = self.create_timer(0.05,self.DrawSquare)
                else:
                    self.circle_co_val_ptr+=1
        


            
              


def main():
    rclpy.init()
    draw_drone = DrawDrone()
    rclpy.spin(draw_drone)
    draw_drone.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

#!/user/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from turtle_catcher_interfaces.msg import ActiveTurtles
from turtle_catcher_interfaces.srv import CatchTurtle
from turtlesim.msg import Pose
import random

class MasterTurtleNode(Node):
    def __init__(self):
        super().__init__('master_turtle') 
        self.get_logger().info(f"Master turtle controller setting up....")
        self.nearest_turtle= ActiveTurtles()
        self.active_turtles= ActiveTurtles()
        self.position_data=Pose()
        self.nearest_distance=100
        self.trajectory_angle=0.0
        self.chase_turtle_client= self.create_client(CatchTurtle, 'chase_turtle')
        self.velocity_publisher= self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.position_subscriber=self.create_subscription(Pose, 'turtle1/pose', self.process_position, 10)
        self.turtle_data= self.create_subscription(ActiveTurtles, 'active_turtles', self.process_turtle_information, 10)
        self.timer_= self.create_timer(0.02, self.publish_velocity)

    def process_position(self, data):
        self.position_data=data

    def process_turtle_information(self, data):
        self.active_turtles= data.active_turtles
        if self.active_turtles:
            turtle = self.find_closest_turtle()
            x_pos= turtle.x
            y_pos = turtle.y
            master_x_pos = self.position_data.x
            master_y_pos = self.position_data.y
            self.trajectory_angle= np.arctan2((y_pos-master_y_pos), (x_pos - master_x_pos))
            self.nearest_distance= self.calculate_distance(x_pos, master_x_pos, y_pos, master_y_pos)
    
    def find_closest_turtle(self):
        self.nearest_turtle= self.active_turtles[0]
        for turtle in self.active_turtles:
            calculated_distance=self.calculate_distance(turtle.x, self.position_data.x, turtle.y, self.position_data.y)
            if calculated_distance < self.nearest_distance:
                self.nearest_turtle= turtle 
                self.nearest_distance= calculated_distance
        return self.nearest_turtle

    def publish_velocity(self):
        velocity_data= Twist()
        master_theta= self.position_data.theta
        angle_diff= self.trajectory_angle - master_theta
        normalized_diff= np.arctan2(np.sin(angle_diff), np.cos(angle_diff)) 
        ANGLE_TOLERANCE= 0.02
        KP_ANGLE=4
        KP_MOVEMENT=1.5

        if normalized_diff>0:
            if normalized_diff > ANGLE_TOLERANCE:
                velocity_data.angular.z = KP_ANGLE * normalized_diff
        elif normalized_diff<0:
            if abs(normalized_diff) > ANGLE_TOLERANCE:
                velocity_data.angular.z = -1 * KP_ANGLE * abs(normalized_diff)

        if isinstance(self.nearest_distance, float):
            velocity_data.linear.x=max((KP_MOVEMENT * self.nearest_distance), 1.5)
            if(abs(self.nearest_distance)<0.3):
                self.turtle_chased_down()
        self.velocity_publisher.publish(velocity_data)
    
    def turtle_chased_down(self):
        while not self.chase_turtle_client.wait_for_service(1):
            self.get_logger().info("Waiting for killing turtle service....")

        request= CatchTurtle.Request()
        if self.active_turtles:
            request.name= (self.nearest_turtle).name
            future= self.chase_turtle_client.call_async(request)
            # future.add_done_callback(self.callback_turtle_chased_down)
    
    # def callback_turtle_chased_down(self, future):
    #     future.result()

    def calculate_distance(self, x1, x2, y1, y2):
        return ((x1-x2)**2 + (y1-y2)**2)**0.5
        
def main(args=None):
    rclpy.init(args=args)
    node= MasterTurtleNode() 
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()
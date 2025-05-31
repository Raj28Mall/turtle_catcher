#!/user/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill
from geometry_msgs.msg import Twist
from turtle_catcher_interfaces.msg import ActiveTurtles, TurtleInfo
from turtle_catcher_interfaces.srv import CatchTurtle
import random
from functools import partial

class SpawnerTurtleNode(Node):
    def __init__(self):
        super().__init__('spawner_turtle')
        self.get_logger().info("Spawner Turtle Node has been initialized.")
        self.active_turtles=[]
        self.spawn_turtle= self.create_client(Spawn, 'spawn')
        self.kill_turtle=self.create_client(Kill, 'kill')
        self.chased_down_turtle_service= self.create_service(CatchTurtle, 'chase_turtle', self.process_turtle_death)
        self.active_turtles_publisher_=self.create_publisher(ActiveTurtles, 'active_turtles', 10)
        self.publish_active_turtles_timer_=self.create_timer(0.02, self.publish_active_turtles)
        self.spawner_timer_=self.create_timer(1.0, self.spawn_new_turtle)

        if not self.spawn_turtle.wait_for_service(1):
            self.get_logger().error("Spawn service not available!")
            return
        
        if not self.kill_turtle.wait_for_service(1):
            self.get_logger().error("Kill service not available!")
            return
        
    def publish_active_turtles(self):
        active_turtles_msg=ActiveTurtles()
        active_turtles_list=[]

        for turtle in self.active_turtles:
            turtle_info= TurtleInfo()
            turtle_info.name = turtle["name"]
            turtle_info.x = turtle["x"]
            turtle_info.y = turtle["y"]
            active_turtles_list.append(turtle_info)
        active_turtles_msg.active_turtles=active_turtles_list

        self.active_turtles_publisher_.publish(active_turtles_msg)

    def spawn_new_turtle(self):
        """Initiates a request to spawn a new turtle."""
        random_x= float(random.uniform(2.0, 9.0)) 
        random_y= float(random.uniform(2.0, 9.0)) 

        request = Spawn.Request()
        request.x = random_x
        request.y = random_y
        request.theta = float(random.uniform(0.0, 6.28)) 

        future = self.spawn_turtle.call_async(request)
        future.add_done_callback(partial(self.callback_spawn_turtle, random_x, random_y))
    
    def callback_spawn_turtle(self, random_x, random_y, future):
        response= future.result()
        if not response.name:
            self.get_logger().warn(f"Response had no name for turtle!")

        turtle_information={'name':response.name, 'x':random_x, 'y':random_y}
        self.active_turtles.append(turtle_information)
    
    def kill_named_turtle(self, name):
        """Initiates a request to kill a specific turtle."""
        if not self.active_turtles:
            self.get_logger().info("No turtles to kill right now")
            return

        request= Kill.Request()
        request.name=name
        future = self.kill_turtle.call_async(request)
        future.add_done_callback(self.callback_kill_turtle)
    
    def callback_kill_turtle(self, future):
        future.result()
    
    def process_turtle_death(self, request: CatchTurtle.Request, response: CatchTurtle.Response):
        turtle_name = request.name
        for turtle in self.active_turtles:
            if(turtle_name==turtle["name"]):
                response.success=True
                self.kill_named_turtle(turtle_name)
                self.active_turtles.remove(turtle)
                return response
        response.success=False
        response.debug_message="Turtle to be deleted not found in list"
        return response
        
def main(args=None):
    rclpy.init(args=args)
    node= SpawnerTurtleNode() 
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()
#!/usr/bin/env python3
import rclpy
import math
import random
from rclpy.node import Node
from trial_interfaces.srv import Killer
from trial_interfaces.msg import AliveStatus
from turtlesim.srv import SetPen
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
 
 
class TurtleGodNode(Node): 
    def __init__(self):
        super().__init__("turtle_god")


        self.t_ = float(0.0)
        self.alive_count_ = int(0)
        self.Turtle_bio_ = AliveStatus()
        self.kill_state_ = bool(True)


        self.client_spawn = self.create_client(Spawn,"/spawn")
        self.client_kill = self.create_client(Kill,"/kill")
        self.server_ = self.create_service(Killer,"killer",self.Callback)
        self.publisher_ = self.create_publisher(AliveStatus,"Alive_Turtles",10)
        self.timer_ = self.create_timer(0.1,self.last_time_state_changed)
        self.timer_1 = self.create_timer(0.1,self.Status_Publisher)
        self.get_logger().info("Spawning Service is online!! ")
 


    def get_current_time(self):
        seconds , nanoseconds = self.get_clock().now().seconds_nanoseconds()
        return seconds + nanoseconds/1000000000.0

    def last_time_state_changed(self):
        now = self.get_current_time()
        if (now - self.t_ >= 2.00):
            if (self.alive_count_ < 5):
                self.Spawn()
            self.t_ = now

    def Spawn(self):
        while not self.client_spawn.wait_for_service(1.0):
            self.get_logger().info("Server not found trying to connect.... ")
        
        request_ = Spawn.Request()
        request_.x =  random.uniform(1.0,10.0)
        request_.y =  random.uniform(1.0,10.0)
        request_.theta =  random.uniform(0.0 , 2*math.pi)

        self.Turtle_bio_.x.append(request_.x)
        self.Turtle_bio_.y.append(request_.y)
        self.Turtle_bio_.theta.append(request_.theta)

        future = self.client_spawn.call_async(request_)
        future.add_done_callback(self.call_Spawn)

    def call_Spawn(self , future):
        response_ = future.result() 
        self.Turtle_bio_.name.append(str(response_.name))

    def Status_Publisher(self):
        msg = AliveStatus()
        msg.name = list(self.Turtle_bio_.name)
        msg.x = list(self.Turtle_bio_.x)
        msg.y = list(self.Turtle_bio_.y)
        msg.theta = list(self.Turtle_bio_.theta)
        self.publisher_.publish(msg)

        self.alive_count_ = len(self.Turtle_bio_.name)

    def Callback(self,request: Killer.Request, response: Killer.Response):
        if request.kill:
            if self.kill_state_:
                self.kill_state_ = False
                self.Kill(request.kill_name)
                

        response.debbug = "the turtle: "+request.kill_name+" has been killed! "
        return response
    
    def Kill(self,name_:str):
        while not self.client_kill.wait_for_service(1.0):
            self.get_logger().info("Server not found trying to connect.... ")
        
        request_ = Kill.Request()
        request_.name = name_

        if name_ not in self.Turtle_bio_.name:
            self.get_logger().warn(f"Turtle {name_} already dead")
            return

        idx = self.Turtle_bio_.name.index(str(name_))
        self.Turtle_bio_.name.pop(idx)
        self.Turtle_bio_.x.pop(idx)
        self.Turtle_bio_.y.pop(idx)
        self.Turtle_bio_.theta.pop(idx)

        future = self.client_kill.call_async(request_)
        future.add_done_callback(self.call_kill)

    def call_kill(self , future):
        response_ = future.result()
        self.kill_state_ = True      

 
def main(args=None):
    rclpy.init(args=args)
    node = TurtleGodNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()

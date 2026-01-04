#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from trial_interfaces.msg import AliveStatus
from trial_interfaces.srv import Killer
from geometry_msgs.msg import Twist
from  turtlesim.msg import Pose

class TurtleControllerNode(Node): 
    def __init__(self):
        super().__init__("turtle_controller")

        self.declare_parameter("P_v",5.0)
        self.declare_parameter("P_A",0.7)
        self.declare_parameter("timer",0.1)
        self.V_ = self.get_parameter("P_v").value
        self.A_ = self.get_parameter("P_A").value
        self.timer = self.get_parameter("timer").value

        self.Active_turtles_ = AliveStatus()
        self.current_pose_ = Pose()
        self.Dist_ = []
        self.Target_ = str("")
        self.Target_set_ = bool(False)
        self.idx_ = int(0)
        self.kill_in_progress = bool(False)

        self.client_ = self.create_client(Killer,"killer")
        self.publisher_ = self.create_publisher(Twist,"/turtle1/cmd_vel",10)
        self.sub1_ = self.create_subscription(AliveStatus,"Alive_Turtles",self.status_acquirer,10)
        self.sub2_ = self.create_subscription(Pose,"/turtle1/pose",self.get_current_pose,10)
        self.timer_ = self.create_timer(self.timer,self.Chase)

    def status_acquirer(self, msg= AliveStatus):
        self.Active_turtles_= msg
    
    def get_current_pose(self,msg= Pose):
        self.current_pose_ = msg

    def set_target(self):
        if self.Target_set_ == False:
            if self.Active_turtles_.name:
                self.Dist_.clear()
                for i in range(len(self.Active_turtles_.name)):
                    dx = self.Active_turtles_.x[i] - self.current_pose_.x
                    dy = self.Active_turtles_.y[i] - self.current_pose_.y
                    self.Dist_.append(math.sqrt(dx*dx + dy*dy))

                self.Target_ = self.Active_turtles_.name[self.Dist_.index(min(self.Dist_))]
                self.idx_ = self.Active_turtles_.name.index(self.Target_)
                self.Target_set_ = True
            else:
                self.get_logger().info("Their are no targets availabe currently")

    def Chase(self):

        self.set_target()

        if not self.Target_set_:
            return

        if self.Target_ not in self.Active_turtles_.name:
            self.Target_set_ = False
            return


        dx = self.Active_turtles_.x[self.idx_]-self.current_pose_.x
        dy = self.Active_turtles_.y[self.idx_]-self.current_pose_.y
        error_dist_ = math.sqrt(dx*dx + dy*dy)
        desired_theta = math.atan2(dy, dx)

        error_angle = desired_theta - self.current_pose_.theta
        alpha = math.atan2(math.sin(error_angle), math.cos(error_angle))

        rotate_ = self.A_ * alpha
        vel_ = self.V_ * error_dist_

        if abs(alpha) > 0.5:
            vel_ = 0.0

        if error_dist_ < 0.5:
            vel_ = 0.0
            rotate_ = 0.0
            if not self.kill_in_progress:
                self.Target_destroy()
                self.kill_in_progress = True

        msg = Twist()
        msg.linear.x = float(vel_)
        msg.angular.z = float(rotate_)
        self.publisher_.publish(msg)

    def Target_destroy(self):
        while not self.client_.wait_for_service(1.0):
            self.get_logger().info("Waiting for server... ")
        request = Killer.Request()
        request.kill = True
        request.kill_name = self.Target_

        future = self.client_.call_async(request)
        future.add_done_callback(self.call_to_turtle_god)

    def call_to_turtle_god(self,future):
        response = future.result()
        self.get_logger().info(str(response.debbug))
        self.Target_set_ = False
        self.idx_ = 0
        self.kill_in_progress = False


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()

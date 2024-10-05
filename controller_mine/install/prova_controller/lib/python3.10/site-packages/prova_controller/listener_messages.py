#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from px4_msgs.msg import SensorCombined


 
class ListenerMessage(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("listener_node") # MODIFY NAME
        self.get_logger().info("Nodo che sputa inizializzato")
        self.publisher_ = self.create_publisher(String, "prova_topic",20)
        self.timer_ = self.create_timer(0.5, self.callback_timer)
    
    def callback_timer(self):
        msg = String()
        msg.data = "messaggio"
        self.publisher_.publish(msg)
        

 
 
def main(args=None):
    rclpy.init(args=args)
    node = ListenerMessage() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

 
class ProvaListener(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("listener_node") # MODIFY NAME
        self.get_logger().info("Nodo che ascolta inizializzato")
        self.listener_ = self.create_subscription(String, "prova_topic",self.callback_ascolto, 20)
    
    def callback_ascolto(self,msg):
         self.get_logger().info(msg.data)
        
        

 
 
def main(args=None):
    rclpy.init(args=args)
    node = ProvaListener() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
from gs_ros import GsRosBridge
import rclpy
from rclpy.node import Node
import genesis as gs


def main(args=None):
    gs.init()
    rclpy.init(args=args)

    ros_node=Node('gs_ros_bridge_node')
    # four ros2 nodes can be provided to the constructor 
    # ros_node(used for publishing sensor data),ros_clock_node,ros_control_node,ros_service_node
    # if only one node is provided, that node is used for everything which may cause bottlenecks
    gs_ros_bridge=GsRosBridge("src/parent_config.yaml",ros_node)

    gs_ros_bridge.build()
    while rclpy.ok():
        gs_ros_bridge.step()
    rclpy.shutdown()

if __name__=='__main__':
    main()
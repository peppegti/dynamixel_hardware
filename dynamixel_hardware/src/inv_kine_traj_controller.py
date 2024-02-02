import rclpy
from rclpy.node import Node
import math

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint




class TrajectoryControllerNode(Node):
    def __init__(self):
        super().__init__('trajectory_controller_node')
        self.publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        
        
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3']  # Replace with your joint names
        
        point = JointTrajectoryPoint()

        # input position
        x = 0.3
        y = 0.0
        z = -0.3

        if z < 0.0 :
            print("z value must be positive!")
            rclpy.shutdown()

        
        # arm parameters
        l1 = 0.42
        l2 = 0.37

        
        joint1_val = math.atan2(y,x)

        dist_xy = math.sqrt(x**2 + y**2) 
        d = math.sqrt(dist_xy**2 + z**2)

        phi_3 = math.atan2(z,dist_xy)
        cos_joint2_val = (l1**2 + d**2 - l2**2) / (2 * l1 * d)
        joint2_val = phi_3 + math.acos(cos_joint2_val)

        cos_joint3_val = (l1**2 + l2**2 - d**2) / (2 * l1 * l2)
        joint3_val = math.acos(cos_joint3_val) - math.pi

    
        # print(f"joint 1 value", joint1_val)
        # print(f"joint 2 value", joint2_val)
        # print(f"joint 3 value", joint3_val)


        point.positions = [joint1_val, joint2_val-math.pi/2, -joint3_val]
        #print(point.positions)
        trajectory_msg.points.append(point)

        # Publish the trajectory
        self.publisher.publish(trajectory_msg)
        self.get_logger().info('Published Joint Trajectory')

  




def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryControllerNode()  
    rclpy.spin(node)


if __name__ == '__main__':
    main()

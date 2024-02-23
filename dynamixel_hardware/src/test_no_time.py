import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryControllerNode(Node):
    def __init__(self):
        super().__init__('trajectory_controller_node')
        self.publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 1)
        
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3'] 
        
        point = JointTrajectoryPoint()
        #point.positions = [0.707, 0.707, 0.707]  # CANDLE POSITION
        point.positions = [0.707, 0.707, 0.707]  # BOOO POSITION
        point.velocities = [0.1, 0.1, 0.1]  # Velocities in radians per second
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

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryControllerNode(Node):
    def __init__(self):
        super().__init__('trajectory_controller_node')
        self.publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        
        # Creating a sample trajectory
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3']  # Replace with your joint names
        
        point = JointTrajectoryPoint()
        #point.positions = [0.707, -0.707, 0.707]
        point.positions = [-0.707, -0.707, 0.707]
        #point.positions = [0.0, -0.707, 0.707]   # Replace with desired joint positions
        #point.positions = [0.0, 0.0, -3.14] # RAISED POSITION
        #point.positions = [0.0, -1.75, -3.14]  # STORED POSITION
        #point.positions = [0.0, 0.0, 0.0]  # HOME POSITION
        point.time_from_start.sec = 1  # Specify the time from the start of the trajectory
        trajectory_msg.points.append(point)

        # Publish the trajectory
        self.publisher.publish(trajectory_msg)
        self.get_logger().info('Published Joint Trajectory')

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryControllerNode()
    #rclpy.spin_once(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

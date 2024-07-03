import rclpy
import rclpy.logging
from rclpy.node import Node
import math

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class Standup(Node):

    """A simple controller that makes the robot stand up."""
    def __init__(self):
        super().__init__('standup')
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.joint_names = ['FL_HFE', 'FL_KFE', 'FR_HFE', 'FR_KFE', 'HL_HFE', 'HL_KFE', 'HR_HFE', 'HR_KFE']

    def standup(self):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        msg.points = [JointTrajectoryPoint()] * 2
        msg.points[1].positions = [0, 0, 0, 0, 0, 0, 0, 0]
        msg.points[1].time_from_start = rclpy.duration.Duration(seconds=3).to_msg()

        msg.points[0].positions = [math.pi/2, -math.pi, math.pi/2, -math.pi, -math.pi/2, math.pi, -math.pi/2, math.pi]
        msg.points[0].time_from_start = rclpy.duration.Duration(seconds=1).to_msg()

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    standup = Standup()

    for i in range(10):
        rclpy.logging.get_logger('standup').info('Standing up...')

        standup.standup()

        # reset time

        # Sleep for 2 seconds
        rclpy.spin_once(standup, timeout_sec=2)

    rclpy.spin(standup)
    standup.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
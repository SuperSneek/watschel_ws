import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.action import ActionClient
import math
import sys
import time

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState

class Movements(Node):

    """A simple controller that makes the robot stand up."""
    def __init__(self):
        super().__init__('movements')
        self.client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        self.joint_names = ['FL_HFE', 'FL_KFE', 'FR_HFE', 'FR_KFE', 'HL_HFE', 'HL_KFE', 'HR_HFE', 'HR_KFE']
        server_reached = self.client.wait_for_server(timeout_sec=60.0)
        if not server_reached:
            self.get_logger().error('Unable to connect to arm action server. Timeout exceeded.')
            sys.exit()

        self.joint_state = JointState()
        self.js_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.js_sub

    def joint_state_callback(self, msg):
        self.joint_state = msg

    def sitdown(self):
        if (self.joint_state is not None):
            initial_point = JointTrajectoryPoint()
            initial_point.positions = self.joint_state.position if (len(self.joint_state.position) > 0) else [math.pi/2, -math.pi, math.pi/2, -math.pi, -math.pi/2, math.pi, -math.pi/2, math.pi]
            initial_point.time_from_start = rclpy.duration.Duration(seconds=1).to_msg()

            final = JointTrajectoryPoint()
            final.positions = [math.pi/2, -math.pi, math.pi/2, -math.pi, -math.pi/2, math.pi, -math.pi/2, math.pi]
            final.time_from_start = rclpy.duration.Duration(seconds=5).to_msg()

            action = FollowJointTrajectory.Goal()
            action.trajectory.joint_names = self.joint_names
            action.trajectory.points = [initial_point, final]

            self.client.send_goal_async(action)
            self.get_logger().info('Sitting down...')

    def standup(self):
        if (self.joint_state is not None):
            initial_point = JointTrajectoryPoint()
            initial_point.positions = self.joint_state.position if (len(self.joint_state.position) > 0) else [0, 0, 0, 0, 0, 0, 0, 0]
            initial_point.time_from_start = rclpy.duration.Duration(seconds=1).to_msg()

            final = JointTrajectoryPoint()
            final.positions = [0, 0, 0, 0, 0, 0, 0, 0]
            final.time_from_start = rclpy.duration.Duration(seconds=5).to_msg()

            action = FollowJointTrajectory.Goal()
            action.trajectory.joint_names = self.joint_names
            action.trajectory.points = [initial_point, final]

            self.client.send_goal_async(action)
            self.get_logger().info('Standing up...')


def main(args=None):
    rclpy.init(args=args)

    move = Movements()

    for i in range(0, 20):
        move.sitdown()
        rclpy.spin_once(move, timeout_sec=5)
        time.sleep(5)
        move.standup()
        rclpy.spin_once(move, timeout_sec=5)
        time.sleep(5)

    move.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
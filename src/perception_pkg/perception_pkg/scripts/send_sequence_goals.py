#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class SequenceNavigator(Node):
    def __init__(self):
        super().__init__('sequence_navigator')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        self.get_logger().info(f'Sending goal: {pose.pose.position}')
        send_goal_future = self._action_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        self.get_logger().info('Goal accepted, waiting for result...')
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result()

        if result.status == 4:  # STATUS_SUCCEEDED
            self.get_logger().info('Goal succeeded!')
            return True
        else:
            self.get_logger().warn(f'Goal failed with status: {result.status}')
            return False


def make_pose(x, y, yaw=0.0):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = rclpy.time.Time().to_msg()

    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.w = 1.0  # No rotation (yaw = 0)

    return pose


def main(args=None):
    rclpy.init(args=args)
    navigator = SequenceNavigator()

    goals = [
        make_pose(1.0, 0.0),
        make_pose(1.0, 1.0),
        make_pose(0.0, 1.0)
    ]

    for i, goal in enumerate(goals, 1):
        navigator.get_logger().info(f'--- Goal {i} ---')
        success = navigator.send_goal(goal)
        if not success:
            navigator.get_logger().error(f'Aborting sequence at goal {i}')
            break

    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Automated pickup test for Orion robot.

Drives to the pickup_object using its published pose, positions the arm,
closes the gripper, lifts, then verifies the object moved.

Usage:
    python3 pickup_test.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

import math
import time

from geometry_msgs.msg import TwistStamped, PoseArray
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration as TrajDuration


#CONSTANTS
DRIVE_SPEED       = 0.3      # m/s forward approach speed
STOP_DISTANCE     = 0.20     # m  distance from object to stop (gripper reach)
APPROACH_TIMEOUT  = 15.0     # s  max time to drive toward object
PICKUP_POSITIONS  = [0.4, 0.15, 1.8, 0.0]   # shoulder, elbow, wrist, wrist_roll
HOME_POSITIONS    = [0.0, 0.0, 0.0, 0.0]
GRIPPER_OPEN      = 0.025    # m  fully open
GRIPPER_CLOSE     = 0.003    # m  gripping (small gap, not fully closed)
ARM_JOINTS        = ['shoulder_joint', 'elbow_joint', 'wrist_joint', 'wrist_roll_joint']
GRIPPER_JOINTS    = ['gripper_left_joint']

class PickupTest(Node):

    def __init__(self):
        super().__init__('pickup_test')

        # Pubs
        self.cmd_pub = self.create_publisher(
            TwistStamped, '/mecanum_controller/reference', 10)

        # Subs
        self.object_pose = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.object_initial_pose = None
        self.object_final_pose = None

        self.pose_sub = self.create_subscription(
            PoseArray, '/model/pickup_object/pose',
            self._pose_cb, 10)

        # Action clients
        self.arm_client = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(
            self, FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory')

    def _pose_cb(self, msg: PoseArray):
        if msg.poses:
            self.object_pose = msg.poses[0]

    #Helpers

    def _stop(self):
        msg = TwistStamped()
        msg.header.frame_id = 'base_link'
        self.cmd_pub.publish(msg)

    def _drive(self, vx=0.0, vy=0.0, wz=0.0):
        msg = TwistStamped()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.angular.z = wz
        self.cmd_pub.publish(msg)

    def _wait_for_object_pose(self, timeout=5.0):
        self.get_logger().info('Waiting for object pose...')
        t0 = time.time()
        while self.object_pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - t0 > timeout:
                self.get_logger().error('Timed out waiting for object pose')
                return False
        self.get_logger().info(
            f'Object at x={self.object_pose.position.x:.3f} '
            f'y={self.object_pose.position.y:.3f} '
            f'z={self.object_pose.position.z:.3f}')
        return True

    def _move_arm(self, positions, duration_sec=3):
        self.get_logger().info(f'Moving arm to {positions}')
        self.arm_client.wait_for_server()

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start = TrajDuration(sec=duration_sec)
        goal.trajectory.points = [pt]

        future = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result.error_code == 0

    def _move_gripper(self, position, duration_sec=2):
        self.get_logger().info(f'Moving gripper to {position:.4f}m')
        self.gripper_client.wait_for_server()

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = GRIPPER_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [position]
        pt.time_from_start = TrajDuration(sec=duration_sec)
        goal.trajectory.points = [pt]

        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return True

    def _drive_to_object(self):
        """Drive forward until within STOP_DISTANCE of object."""
        self.get_logger().info(
            f'Driving toward object (stopping at {STOP_DISTANCE}m)...')
        t0 = time.time()

        while True:
            rclpy.spin_once(self, timeout_sec=0.05)

            if self.object_pose is None:
                self._stop()
                self.get_logger().error('Lost object pose during approach')
                return False

            dist = self.object_pose.position.x
            lateral = self.object_pose.position.y

            if dist <= STOP_DISTANCE:
                self._stop()
                self.get_logger().info(
                    f'Reached object at distance {dist:.3f}m')
                return True

            if time.time() - t0 > APPROACH_TIMEOUT:
                self._stop()
                self.get_logger().error('Approach timed out')
                return False

            # Proportional lateral correction to keep object centred
            lateral_correction = -lateral * 0.5
            self._drive(vx=DRIVE_SPEED, vy=lateral_correction)
            time.sleep(0.05)
    def run(self):
        self.get_logger().info('=== Orion Pickup Test ===')

        if not self._wait_for_object_pose():
            return
        self.object_initial_pose = (
            self.object_pose.position.x,
            self.object_pose.position.y,
            self.object_pose.position.z,
        )

        self.get_logger().info('Step 1: Opening gripper')
        self._move_gripper(GRIPPER_OPEN)


        self.get_logger().info('Step 2: Homing arm')
        self._move_arm(HOME_POSITIONS, duration_sec=2)


        self.get_logger().info('Step 3: Driving to object')
        if not self._drive_to_object():
            self._stop()
            return

        self.get_logger().info('Step 4: Positioning arm')
        self._move_arm(PICKUP_POSITIONS, duration_sec=4)


        self.get_logger().info('Step 5: Closing gripper')
        self._move_gripper(GRIPPER_CLOSE)
        time.sleep(0.5)

        self.get_logger().info('Step 6: Lifting arm')
        lift_positions = [0.8, PICKUP_POSITIONS[1], PICKUP_POSITIONS[2], 0.0]
        self._move_arm(lift_positions, duration_sec=3)

        time.sleep(1.0)
        rclpy.spin_once(self, timeout_sec=0.5)
        if self.object_pose:
            final = (
                self.object_pose.position.x,
                self.object_pose.position.y,
                self.object_pose.position.z,
            )
            dz = final[2] - self.object_initial_pose[2]
            self.get_logger().info(
                f'Object initial z={self.object_initial_pose[2]:.3f}m  '
                f'final z={final[2]:.3f}m  dz={dz:.3f}m')

            if dz > 0.01:
                self.get_logger().info('PICKUP SUCCESS — object lifted')
            else:
                self.get_logger().warn(
                    'Object z unchanged — gripper may have missed. '
                    'Try adjusting PICKUP_POSITIONS or STOP_DISTANCE.')
        else:
            self.get_logger().warn('Could not read final object pose')

        self.get_logger().info('=== Pickup test complete ===')


def main():
    rclpy.init()
    node = PickupTest()
    try:
        node.run()
    except KeyboardInterrupt:
        node._stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

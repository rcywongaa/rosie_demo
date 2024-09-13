#!/usr/bin/env python

from time import sleep
import rclpy
from rclpy.node import Node
import copy
import geometry_msgs.msg
import tf2_ros
import tf2_geometry_msgs
from tf_transformations import euler_from_quaternion
import csv
import numpy as np
from threading import RLock
import hebi

ROBOT_NAME="rosie1"
ARM_GROUP_NAME="Arm1"
SHOULD_SEND_COMMANDS = True
# SHOULD_SEND_COMMANDS = False
# GO_TO_STARTING_POSITION = False
GO_TO_STARTING_POSITION = True
IS_LOG = True

# control frequency in Hz
loop_frequency = 50

max_base_velocity = 1
max_joint_velocity = 1

pose_timeout_ns = 0.5*1e9
#Tolerance interval for declaring a position as reached (x, y, heading angle)
reach_tolerance = (0.05, 0.05, 0.1)
base_gains = (0.5, 0.5, 0.5)

module_names = [ "J1_base", "J2_shoulder", "J3_elbow", "J4_wrist1", "J5_wrist2", "J6_wrist3" ] # Refer to A-2085-06_params.yaml
home_position = [0, 2.09, 2.09, 0, 1.65, 0]

#=====================================
# Apply transform to a twist message
#     including angular velocity
#=====================================
def transform_twist(twist = geometry_msgs.msg.Twist, transform_stamped = geometry_msgs.msg.TransformStamped):

    transform_stamped_ = copy.deepcopy(transform_stamped)
    #Inverse real-part of quaternion to inverse rotation
    transform_stamped_.transform.rotation.w = - transform_stamped_.transform.rotation.w

    twist_vel = geometry_msgs.msg.Vector3Stamped()
    twist_rot = geometry_msgs.msg.Vector3Stamped()
    twist_vel.vector = twist.linear
    twist_rot.vector = twist.angular
    out_vel = tf2_geometry_msgs.do_transform_vector3(twist_vel, transform_stamped_)
    out_rot = tf2_geometry_msgs.do_transform_vector3(twist_rot, transform_stamped_)

    #Populate new twist message
    new_twist = geometry_msgs.msg.Twist()
    new_twist.linear = out_vel.vector
    new_twist.angular = out_rot.vector

    return new_twist

class PositionController(Node):
    def __init__(self):
        super().__init__("position_controller")
        self.logger = self.get_logger()
        self.create_subscription(geometry_msgs.msg.PoseStamped, "/qualisys/"+ROBOT_NAME+"/pose", self.pose_callback, 1)
        self.group = None
        while self.group is None:
            self.logger.info(f"Waiting to get hebi group: {ARM_GROUP_NAME}")
            self.group = hebi.Lookup().get_group_from_names([ARM_GROUP_NAME], module_names)
            sleep(0.5)
        self.group_feedback = hebi.GroupFeedback(self.group.size)
        self.cmd = hebi.GroupCommand(self.group.size)
        self.base_vel_pubs = self.create_publisher(geometry_msgs.msg.Twist, "/" + ROBOT_NAME + "/cmd_vel", 1)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.last_pose = None
        # self.last_pose = geometry_msgs.msg.PoseStamped() # FIXME: Remove
        # self.last_received_pose_time = self.get_clock().now() # FIXME: Remove
        self.mtx = RLock()

    def pose_callback(self, pose_stamped_msg):
        with self.mtx:
            self.last_pose = pose_stamped_msg
            self.last_received_pose_time = self.get_clock().now()

    def set_goal(self, base_position_goal, arm_joint_position_goal):
        with self.mtx:
            self.base_position_goal = base_position_goal
            self.arm_joint_position_goal = arm_joint_position_goal
            self.logger.info(f"Current target: {base_position_goal}")

    def get_error(self):
        with self.mtx:
            error_x = self.base_position_goal[0] - self.last_pose.pose.position.x
            error_y = self.base_position_goal[1] - self.last_pose.pose.position.y
            (roll, pitch, yaw) = euler_from_quaternion([self.last_pose.pose.orientation.x,
                                                        self.last_pose.pose.orientation.y,
                                                        self.last_pose.pose.orientation.z,
                                                        self.last_pose.pose.orientation.w])
            error_heading = self.base_position_goal[2] - yaw
            return error_x, error_y, error_heading

    def loop_once(self):
        with self.mtx:
            now  = self.get_clock().now()
            base_vel_cmd_msg = geometry_msgs.msg.Twist()


            if (now.nanoseconds < self.last_received_pose_time.nanoseconds + pose_timeout_ns):
                error_x, error_y, error_heading = self.get_error()
                base_vel_cmd_msg.linear.x = np.clip(error_x * base_gains[0], -max_base_velocity, max_base_velocity)
                base_vel_cmd_msg.linear.y = np.clip(error_y * base_gains[1], -max_base_velocity, max_base_velocity)
                base_vel_cmd_msg.angular.z = np.clip(error_heading * base_gains[2], -max_base_velocity, max_base_velocity)
            else:
                base_vel_cmd_msg.linear.x = 0
                base_vel_cmd_msg.linear.y = 0
                base_vel_cmd_msg.angular.z = 0

            transform = self.tf_buffer.lookup_transform('mocap', ROBOT_NAME, rclpy.time.Time())
            base_vel_cmd_transformed = transform_twist(base_vel_cmd_msg, transform)
            self.base_vel_pubs.publish(base_vel_cmd_transformed)

            # arm_pan = self.arm_joint_position_goal[0]
            # arm_lift = self.arm_joint_position_goal[1]
            # arm_flex = self.arm_joint_position_goal[2]
            # arm_w1 = self.arm_joint_position_goal[3]
            # arm_w2 = self.arm_joint_position_goal[4]
            # arm_w3 = self.arm_joint_position_goal[5]
            self.cmd.position = self.arm_joint_position_goal
            self.group.send_command(self.cmd)

# class PathPlayer(Node):
def main(args=None):
    rclpy.init(args=args)
    logger = rclpy.logging.get_logger("position_controller")
    controller = PositionController()

    is_going_to_starting_point = True

    controller.declare_parameter("positions_file", rclpy.Parameter.Type.STRING)
    with open(controller.get_parameter("positions_file").value, "r") as positions_file:
        timed_positions = list(csv.reader(positions_file, quoting=csv.QUOTE_NONNUMERIC))

    line_idx = 0
    loop_rate = controller.create_rate(loop_frequency)
    while rclpy.ok() and line_idx < len(timed_positions):
        if is_going_to_starting_point:
            logger.info("Going to starting position...")
            base_position_target = timed_positions[0][0:3]
            arm_joint_target = home_position
            controller.set_goal(base_position_target, arm_joint_target)
            error_x, error_y, error_heading = controller.get_error()
            if ((abs(error_x) < reach_tolerance[0]) and (abs(error_y) < reach_tolerance[1]) and (abs(error_heading) < reach_tolerance[2])):
                logger.info("Reached initial goal: " + str(base_position_target) + "!")
                is_going_to_starting_point = False
        else:
            logger.info("Line: %d", line_idx)
            error_x, error_y, error_heading = controller.get_error()
            if ((abs(error_x) < reach_tolerance[0]) and (abs(error_y) < reach_tolerance[1]) and (abs(error_heading) < reach_tolerance[2])):
                line_idx += 1
        controller.loop_once()
        rclpy.spin_once(controller)
        loop_rate.sleep()

    rclpy.shutdown()

if __name__ == "__main__":
    main()

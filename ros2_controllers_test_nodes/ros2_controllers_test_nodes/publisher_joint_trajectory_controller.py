# Copyright 2022 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Denis Štogl, Lovro Ivanov
#

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState


class PublisherJointTrajectory(Node):
    def __init__(self):
        super().__init__("publisher_position_trajectory_controller")
        # Declare all parameters
        self.declare_parameter("controller_name_arm", "joint_trajectory_controller")
        self.declare_parameter("controller_name_gripper", "rg2_gripper_trajectory_controller")
        self.declare_parameter("wait_sec_between_publish", 0.1)
        self.declare_parameter("goal_names_arm", ["start_pos_arm"])
        self.declare_parameter("goal_names_gripper", ["start_pos_gripper"])
        self.declare_parameter("joints_arm", ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"])
        self.declare_parameter("joints_gripper", ["rg2_finger_joint1", "rg2_finger_joint2"])
        self.declare_parameter("check_starting_point", False)
        self.declare_parameter("starting_point_limits")

        # Read parameters
        controller_name_arm = self.get_parameter("controller_name_arm").value
        controller_name_gripper = self.get_parameter("controller_name_gripper").value
        wait_sec_between_publish = self.get_parameter("wait_sec_between_publish").value
        goal_names_arm = ["start_pos_arm"] #self.get_parameter("goal_names").value
        goal_names_gripper = ["start_pos_gripper"] #self.get_parameter("goal_names").value
        self.joints_arm = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"] #self.get_parameter("joints").value
        self.joints_gripper =  ["rg2_finger_joint1", "rg2_finger_joint2"]
        self.check_starting_point = self.get_parameter("check_starting_point").value
        self.starting_point = {}

        # if self.joints is None or len(self.joints) == 0:
        #     raise Exception('"joints" parameter is not set!')

        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )
        # initialize starting point status
        self.joint_state_sub
        self.starting_point_ok = not self.check_starting_point

        self.joint_state_msg_received = False

        # Read all positions from parameters
        self.goals_arm = []
        for name in goal_names_arm:
            self.declare_parameter(name)
            goal_arm = [1.59, -1.47, 1.67, -1.78, -1.58, 0.0]#self.get_parameter(name).value
            if goal_arm is None or len(goal_arm) == 0:
                raise Exception(f'Values for goal "{name}" not set!')

            float_goal_arm = []
            for value in goal_arm:
                float_goal_arm.append(float(value))
            self.goals_arm.append(float_goal_arm)

        publish_topic_arm = "/" + controller_name_arm + "/" + "joint_trajectory"

        self.get_logger().info(
            'Publishing {} goals on topic "{}" every {} s'.format(
                len(goal_names_arm), publish_topic_arm, wait_sec_between_publish
            )
        )

        self.goals_gripper = []
        for name in goal_names_gripper:
            self.declare_parameter(name)
            goal_gripper = [0.0, 0.0]#self.get_parameter(name).value
            if goal_gripper is None or len(goal_gripper) == 0:
                raise Exception(f'Values for goal "{name}" not set!')

            float_goal_gripper = []
            for value in goal_gripper:
                float_goal_gripper.append(float(value))
            self.goals_gripper.append(float_goal_gripper)

        publish_topic_gripper = "/" + controller_name_gripper + "/" + "joint_trajectory"
        self.get_logger().info(
            'Publishing {} goals on topic "{}" every {} s'.format(
                len(goal_names_gripper), publish_topic_gripper, wait_sec_between_publish
            )
        )

        self.publisher_arm = self.create_publisher(JointTrajectory, publish_topic_arm, 1)
        self.publisher_gripper = self.create_publisher(JointTrajectory, publish_topic_gripper, 1)
        self.timer = self.create_timer(wait_sec_between_publish, self.timer_callback)
        self.i = 0
        self.goal_published = False

        self.goal_published_arm = False
        self.goal_published_gripper = False

    def timer_callback(self):
        if not self.joint_state_msg_received:
            self.get_logger().warn('No joint state received')
            return
        
        self.goal_published = self.goal_published_arm and self.goal_published_gripper

        if self.goal_published:
            self.get_logger().warn('Goal published. Destroy node')
            self.destroy_node()
            raise Exception('Throwing exception to destroy node completely')

        if not self.goal_published_arm:
            self.get_logger().warn('Publishing arm goal...')
            traj_arm = JointTrajectory()
            traj_arm.joint_names = self.joints_arm
            point = JointTrajectoryPoint()
            point.positions = self.goals_arm[self.i]
            point.time_from_start = Duration(sec=2)

            traj_arm.points.append(point)
            self.publisher_arm.publish(traj_arm)
            self.goal_published_arm = True

        if not self.goal_published_gripper:
            self.get_logger().warn('Publishing gripper goal...')
            traj_gripper = JointTrajectory()
            traj_gripper.joint_names = self.joints_gripper
            point = JointTrajectoryPoint()
            point.positions = self.goals_gripper[self.i]
            point.time_from_start = Duration(sec=2)

            traj_gripper.points.append(point)
            self.publisher_gripper.publish(traj_gripper)
            self.goal_published_gripper = True
            

        elif self.check_starting_point and not self.joint_state_msg_received:
            self.get_logger().warn(
                'Start configuration could not be checked! Check "joint_state" topic!'
            )
        else:
            self.get_logger().warn("Start configuration is not within configured limits!")

    def joint_state_callback(self, msg):
        if not self.joint_state_msg_received:

            # check start state
            if self.check_starting_point:
                limit_exceeded = [False] * len(msg.name)
                for idx, enum in enumerate(msg.name):
                    if (msg.position[idx] < self.starting_point[enum][0]) or (
                        msg.position[idx] > self.starting_point[enum][1]
                    ):
                        self.get_logger().warn(f"Starting point limits exceeded for joint {enum} !")
                        limit_exceeded[idx] = True

                if any(limit_exceeded):
                    self.starting_point_ok = False
                else:
                    self.starting_point_ok = True

            self.joint_state_msg_received = True
            self.get_logger().warn('joint state received...')



def main(args=None):
    rclpy.init(args=args)
    try:
        publisher_joint_trajectory = PublisherJointTrajectory()
        rclpy.spin(publisher_joint_trajectory)
    except:
        print("Entered except")
        publisher_joint_trajectory.destroy_node()
    finally:
        print("Entered finally")
        rclpy.shutdown()


if __name__ == "__main__":
    main()

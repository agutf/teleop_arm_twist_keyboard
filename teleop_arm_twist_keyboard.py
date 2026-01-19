# Copyright 2011 Brown University Robotics.
# Copyright 2017 Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import sys
import threading

import geometry_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import rcl_interfaces.msg
import rclpy
from rclpy.node import Node
import termios
import tty

class TeleopArm(Node):
    def __init__(self):
        super().__init__('TeleopArmTwistKeyboard')

        self.settings = self.saveTerminalSettings()

        # parameters
        read_only_descriptor = rcl_interfaces.msg.ParameterDescriptor(read_only=True)
        self.stamped = self.declare_parameter('stamped', False, read_only_descriptor).value
        self.frame_id = self.declare_parameter('frame_id', '', read_only_descriptor).value
        self.speed = self.declare_parameter('speed', 0.5, read_only_descriptor).value
        self.turn = self.declare_parameter('turn', 1.0, read_only_descriptor).value

        if not self.stamped and self.frame_id:
            raise Exception("'frame_id' can only be set when 'stamped' is True")

        self.msg = """
            This node takes keypresses from the keyboard and publishes them
            as Twist/TwistStamped messages.
            ---------------------------
            Moving around:
                w
            a       d
                s    

            z : up (+z)
            x : down (-z)

            Orientation (same but with Shift):
            ---------------------------
                W    
            A       D
                S    

            Z : up (+z)
            X : down (-z)

            Gripper::
            ---------------------------

            o : open gripper
            c : close gripper

            anything else : stop

            +/- : increase/decrease speed by 10%

            CTRL-C to quit
            """

        self.moveBindings = {
            'w': (1, 0, 0, 0, 0, 0),
            's': (-1, 0, 0, 0, 0, 0),
            'a': (0, 1, 0, 0, 0, 0),
            'd': (0, -1, 0, 0, 0, 0),
            'z': (0, 0, 1, 0, 0, 0),
            'x': (0, 0, -1, 0, 0, 0),
        }

        self.rotationBindings = {
            'W': ("arm_7_joint", 0.1),
            'S': ("arm_7_joint", -0.1),
            'A': ("arm_6_joint", 0.1),
            'D': ("arm_6_joint", -0.1),
            'Z': ("arm_5_joint", 0.1),
            'X': ("arm_5_joint", -0.1),
        }

        self.speedBindings = {
            '+': (1.1, 1.1),
            '-': (.9, .9),
        }

        self.gripperBindings = {
            'o': 0.005,  # open
            'c': -0.005, # close
        }

        self.arm_joints = ["arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"]
        self.gripper_joints = ['gripper_right_finger_joint', 'gripper_left_finger_joint']
        self.current_positions = None

        if self.stamped:
            TwistMsg = geometry_msgs.msg.TwistStamped
        else:
            TwistMsg = geometry_msgs.msg.Twist

        self.twist_msg = TwistMsg()

        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_cb,
            10
        )

        self.arm_pub = self.create_publisher(TwistMsg, 'cmd_vel', 10)

        self.rotation_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        self.gripper_pub = self.create_publisher(
            JointTrajectory,
            '/gripper_controller/joint_trajectory',
            10
        )

        self.keyboard_thread = threading.Thread(
            target=self.keyboard_loop,
            daemon=True
        )
        self.keyboard_thread.start()

    def keyboard_loop(self):
        print(self.msg)
        while rclpy.ok():
            key = self.getKey()
            self.process_key(key)

    def process_key(self, key):
        if key in self.moveBindings.keys():
            x = self.moveBindings[key][0]
            y = self.moveBindings[key][1]
            z = self.moveBindings[key][2]
            self.move_arm(x, y, z)
        elif key in self.rotationBindings.keys():
            # arm_link_7 = pos 11, arm_link_6 = pos 9 y arm_link_5 = pos 12
            joint_name, delta = self.rotationBindings[key]
            self.rotate_arm(joint_name, delta)
        elif key in self.gripperBindings.keys():
            delta = self.gripperBindings[key]
            self.move_gripper(delta)
        elif key in self.speedBindings.keys():
            self.speed = self.speed * self.speedBindings[key][0]
            self.turn = self.turn * self.speedBindings[key][1]
        elif key == '\x03':  # CTRL-C
            rclpy.shutdown()
        else:
            x = 0.0
            y = 0.0
            z = 0.0
            self.move_arm(x, y, z)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

        return key
    
    def saveTerminalSettings(self):
        return termios.tcgetattr(sys.stdin)

    def joint_states_cb(self, msg):
        self.current_positions = {'names': msg.name, 'positions': msg.position}

    def move_arm(self, x, y, z):
        if self.stamped:
            self.twist_msg.header.stamp = self.get_clock().now().to_msg()

        self.twist_msg.twist.linear.x = x * self.speed
        self.twist_msg.twist.linear.y = y * self.speed
        self.twist_msg.twist.linear.z = z * self.speed
        self.twist_msg.twist.angular.x = 0.0
        self.twist_msg.twist.angular.y = 0.0
        self.twist_msg.twist.angular.z = 0.0
        self.arm_pub.publish(self.twist_msg)

    def rotate_arm(self, joint_name, delta):
        if self.current_positions is None:
            return
        
        joint_idx = self.arm_joints.index(joint_name)
        
        joint_positions = [self.current_positions['names'].index(joint) for joint in self.arm_joints]
        positions = [self.current_positions['positions'][i] for i in joint_positions]

        traj = JointTrajectory()
        traj.joint_names = self.arm_joints

        point = JointTrajectoryPoint()
        positions[joint_idx] = positions[joint_idx] + (delta * self.speed)
        point.positions = positions

        traj.points.append(point)
        self.rotation_pub.publish(traj)

    def move_gripper(self, delta):
        if self.current_positions is None:
            return
        
        right_gripper_idx = self.current_positions['names'].index(self.gripper_joints[0])
        left_gripper_idx = self.current_positions['names'].index(self.gripper_joints[1])

        traj = JointTrajectory()
        traj.joint_names = self.gripper_joints

        point = JointTrajectoryPoint()
        point.positions = [self.current_positions['positions'][right_gripper_idx] + (delta * self.speed), self.current_positions['positions'][left_gripper_idx] + (delta * self.speed)]

        traj.points.append(point)
        self.gripper_pub.publish(traj)

def restoreTerminalSettings(settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def main():

    rclpy.init()

    node = TeleopArm()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        restoreTerminalSettings(node.settings)


if __name__ == '__main__':
    main()

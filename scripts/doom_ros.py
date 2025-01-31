#!/usr/bin/env python3
import threading

import cydoomgeneric as cdg
import rclpy
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from sensor_msgs.msg import Image, Joy


class DoomROS(Node):
    def __init__(self):
        super().__init__("doom_ros")
        self.declare_parameter(
            "width",
            320,
            ParameterDescriptor(description="Width of the rendered frames"),
        )
        self.declare_parameter(
            "height",
            200,
            ParameterDescriptor(description="Height of the rendered frames"),
        )
        self.declare_parameter(
            "doom_image",
            "doom_image",
            ParameterDescriptor(description="The topic of the joy message"),
        )
        self.declare_parameter(
            "joy_topic",
            "joy",
            ParameterDescriptor(description="The topic of the joy message"),
        )
        self.declare_parameter(
            "left_key_buttons_index",
            13,
            ParameterDescriptor(
                description="The index of the buttons array of the joy message used as the LEFT ARROW button"
            ),
        )
        self.declare_parameter(
            "right_key_buttons_index",
            14,
            ParameterDescriptor(
                description="The index of the buttons array of the joy message used as the RIGHT ARROW button"
            ),
        )
        self.declare_parameter(
            "up_key_buttons_index",
            11,
            ParameterDescriptor(
                description="The index of the buttons array of the joy message used as the UP ARROW button"
            ),
        )
        self.declare_parameter(
            "down_key_buttons_index",
            12,
            ParameterDescriptor(
                description="The index of the buttons array of the joy message used as the DOWN ARROW button"
            ),
        )
        self.declare_parameter(
            "strafe_left_key_buttons_index",
            9,
            ParameterDescriptor(
                description="The index of the buttons array of the joy message used as the STRAFE LEFT button"
            ),
        )
        self.declare_parameter(
            "strafe_right_key_buttons_index",
            10,
            ParameterDescriptor(
                description="The index of the buttons array of the joy message used as the STRAFE RIGHT button"
            ),
        )
        self.declare_parameter(
            "fire_key_buttons_index",
            1,
            ParameterDescriptor(
                description="The index of the buttons array of the joy message used as the FIRE button"
            ),
        )
        self.declare_parameter(
            "use_key_buttons_index",
            0,
            ParameterDescriptor(
                description="The index of the buttons array of the joy message used as the USE button"
            ),
        )
        self.declare_parameter(
            "run_key_buttons_index",
            3,
            ParameterDescriptor(
                description="The index of the buttons array of the joy message used as the RUN button (RSHIFT)"
            ),
        )
        self.declare_parameter(
            "enter_key_buttons_index",
            6,
            ParameterDescriptor(
                description="The index of the buttons array of the joy message used as the ENTER button"
            ),
        )
        self.declare_parameter(
            "escape_key_buttons_index",
            4,
            ParameterDescriptor(
                description="The index of the buttons array of the joy message used as the ESCAPE button"
            ),
        )
        self.width = self.get_parameter("width").get_parameter_value().integer_value
        self.height = self.get_parameter("height").get_parameter_value().integer_value
        self.doom_image = (
            self.get_parameter("doom_image").get_parameter_value().string_value
        )
        self.joy_topic = (
            self.get_parameter("joy_topic").get_parameter_value().string_value
        )
        self.key_mappings = {
            cdg.Keys.LEFTARROW: self.get_parameter("left_key_buttons_index")
            .get_parameter_value()
            .integer_value,
            cdg.Keys.RIGHTARROW: self.get_parameter("right_key_buttons_index")
            .get_parameter_value()
            .integer_value,
            cdg.Keys.UPARROW: self.get_parameter("up_key_buttons_index")
            .get_parameter_value()
            .integer_value,
            cdg.Keys.DOWNARROW: self.get_parameter("down_key_buttons_index")
            .get_parameter_value()
            .integer_value,
            cdg.Keys.STRAFE_L: self.get_parameter("strafe_left_key_buttons_index")
            .get_parameter_value()
            .integer_value,
            cdg.Keys.STRAFE_R: self.get_parameter("strafe_right_key_buttons_index")
            .get_parameter_value()
            .integer_value,
            cdg.Keys.FIRE: self.get_parameter("fire_key_buttons_index")
            .get_parameter_value()
            .integer_value,
            cdg.Keys.USE: self.get_parameter("use_key_buttons_index")
            .get_parameter_value()
            .integer_value,
            cdg.Keys.RSHIFT: self.get_parameter("run_key_buttons_index")
            .get_parameter_value()
            .integer_value,
            cdg.Keys.ENTER: self.get_parameter("enter_key_buttons_index")
            .get_parameter_value()
            .integer_value,
            cdg.Keys.ESCAPE: self.get_parameter("escape_key_buttons_index")
            .get_parameter_value()
            .integer_value,
        }
        self.btn_counter = 0
        self.next_frame = Image()
        self.next_frame.width = self.width
        self.next_frame.height = self.height
        self.next_frame.encoding = "bgra8"
        self.next_frame.is_bigendian = 0
        self.next_frame.step = self.width * 4
        self.subscription = self.create_subscription(
            Joy, self.joy_topic, self.joy_callback, 60
        )
        self.last_btn_states = {}
        # for i in self.key_mappings.values():
        #     self.last_btn_states[i] = 0
        for _, index in self.key_mappings.items():
            self.last_btn_states[index] = 0
        self.joy_commands = []
        self.last_joy_command = Joy()
        self.joy_mutex = threading.Lock()
        self.publisher_ = self.create_publisher(Image, self.doom_image, 1)

    def joy_callback(self, msg):
        with self.joy_mutex:
            for btn, index in self.key_mappings.items():
                curr_btn_state = msg.buttons[index]
                last_btn_state = self.last_btn_states[index]
                if last_btn_state != curr_btn_state:
                    self.joy_commands.append((curr_btn_state, btn))
                    self.last_btn_states[index] = curr_btn_state

    def draw_frame(self, pixels):
        self.next_frame.header.stamp = self.get_clock().now().to_msg()
        self.next_frame.data = bytes(pixels)
        self.publisher_.publish(self.next_frame)
        # return None

    def get_key(self):
        with self.joy_mutex:
            if self.joy_commands:
                return self.joy_commands.pop()
        return None


def node_spinner(node):
    rclpy.spin(node)


def main(args=None):
    rclpy.init(args=args)

    doom_ros_node = DoomROS()
    spin_thread = threading.Thread(target=node_spinner, args=(doom_ros_node,))
    spin_thread.start()

    package_name = "doom_ros"
    package_path = get_package_share_directory(package_name)
    cdg.init(
        doom_ros_node.width,
        doom_ros_node.height,
        doom_ros_node.draw_frame,
        doom_ros_node.get_key,
    )
    # TODO: switch to the shareware version
    cdg.main(argv=["cydoomgeneric", "-iwad", package_path + "/data/DOOM1.WAD"])

    doom_ros_node.destroy_node()
    rclpy.shutdown()
    spin_thread.join()


if __name__ == "__main__":
    main()

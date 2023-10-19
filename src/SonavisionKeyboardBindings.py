#!/usr/bin/env python

import rospy
import dynamic_reconfigure.client
import sys, select, termios, tty


class SonavisionKeyboardBindings:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node("sonavision_keyboard_bindings")
        rospy.logwarn_once(
            "Client listens to keypresses and sends them to the server.\
            \r\n\t[UP]: Increase darkness,\r\n\t[DOWN]: Decrease darkness,\
            \r\n\t[LEFT]: Increase blur,\r\n\t[RIGHT]: Decrease blur"
        )
        self.client = dynamic_reconfigure.client.Client(
            "ros1_sonavision_inference", timeout=30, config_callback=self.reconfigure
        )
        self.config = self.client.get_configuration()

        self.delta = 0.1

        self.darkness_level = self.config["darkness_level"]
        self.blur_level = self.config["blur_level"]

        # Initialize variables to control the loop
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)

    def reconfigure(self, config):
        rospy.loginfo("Config set to {blur_level}, {darkness_level}".format(**config))

    def run(self):
        try:
            tty.setraw(sys.stdin.fileno())
            while not rospy.is_shutdown():
                if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
                    key = sys.stdin.read(1)
                    if key == "\x1b":  # Check for an escape sequence
                        next_key = sys.stdin.read(1)
                        if next_key == "[":
                            key = sys.stdin.read(1)
                            # increase darkness by 0.1
                            if key == "A":
                                self.darkness_level = (
                                    round((self.darkness_level + self.delta) * 100)
                                    / 100
                                    if self.darkness_level < 0.95
                                    else 0.95
                                )
                            # decrease darkness by self.delta
                            elif key == "B":
                                self.darkness_level = (
                                    round((self.darkness_level - self.delta) * 100)
                                    / 100
                                    if self.darkness_level > self.delta
                                    else 0
                                )

                            # increase blue by self.delta
                            elif key == "C":
                                self.blur_level = (
                                    round((self.blur_level + self.delta) * 100) / 100
                                    if self.blur_level < 0.95
                                    else 0.95
                                )
                            # decrease blur by self.delta
                            elif key == "D":
                                self.blur_level = (
                                    round((self.blur_level - self.delta) * 100) / 100
                                    if self.blur_level > self.delta
                                    else 0
                                )
                            self.client.update_configuration(
                                {
                                    "blur_level": self.blur_level,
                                    "darkness_level": self.darkness_level,
                                }
                            )
                # self.rate.sleep()
        finally:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)


if __name__ == "__main__":
    try:
        node = SonavisionKeyboardBindings()
        node.run()
    except rospy.ROSInterruptException:
        rospy.logerr("Could not start SonavisionKeyboardBindings.")

import numpy
from datetime import datetime
import os
import cv2
import pyautogui
import sys

from kivy.uix.button import Button
from pykinect_azure.k4abt._k4abtTypes import K4ABT_JOINT_NAMES
from pykinect_azure.k4abt import _k4abt

import pykinect_azure as pykinect
from pykinect_azure.k4a import _k4a
from ODrive_Ease_Lib import *
from time import sleep

# uiStuff
import datetime
import random
from pyautogui import *
import time
from kivy.properties import ObjectProperty
from kivy.app import App
from kivy.core.window import Window, Animation
from kivy.lang import Builder
from kivy.uix.screenmanager import ScreenManager, Screen
from kivy.graphics.texture import Texture
from threading import Thread
from kivy.clock import Clock
from time import sleep
# from Email import Email
# from Firmata import Firmata
from pyfirmata import Arduino, util

import numpy as np
import cv2
from kivy.graphics.texture import Texture
from kivy.clock import Clock
from kivy.uix.image import Image
from pidev.kivy import DPEAButton
from pidev.kivy import ImageButton
# from datetime import datetime
from pidev.kivy.selfupdatinglabel import SelfUpdatingLabel
from pidev.MixPanel import MixPanel


class OdriveMotor:
    def __init__(self, odrive_serial_number, current_limit, velocity_limit):
        self.serial_number = odrive_serial_number
        self.current_limit = current_limit
        self.velocity_limit = velocity_limit
        self.odrive_board = odrive.find_any(serial_number=self.serial_number)
        self.ax = ODrive_Axis(self.odrive_board.axis0, self.current_limit, self.velocity_limit)
        self.homing_sensor = -7
        self.ball_enter_sensor = -9
        self.ball_exit_sensor = -8
        self.homing_sensor_tripped = False
        self.ball_enter_sensor_tripped = False
        self.ball_exit_sensor_tripped = False

    def kinect_motor_calibrate(self):
        if not self.ax.is_calibrated():
            print("calibrating wheel ... ")
            self.ax.calibrate()
            self.ax.gainz(20, 0.16, 0.32, False)
            self.ax.idle()
            dump_errors(self.odrive_board)

    def check_sensors(self):
        states = bin(self.odrive_board.get_gpio_states())
        if int(states[self.homing_sensor]) == 0:
            self.homing_sensor_tripped = True
        if int(states[self.ball_exit_sensor]) == 0:
            self.ball_exit_sensor_tripped = True
        if int(states[self.ball_enter_sensor]) == 0:
            self.ball_enter_sensor_tripped = True

    def check_prox_constantly(self):
        Thread(target=self.check_constantly_thread, daemon=True).start()

    def check_constantly_thread(self):
        while True:
            self.check_sensors()


class Kinect:

    def __init__(self):
        # Initialize the library, if the library is not found, add the library path as argument
        self.combined_image = None
        self.body_image_color = None
        self.depth_color_image = None
        self.body_frame = None
        self.capture = None
        self.Kinect_Motor_Is_On = True
        self.Kinect_Is_On = True
        self.Keyboard_Is_On = False
        pykinect.initialize_libraries(module_k4abt_path="/usr/lib/libk4abt.so", track_body=True)
        self.device_config = pykinect.default_configuration
        self.device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_OFF
        self.device_config.depth_mode = pykinect.K4A_DEPTH_MODE_WFOV_2X2BINNED
        self.device = pykinect.start_device(config=self.device_config)
        self.bodyTracker = pykinect.start_body_tracker()
        self.close_body = None
        self.motor = OdriveMotor("207C34975748", 15, 10)
        self.motor.kinect_motor_calibrate()
        self.motor.check_prox_constantly()

    def start(self):
        Thread(target=self.start_thread, daemon=True).start()
        # self.start_thread()

    def kinect_setup_image(self, showImage: bool = True):
        self.capture = self.device.update()

        self.body_frame = self.bodyTracker.update()

        ret, self.depth_color_image = self.capture.get_colored_depth_image()

        ret, self.body_image_color = self.body_frame.get_segmentation_image()
        if not ret:
            return

        self.combined_image = cv2.addWeighted(self.depth_color_image, 0.6, self.body_image_color, 0.4, 0)

        self.combined_image = self.body_frame.draw_bodies(self.combined_image)

        # resized = cv2.resize(combined_image, (1000, 1000))
        # fliparray
        self.combined_image = numpy.fliplr(self.combined_image)
        self.search_for_closest_body(self.body_frame)
        if showImage:
            cv2.imshow('Depth image with skeleton', self.combined_image)
        cv2.waitKey(1)

    def start_thread(self):
        cv2.namedWindow('Depth image with skeleton', cv2.WINDOW_NORMAL)
        while self.Kinect_Is_On:
            while self.Kinect_Motor_Is_On:
                self.kinect_setup_image()
                # tested, works
                if self.close_body is not None:
                    vel = 2
                    hand_left_y = self.generate_points("left hand").y
                    hand_right_y = self.generate_points("right hand").y
                    hand_right_x = self.generate_points("right hand").x
                    hand_left_x = self.generate_points("left hand").x
                    hand_slope = (hand_left_y - hand_right_y) / (hand_left_x - hand_right_x)

                    if -0.2 < hand_slope < 0.2:
                        # ax.set_ramped_vel(0, 2)
                        self.motor.ax.set_ramped_vel(-(self.motor.ax.get_vel()), 2)
                        # ax.idle() # These idles make motor very jittery - Cesar -cool thanks
                    if hand_slope > 0.2:
                        self.motor.ax.set_ramped_vel(vel, 2)
                    if hand_slope < -0.2:
                        self.motor.ax.set_ramped_vel(-vel, 2)

                    if hand_right_x < -700 or hand_right_x > 700:
                        self.motor.ax.set_ramped_vel(0, 2)
                        # ax.idle()

                    if hand_left_x < -700 or hand_left_x > 700:
                        self.motor.ax.set_ramped_vel(0, 2)
                        # ax.idle()
                if self.motor.ball_exit_sensor_tripped:
                    print("True")
                    self.Kinect_Motor_Is_On = False
            while not self.Kinect_Motor_Is_On:
                self.kinect_setup_image()
                self.move_to_hand()
            sleep(0.1)
            print('sleeping')

    def off(self):
        self.Kinect_Is_On = False

    def on(self):
        self.Kinect_Motor_Is_On = True

    def search_for_closest_body(self, frame):

        body_list = [frame.get_body_skeleton(body_num) for body_num in
                     range(frame.get_num_bodies())]  # creates bodylist
        try:
            self.close_body = min(body_list, key=lambda body: body.joints[
                26].position.xyz.z)  # grabs the minimum body according to the head z depth
        except ValueError:
            self.close_body = None

    def generate_points(self, joint: str):
        if self.close_body is not None:
            return self.close_body.joints[K4ABT_JOINT_NAMES.index(joint)].position.xyz  # return + .xyorz

            # print('no body found', e)

    def moveTo_percent(percX: int, percY: int, seconds=0.01):
        screen_size = size()

        screen_size_x = screen_size[0]

        screen_size_y = screen_size[1]

        moveTo((percX / 100) * screen_size_x, screen_size_y - (percY / 100) * screen_size_y, seconds)

    def move_to_hand(self):
        try:
            x = self.generate_points("right hand").x
            y = self.generate_points("right hand").y
            ly = self.generate_points("left hand").y
            heady = self.generate_points("head").y

            x += 1000
            y += 1000
            percx = x / 2000
            percy = y / 2000
            percx *= 100
            percy *= 100
            percx = int(percx)
            percy = int(percy)
            if ly < heady:
                click()
            self.moveTo_percent(100 - percx, 100 - percy)
        except AttributeError:
            pass


'''
GUI Globals
'''
SCREEN_MANAGER = ScreenManager()
MAIN_SCREEN_NAME = 'main'
'''
GUI Globals
'''


class KinectGUI(App):
    """
    Class to handle running the GUI Application
    """

    def build(self):
        """
        Build the application
        :return: Kivy Screen Manager instance
        """
        return SCREEN_MANAGER


Window.clearcolor = (0, 1, 0, 1)


class MainScreen(Screen):
    """
    Class to handle the main screen and its associated touch events
    """
    button1 = ObjectProperty(None)
    button2 = ObjectProperty(None)

    def pressed(self):
        """
        Function called on button touch event for button with id: testButton
        :return: None
        """
        # print(knect.motor.ax.get_vel())
        pass
        # print(self.button2.x)                x = knect.generate_points("right hand").x


if __name__ == "__main__":
    knect = Kinect()

    try:
        Builder.load_file('main.kv')
        SCREEN_MANAGER.add_widget(MainScreen(name=MAIN_SCREEN_NAME))
        knect.start()
        KinectGUI().run()
    finally:
        knect.off()
        print('ending')
        knect.motor.ax.idle()

# kinect_motor = odrive_motor('207C34975748', 15, 9)
# kinect_motor.calibrate()
# kinect_motor.check_prox_constantly()
#
# kct = Kinect()
# try:
#     kct.start()
#     while True:
#         print("keep me alive!")
# finally:
#     kct.motor.ax.idle()
#     print("idle")
#
#
# # main class file for classes, see paper
# # do we have to trye the entire program:?

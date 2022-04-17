import numpy
from datetime import datetime
import os
import cv2
import pyautogui
import sys
import this
from kivy.uix.button import Button

# from pykinect_azure.k4abt._k4abtTypes import K4ABT_JOINT_NAMES
# from pykinect_azure.k4abt import _k4abt
# import pykinect_azure as pykinect
# from pykinect_azure.k4a import _k4a
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
                mouseDown()
            elif ly >= heady:
                mouseUp()
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


Window.clearcolor = (0.5, 0.5, 0.5, 0.2)


class MainScreen(Screen):
    def on_enter_mainscreen(self):
        self.reset_keyboard_objects()
        self.reset_leaderboard_objects()
        Thread(target=self.timer_object_update).start()


    """
    Section of Class to handle the timer gui and its associated touch events
    """
    timer = ObjectProperty(None)

    def timer_object_update(self):
        self.timer.pos_hint={"x": 0, "y": 0}
        self.timer.font_size = 250

        self.timer.text = "GET READY!"
        sleep(2)
        self.timer.text = "THREE"
        sleep(0.8)
        self.timer.text = "TWO"
        sleep(0.8)
        self.timer.text = "ONE"
        sleep(0.8)
        self.timer.text = "GO!!!"
        sleep(0.7)
        time_start = time.time()
        while True:
            seconds = int(time.time() - time_start)
            self.timer.font_size = 400
            self.timer.text = str(seconds)
            sleep(1)
            if seconds == 1:
                file = open('storage.txt', 'a')
                file.write(str(seconds) + ' ')
                file.close()
                self.set_keyboard_objects()
                temp = self.timer.text
                self.timer.text = "Your Score: " + temp + " seconds"
                self.timer.font_size = 80
                self.timer.pos_hint = {"x": 0, "y": -.38}
                break
            # if knect.Kinect_Motor_Is_On == False:
            #     print("Seconds Passed:", seconds)
            #     file = open('storage.txt', 'a')
            #     file.write('\n' + str(seconds) + '')
            #     file.close()
            #     break

    """
    Section of Class to handle the keyboard gui and its associated touch events
    """
    a1 = ObjectProperty(None)
    b1 = ObjectProperty(None)
    c1 = ObjectProperty(None)
    d1 = ObjectProperty(None)
    e1 = ObjectProperty(None)
    f1 = ObjectProperty(None)
    g1 = ObjectProperty(None)
    h1 = ObjectProperty(None)
    i1 = ObjectProperty(None)
    j1 = ObjectProperty(None)
    k1 = ObjectProperty(None)
    l1 = ObjectProperty(None)
    m1 = ObjectProperty(None)
    n1 = ObjectProperty(None)
    o1 = ObjectProperty(None)
    p1 = ObjectProperty(None)
    q1 = ObjectProperty(None)
    r1 = ObjectProperty(None)
    s1 = ObjectProperty(None)
    t1 = ObjectProperty(None)
    u1 = ObjectProperty(None)
    v1 = ObjectProperty(None)
    w1 = ObjectProperty(None)
    x1 = ObjectProperty(None)
    y1 = ObjectProperty(None)
    z1 = ObjectProperty(None)
    space = ObjectProperty(None)
    star = ObjectProperty(None)
    dash = ObjectProperty(None)
    delete = ObjectProperty(None)
    enter = ObjectProperty(None)
    nicknamekv = ObjectProperty(None)

    # cursormovement
    square = ObjectProperty(None)

    nickname = ""

    def set_keyboard_objects(self, color: str = "lightblue"):
        keyboard_buttons = [self.q1, self.w1, self.e1, self.r1, self.t1, self.y1, self.u1, self.i1, self.o1, self.p1,
                            self.a1, self.s1, self.d1, self.f1, self.g1, self.h1, self.j1, self.k1, self.l1, self.space,
                            self.z1, self.x1, self.c1, self.v1, self.b1, self.n1, self.m1, self.star, self.dash,
                            self.delete, self.enter]
        x_spacing = .1
        y_spacing = -.15
        x_offset = 0
        y_offset = 0
        button_count = 0
        for btn in keyboard_buttons:
            btn.pos_hint = {"x": .02 + x_offset, "y": .55 + y_offset}
            button_count += 1
            x_offset += x_spacing
            btn.color = color
            if button_count % 10 == 0:
                x_offset = 0
                y_offset += y_spacing

        self.enter.pos_hint = {"x": 0.1, "y": .1}
        self.nicknamekv.pos_hint = {"x": 0, "y": .3}

    def reset_keyboard_objects(self, color: str = "lightblue"):
        offset_num = 1.5
        keyboard_buttons = [self.q1, self.w1, self.e1, self.r1, self.t1, self.y1, self.u1, self.i1, self.o1, self.p1,
                            self.a1, self.s1, self.d1, self.f1, self.g1, self.h1, self.j1, self.k1, self.l1, self.space,
                            self.z1, self.x1, self.c1, self.v1, self.b1, self.n1, self.m1, self.star, self.dash,
                            self.delete, self.enter]
        x_spacing = .1
        y_spacing = -.15
        x_offset = 0
        y_offset = 0
        button_count = 0
        for btn in keyboard_buttons:
            btn.pos_hint = {"x": offset_num + x_offset, "y": offset_num + y_offset}
            button_count += 1
            x_offset += x_spacing
            btn.color = color
            if button_count % 10 == 0:
                x_offset = 0
                y_offset += y_spacing

        self.enter.pos_hint = {"x": offset_num, "y": offset_num}
        self.nicknamekv.pos_hint = {"x": offset_num, "y": offset_num}
        self.nicknamekv.text = "Move your right hand to move the mouse,\nmove your left above your head to click!"

    def letter_key_update(self, button):
        self.nickname += button.text
        first_caps = self.nickname[0].upper()
        self.nickname = self.nickname[1:]
        self.nickname = first_caps + self.nickname
        self.timer_update()

    def delete_key_update(self):
        self.nickname = self.nickname[:-1]
        self.timer_update()

    def enter_key_update(self):
        if len(self.nickname) > 1:
            if self.nickname != "Not A Valid Input!":
                with open("storage.txt", "a") as f:
                    f.write(str(self.nickname + "\n"))
                    self.nickname = ""
                self.reset_keyboard_objects()
                self.score_update()
                self.set_leaderboard_objects()
        else:
            self.nicknamekv.text = "Not A Valid Input!"
            self.nickname = ""

    def timer_update(self):
        self.nicknamekv.text = str(self.nickname)
        self.profanity_check(self.nickname)
        # self.nickname[0:] = self.nickname[0:].upper() # trying to uppercase the first char of string

    def profanity_check(self, streng: str):
        profanity_list = ["fuc", 'bit']
        for i in range(len(profanity_list)):
            if profanity_list[i] in self.nickname.lower():
                self.nickname = ""
                self.nicknamekv.text = ""

    """
        Section of Class to handle the leaderboard gui and its associated events
    """
    first_place = ObjectProperty(None)
    leaderboard_text = ObjectProperty(None)
    pairsList = None
    def set_leaderboard_objects(self):
        self.first_place.pos_hint = {"x": 0, "y": 0}
        self.leaderboard_text.pos_hint = {"x": 0, "y": 0.44}

    def reset_leaderboard_objects(self):
        offset = 1.9
        self.first_place.pos_hint = {"x": offset, "y": 0}
        self.leaderboard_text.pos_hint = {"x": offset, "y": 0.44}



    def score_update(self):

        scores = []
        names = []
        with open("storage.txt", "r") as file:
            for line in file:
                split_line = line.strip().split()
                scores.append(split_line[0])
                names.append(split_line[1])


        pairs = list(zip(scores, names))
        pairs.sort(key=lambda pair: int(pair[0]))
        self.pairsList = dict(pairs)

        count = 1
        score_board = ""
        while count < 11:
            score_board += str(count) + ". " + pairs[count][0] + " " + pairs[count][1] + "\n"
            count += 1
        self.first_place.text = score_board


        Thread(target=self.testfunctiondeletelater).start()
    def testfunctiondeletelater(self):
        sleep(3)
        self.on_enter_mainscreen()

def email_process():
    os.system("python3 KineticMail.py")
# start (i already know how to play option), timer, keyboard, leaderboard, start
if __name__ == "__main__":
    # knect = Kinect()
    try:
        Thread(target=email_process, daemon=True).start()
        Builder.load_file('main.kv')
        SCREEN_MANAGER.add_widget(MainScreen(name=MAIN_SCREEN_NAME))
        # knect.start()
        KinectGUI().run()
    finally:
        # knect.off()
        print('ending')
        # knect.motor.ax.idle()

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

import re
import sys
from multiprocessing import Process

import cv2
import numpy as np
from detecto import core, utils
import DobotSDK
from PIL import Image
from time import sleep

stop = False
firstTime = True
secondTime = True

class findAndGotoTarget:

    def __init__(self, targetToCheck):
        self.api = DobotSDK.load()
        self.result_connect = DobotSDK.ConnectDobot(self.api, "192.168.1.6")
        self.model = core.Model.load('model_weights.pth', ['robo', 'dfan', 'slant'])
        self.targetToCheck = targetToCheck
        self.labels = None
        self.boxes = None
        self.scores = None
        self.speed = -2

        self.border_x = [0, 375]
        self.border_y = [-275, 250]
        self.border_z = [-55, 120]

        self.border_x_targetsContainer = [200, 375]
        self.border_y_targetsContainer = [-15, -250]

        self.firstTimeMove = True
        self.previousDisatance = 0.0

    def get_screenshot_and_locations(self, value = 1.25, quality = 100):

        webcam = cv2.VideoCapture(1)
        check, frame = webcam.read()
        cv2.imwrite(filename=r'saved_img.png', img=frame)
        webcam.release()

        im = Image.open('saved_img.png')

        source = im.split()

        R, G, B = 0, 1, 2
        constant = value  # constant by which each pixel is divided

        Red = source[R].point(lambda i: i / constant)
        Green = source[G].point(lambda i: i / constant)
        Blue = source[B].point(lambda i: i / constant)

        im = Image.merge(im.mode, (Red, Green, Blue))

        im.save('saved_img.png', 'PNG', quality=quality)

        image = utils.read_image(r'saved_img.png')
        predictions = self.model.predict(image)

        labels, boxes, scores = predictions

        labels_new, boxes_new, scores_new = [], [], []

        for index, value in enumerate(scores.tolist()):
            j = np.asarray(boxes.tolist())
            if value >= 0.75:
                labels_new.append(labels[index])
                boxes_new.append(j[index])
                scores_new.append(scores[index])

        print(labels_new)
        print(boxes_new)
        print(scores_new)
        
        self.labels = labels_new
        self.boxes = boxes_new
        self.scores = scores_new

    def check_targets(self):

        global stop
        X_value = 0
        Y_value = 0
        while not stop:
            if self.targetToCheck not in self.labels:
                default_position = DobotSDK.GetExchange(self.api)[9]
                print(default_position)

                if default_position[0] < self.border_x[0] or default_position[0] > self.border_x[1] or default_position[1] < self.border_y[0] or default_position[1] > self.border_y[1] or default_position[2] < self.border_z[0] or default_position[2] > self.border_z[1]:
                    print("Parallelogram Error during check_targets!!!!!!")
                    sys.exit(1)

                if self.firstTimeMove:

                    X_value = (self.border_x_targetsContainer[0] + self.border_x_targetsContainer[1]) / 2
                    # if x < default_position[0]:
                    #     X_value = -10
                    # else:
                    #     X_value = 10

                    y = (self.border_y_targetsContainer[0] + self.border_y_targetsContainer[1] ) / 1.25
                    if y < default_position[1]:
                        Y_value = -10
                    else:
                        Y_value = 10

                    self.firstTimeMove = False

                default_position[0] = X_value
                default_position[1] = default_position[1] + Y_value
                default_position[2] = abs((self.border_z[0] + self.border_z[1]))
                default_position[3] = 100
                DobotSDK.MovJ(self.api, default_position, isBlock=True)

                self.get_screenshot_and_locations()
            else:
                stop = True
                default_position = DobotSDK.GetExchange(self.api)[9]
                default_position[1] = default_position[1] + Y_value + Y_value
                DobotSDK.MovJ(self.api, default_position, isBlock=True)

        
    def gotoTarget_x(self):

        if "robo" in self.labels and self.targetToCheck in self.labels:
            robo_box_x = None
            dfan_box_x = None
            global stop, firstTime, secondTime
            for index, value in enumerate(self.labels):
                if value == "robo":
                    robo_box = (self.boxes[index]).tolist()
                    robo_box_x = (robo_box[2] + robo_box[0]) / 2
                if value == self.targetToCheck:
                    dfan_box = (self.boxes[index]).tolist()
                    dfan_box_x = (dfan_box[2] + dfan_box[0]) / 2
                if robo_box_x is not None and dfan_box_x is not None:
                    goto = (robo_box_x - dfan_box_x)
                    print(goto)

                    default_position = DobotSDK.GetExchange(self.api)[9]
                    ls = [-20, -21, -22, -23, -24, -25]
                    if default_position[0] < (self.border_x_targetsContainer[0] + self.border_x_targetsContainer[1]) / 2:
                        ls = [-30, -31, -32, -33, -34, -35]
                    ls = list(range(-25, -16))
                    if int(goto) in ls:
                        stop = True
                        firstTime = False
                        break

                    if secondTime and not firstTime:
                        if abs(goto) >= abs(self.previousDisatance):
                            self.speed = -self.speed
                        secondTime = False

                    if firstTime:
                        if int(goto) >= ls[2]:
                            self.speed = 2
                        else:
                            self.speed = -2
                        firstTime = False
                        self.previousDisatance = goto

                    print(self.speed)

                    default_position[0] = default_position[0] + self.speed
                    default_position[3] = 100

                    if default_position[0] < self.border_x[0] or default_position[0] > self.border_x[1] or \
                            default_position[1] < self.border_y[0] or default_position[1] > self.border_y[1] or \
                            default_position[2] < self.border_z[0] or default_position[2] > self.border_z[1]:
                        print("Parallelogram Error during gototarget_x!!!!!!")
                        sys.exit(1)

                    DobotSDK.MovJ(self.api, default_position, isBlock=True)
                    break
        else:
            stop = False
            self.check_targets()

    def gotoTarget_x_confirmation(self):
        robo_box_x = None
        dfan_box_x = None
        global stop
        for index, value in enumerate(self.labels):
            if value == "robo":
                robo_box = (self.boxes[index]).tolist()
                robo_box_x = (robo_box[2] + robo_box[0]) / 2
            if value == self.targetToCheck:
                dfan_box = (self.boxes[index]).tolist()
                dfan_box_x = (dfan_box[2] + dfan_box[0]) / 2
            if robo_box_x is not None and dfan_box_x is not None:
                previous_goto = (robo_box_x - dfan_box_x)
                print(previous_goto)

                if self.speed == 2:
                    self.speed = -2
                else:
                    self.speed = 2

                default_position = DobotSDK.GetExchange(self.api)[9]

                default_position[0] = default_position[0] + self.speed
                default_position[3] = 100

                if default_position[0] < self.border_x[0] or default_position[0] > self.border_x[1] or \
                        default_position[1] < self.border_y[0] or default_position[1] > self.border_y[1] or \
                        default_position[2] < self.border_z[0] or default_position[2] > self.border_z[1]:
                    print("Parallelogram Error during gototarget_x_confirmation!!!!!!")
                    sys.exit(1)

                DobotSDK.MovJ(self.api, default_position, isBlock=True)

                self.get_screenshot_and_locations()

                for index, value in enumerate(self.labels):
                    if value == "robo":
                        robo_box = (self.boxes[index]).tolist()
                        robo_box_x = (robo_box[2] + robo_box[0]) / 2
                    if value == self.targetToCheck:
                        dfan_box = (self.boxes[index]).tolist()
                        dfan_box_x = (dfan_box[2] + dfan_box[0]) / 2
                    if robo_box_x is not None and dfan_box_x is not None:
                        new_goto = (robo_box_x - dfan_box_x)
                        print(new_goto)

                        if new_goto > previous_goto:

                            if self.speed == 2:
                                self.speed = -2
                            else:
                                self.speed = 2

                            default_position = DobotSDK.GetExchange(self.api)[9]

                            default_position[0] = default_position[0] + self.speed
                            default_position[3] = 100

                            if default_position[0] < self.border_x[0] or default_position[0] > self.border_x[1] or \
                                    default_position[1] < self.border_y[0] or default_position[1] > self.border_y[1] or \
                                    default_position[2] < self.border_z[0] or default_position[2] > self.border_z[1]:
                                print("Parallelogram Error during gototarget_x_confirmation!!!!!!")
                                sys.exit(1)

                            DobotSDK.MovJ(self.api, default_position, isBlock=True)

                            stop = True

    def gotoTarget_y(self):

        if "robo" in self.labels and self.targetToCheck in self.labels:
            robo_box_y = None
            dfan_box_y = None
            global stop, firstTime, secondTime
            for index, value in enumerate(self.labels):
                if value == "robo":
                    robo_box = (self.boxes[index]).tolist()
                    robo_box_y = (robo_box[3] + robo_box[1]) / 2
                if value == self.targetToCheck:
                    dfan_box = (self.boxes[index]).tolist()
                    dfan_box_y = (dfan_box[3] + dfan_box[1]) / 2
                if robo_box_y is not None and dfan_box_y is not None:
                    goto = (robo_box_y - dfan_box_y)
                    print(goto)
                    ls = [283, 284, 285, 286, 287, 278]
                    ls = list(range(277, 292))
                    if int(goto) in ls:
                        stop = True
                        break

                    if secondTime and not firstTime:
                        if abs(goto) >= abs(self.previousDisatance):
                            self.speed = -self.speed
                        secondTime = False

                    if firstTime:
                        if int(goto) >= ls[2]:
                            self.speed = -2
                        else:
                            self.speed = 2
                        firstTime = False
                        self.previousDisatance = goto

                    print(self.speed)
                    default_position = DobotSDK.GetExchange(self.api)[9]
                    default_position[1] = default_position[1] + self.speed
                    default_position[3] = 100

                    if default_position[0] < self.border_x[0] or default_position[0] > self.border_x[1] or \
                            default_position[1] < self.border_y[0] or default_position[1] > self.border_y[1] or \
                            default_position[2] < self.border_z[0] or default_position[2] > self.border_z[1]:
                        print("Parallelogram Error during gototarget_y!!!!!!")
                        sys.exit(1)

                    DobotSDK.MovJ(self.api, default_position, isBlock=True)
                    break
        else:
            stop = False
            self.check_targets()

    def gotoTarget_y_confirmation(self):
        robo_box_x = None
        dfan_box_x = None
        global stop
        for index, value in enumerate(self.labels):
            if value == "robo":
                robo_box = (self.boxes[index]).tolist()
                robo_box_y = (robo_box[3] + robo_box[0]) / 2
            if value == self.targetToCheck:
                dfan_box = (self.boxes[index]).tolist()
                dfan_box_y = (dfan_box[3] + dfan_box[0]) / 2
            if robo_box_y is not None and dfan_box_y is not None:
                previous_goto = (robo_box_y - dfan_box_y)
                print(previous_goto)

                if self.speed == 2:
                    self.speed = -2
                else:
                    self.speed = 2

                default_position = DobotSDK.GetExchange(self.api)[9]

                default_position[0] = default_position[0] + self.speed
                default_position[3] = 100

                if default_position[0] < self.border_x[0] or default_position[0] > self.border_x[1] or \
                        default_position[1] < self.border_y[0] or default_position[1] > self.border_y[1] or \
                        default_position[2] < self.border_z[0] or default_position[2] > self.border_z[1]:
                    print("Parallelogram Error during gototarget_x_confirmation!!!!!!")
                    sys.exit(1)

                DobotSDK.MovJ(self.api, default_position, isBlock=True)

                self.get_screenshot_and_locations()

                for index, value in enumerate(self.labels):
                    if value == "robo":
                        robo_box = (self.boxes[index]).tolist()
                        robo_box_x = (robo_box[2] + robo_box[0]) / 2
                    if value == self.targetToCheck:
                        dfan_box = (self.boxes[index]).tolist()
                        dfan_box_x = (dfan_box[2] + dfan_box[0]) / 2
                    if robo_box_x is not None and dfan_box_x is not None:
                        new_goto = (robo_box_x - dfan_box_x)
                        print(new_goto)

                        if new_goto > previous_goto:

                            if self.speed == 2:
                                self.speed = -2
                            else:
                                self.speed = 2

                            default_position = DobotSDK.GetExchange(self.api)[9]

                            default_position[0] = default_position[0] + self.speed
                            default_position[3] = 100

                            if default_position[0] < self.border_x[0] or default_position[0] > self.border_x[1] or \
                                    default_position[1] < self.border_y[0] or default_position[1] > self.border_y[1] or \
                                    default_position[2] < self.border_z[0] or default_position[2] > self.border_z[1]:
                                print("Parallelogram Error during gototarget_x_confirmation!!!!!!")
                                sys.exit(1)

                            DobotSDK.MovJ(self.api, default_position, isBlock=True)

                            stop = True

    def main(self):
        print(f"Result Connection: {self.result_connect}")
        global stop, firstTime

        while not stop:
            self.get_screenshot_and_locations()
            if "robo" not in self.labels:
                self.get_screenshot_and_locations(1)
            if "robo" not in self.labels:
                self.get_screenshot_and_locations(1.25, quality = 50)
            self.check_targets()

        #while default_position[2] > -46 and int(default_position[2]) in [64, 65, 66]:
        stop = False
        firstTime = True
        self.speed = -2
        print("#############")
        print("Moving X Axis")
        print("#############")
        while not stop:
            self.get_screenshot_and_locations()
            if "robo" not in self.labels:
                self.get_screenshot_and_locations(1)
            if "robo" not in self.labels:
                self.get_screenshot_and_locations(1.25, quality = 50)
            self.gotoTarget_x()

        stop = False
        while not stop:
            self.get_screenshot_and_locations()
            if "robo" not in self.labels:
                self.get_screenshot_and_locations(1)
            if "robo" not in self.labels:
                self.get_screenshot_and_locations(1.25, quality=50)
            self.gotoTarget_x_confirmation()

        stop = False
        firstTime = True
        self.speed = -2
        print("#############")
        print("Moving Y Axis")
        print("#############")
        while not stop:
            self.get_screenshot_and_locations()
            if "robo" not in self.labels:
                self.get_screenshot_and_locations(1)
            if "robo" not in self.labels:
                self.get_screenshot_and_locations(1.25, quality = 50)
            self.gotoTarget_y()

        default_position = DobotSDK.GetExchange(self.api)[9]
        print(default_position)
        default_position[2] = -54
        DobotSDK.MovJ(self.api, default_position, isBlock=True)

        cv2.destroyAllWindows()

        DobotSDK.DisconnectDobot(self.api)

        sleep(1)

        DobotSDK.ConnectDobot(self.api, "192.168.1.6")

        sleep(1)

        num = {1: True, 2: False, 3: False, 4: False, 5: False, 6: False, 7: False, 8: False, 9: False, 10: False,
               11: False, 12: False, 13: False, 14: False, 15: False, 16: False, 17: False, 18: False, 19: False,
               20: False, 21: False, 22: False, 23: False, 24: False, 25: False, 26: False, 27: False, 28: False,
               29: False, 30: False, 31: False, 32: False, 33: False, 34: False, 35: False, 36: False, 37: False,
               38: False, 39: False, 40: False, 41: False, 42: False, 43: False, 44: False, 45: False, 46: False,
               47: False, 48: False, 49: False, 50: False, 51: False, 52: False, 53: False, 54: False, 55: False,
               56: False, 57: False, 58: False, 59: False, 60: False, 61: False, 62: False, 63: False, 64: False}
        DobotSDK.SetDO(self.api, num)

        sleep(1)

        DobotSDK.DisconnectDobot(self.api)

        sleep(1)

        DobotSDK.ConnectDobot(self.api, "192.168.1.6")

        sleep(1)

        num = {1: False, 2: False, 3: False, 4: False, 5: False, 6: False, 7: False, 8: False, 9: False, 10: False,
               11: False, 12: False, 13: False, 14: False, 15: False, 16: False, 17: False, 18: False, 19: False,
               20: False, 21: False, 22: False, 23: False, 24: False, 25: False, 26: False, 27: False, 28: False,
               29: False, 30: False, 31: False, 32: False, 33: False, 34: False, 35: False, 36: False, 37: False,
               38: False, 39: False, 40: False, 41: False, 42: False, 43: False, 44: False, 45: False, 46: False,
               47: False, 48: False, 49: False, 50: False, 51: False, 52: False, 53: False, 54: False, 55: False,
               56: False, 57: False, 58: False, 59: False, 60: False, 61: False, 62: False, 63: False, 64: False}
        DobotSDK.SetDO(self.api, num)


if __name__ == "__main__":
    ft = findAndGotoTarget("dfan")
    ft.main()

# read all image, run classifier to label

import cv2
import numpy as np
import os
import sys
import csv

from os import path
current_dir = path.dirname(path.abspath(__file__))

UNKNOWN=4
GREEN=2
YELLOW=1
RED=0

class Prep(object):
    def __init__(self):
        pass

    def Load(self):
        image_dir = path.join(current_dir, 'images')
        files = sorted(os.listdir(image_dir))

        with open(path.join(current_dir, 'dict.csv'), 'wb') as csv_file:
            writer = csv.writer(csv_file)
            for file in files:
                image_path = os.path.join(image_dir, file)
                image = cv2.imread(image_path)
                result = self.get_classification(image)
                writer.writerow([file, result])

    def get_classification(self, image):
        h, w, c = image.shape
        total_area = h * w

        if total_area < 10000:
            print('Light is too small')
            return UNKNOWN

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        inv = cv2.bitwise_not(gray)
        _, th = cv2.threshold(inv, 220, 255, cv2.THRESH_TOZERO)
        _, contours, _ = cv2.findContours(th, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        cntr = []
        for c in contours:
            area = cv2.contourArea(c) / total_area
            if 0.01 < area and area < 0.2:
                cntr.append(c)

        mask = np.zeros_like(image[:, :, 0])
        mask = cv2.drawContours(mask, cntr, -1, 255, -1)
        cut = cv2.bitwise_and(image, image, mask=mask)

        blue = cv2.calcHist([cut], [0], None, [3], [100, 250]).item(2)
        green = cv2.calcHist([cut], [1], None, [3], [100, 250]).item(2)
        red = cv2.calcHist([cut], [2], None, [3], [100, 250]).item(2)

        norm = blue + green + red
        if norm > 0:
            b = blue / norm
            g = green / norm
            r = red / norm

            if g > 0.4 and r > 0.4:
                print('Light is YELLOW')
                return YELLOW
            elif r > 0.5:
                print('Light is RED')
                return RED
            elif g > 0.5:
                print('Light is GREEN')
                return GREEN
            else:
                print('Light is UNKNOWN')
                return UNKNOWN

        print("norm is 0")
        return UNKNOWN

p = Prep()
p.Load()
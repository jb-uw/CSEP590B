#!/usr/bin/env python3

import asyncio
import sys

import cv2
import numpy as np
import cozmo

from cozmo.util import degrees, distance_mm

import imgclassification

try:
    from PIL import ImageDraw, ImageFont, Image
except ImportError:
    sys.exit('run `pip3 install --user Pillow numpy` to run this example')
import time

def run(robot: cozmo.robot.Robot):
    '''The run method runs once the Cozmo SDK is connected.'''

	# Move lift down and tilt the head up
    robot.set_lift_height(0).wait_for_completed()
    robot.set_head_angle(degrees(0)).wait_for_completed()
    
    img_clf = imgclassification.ImageClassifier()
    (train_raw, train_labels) = img_clf.load_data_from_folder('./train/')
    (test_raw, test_labels) = img_clf.load_data_from_folder('./test/')
    raw = np.concatenate((train_raw, test_raw))
    labels = np.concatenate((train_labels, test_labels))
    # convert images into features
    train_data = img_clf.extract_image_features(raw)
    img_clf.train_classifier(train_data, labels)
    sliding_window = []
    id_count = {'drone': 0, 'hands': 0, 'inspection': 0, 'none': 0, 'order': 0, 'place': 0, 'plane': 0, 'truck': 0}
    
    try:
        while True:
            predicted_labels = []
            id = None
            #get camera image
            event = robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)

            #convert camera image to opencv format
            opencv_image = cv2.cvtColor(np.asarray(event.image), cv2.COLOR_RGB2GRAY)
            image = np.array([opencv_image])
            test_data = img_clf.extract_image_features(image)
            
            predicted_labels = img_clf.predict_labels(test_data)
            
            # a sliding window of 10 ids
            if len(sliding_window) >= 10:
                old_id = sliding_window.pop(0)
                id_count[old_id] -= 1
            sliding_window.append(predicted_labels[0])
            id_count[predicted_labels[0]] += 1

            # an id is confirmed if 8 out of 10 is the same image
            for i in list(id_count.keys()):
                if id_count[i] >= 8:
                    id = i
                    break
                    
            if id == 'plane':
                robot.say_text("I see plane").wait_for_completed()
                robot.set_lift_height(1).wait_for_completed()
                robot.set_lift_height(0).wait_for_completed()
                id_count = {'drone': 0, 'hands': 0, 'inspection': 0, 'none': 0, 'order': 0, 'place': 0, 'plane': 0, 'truck': 0}
                sliding_window.clear()
                time.sleep(2)
            elif id == 'order':
                robot.say_text("I see order").wait_for_completed()
                robot.play_anim(name="anim_poked_giggle").wait_for_completed()
                id_count = {'drone': 0, 'hands': 0, 'inspection': 0, 'none': 0, 'order': 0, 'place': 0, 'plane': 0, 'truck': 0}
                sliding_window.clear()
                time.sleep(2)
            elif id == 'drone':
                robot.say_text("I see drone").wait_for_completed()
                robot.play_anim_trigger(cozmo.anim.Triggers.CubePounceLoseSession,ignore_body_track=True).wait_for_completed()
                id_count = {'drone': 0, 'hands': 0, 'inspection': 0, 'none': 0, 'order': 0, 'place': 0, 'plane': 0, 'truck': 0}
                sliding_window.clear()
                time.sleep(2)
            elif id == 'inspection':
                robot.say_text("I see inspection").wait_for_completed()
                robot.set_head_angle(degrees(45)).wait_for_completed()
                robot.set_head_angle(degrees(0)).wait_for_completed()
                id_count = {'drone': 0, 'hands': 0, 'inspection': 0, 'none': 0, 'order': 0, 'place': 0, 'plane': 0, 'truck': 0}
                sliding_window.clear()
                time.sleep(2)
            elif id == 'place':
                robot.say_text("I see place").wait_for_completed()
                robot.play_anim_trigger(cozmo.anim.Triggers.Shocked,ignore_body_track=True).wait_for_completed()
                id_count = {'drone': 0, 'hands': 0, 'inspection': 0, 'none': 0, 'order': 0, 'place': 0, 'plane': 0, 'truck': 0}
                sliding_window.clear()
                time.sleep(2)
            elif id == 'hands':
                robot.say_text("I see hands").wait_for_completed()
                robot.play_anim_trigger(cozmo.anim.Triggers.Shiver,ignore_body_track=True).wait_for_completed()
                id_count = {'drone': 0, 'hands': 0, 'inspection': 0, 'none': 0, 'order': 0, 'place': 0, 'plane': 0, 'truck': 0}
                sliding_window.clear()
                time.sleep(2)
            elif id == 'truck':
                robot.say_text("I see truck").wait_for_completed()
                robot.play_anim_trigger(cozmo.anim.Triggers.Surprise,ignore_body_track=True).wait_for_completed()
                id_count = {'drone': 0, 'hands': 0, 'inspection': 0, 'none': 0, 'order': 0, 'place': 0, 'plane': 0, 'truck': 0}
                sliding_window.clear()
                time.sleep(2)
            
    except KeyboardInterrupt:
        print("")
        print("Exit requested by user")
    except cozmo.RobotBusy as e:
        print(e)



if __name__ == '__main__':
    cozmo.run_program(run, use_viewer = True, force_viewer_on_top = True)


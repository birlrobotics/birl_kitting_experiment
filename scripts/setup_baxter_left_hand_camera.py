#!/usr/bin/env python

from baxter_interface import CameraController

if __name__ == '__main__':
    camera = CameraController("left_hand_camera")
    camera.open()
    res = (1280,800)
    camera.resolution = res
    camera.exposure = camera.CONTROL_AUTO
    camera.fps = 1
    camera.gain = camera.CONTROL_AUTO

#!/usr/bin/env python3
"""
Shows how to toss a capsule to a container.
"""
from mujoco_py import load_model_from_path, MjSim, MjViewer
from mujoco_py.generated import const
import os
import cv2
import numpy as np
import glfw
import pickle
import math


# from mujoco_py import GlfwContext
# GlfwContext(offscreen=True)

model = load_model_from_path("model.xml")
sim = MjSim(model)
viewer = MjViewer(sim)
viewer.cam.type = const.CAMERA_FREE
viewer.cam.fixedcamid = 0
# Important: set _hide_overlay=False in MjViewer()

sim_state = sim.get_state()

recording = False
quit = False
all_data=[]
time_step=0
thetas = np.linspace(-180, 180, 50, endpoint=False)
initial_cam_azimuth = None
initial_cam_elevation = None
sweep_camera_spin = 0
# while True:
#     sim.set_state(sim_state)

d_theta = 0.1
while not quit:
    if time_step > 400:
        sim.set_state(sim_state)
        time_step = 0
        input()
    
    fovy = sim.model.cam_fovy[:]
    # print("camera fovy:",fovy)

    for i in range(20):
        if time_step < 210:
            sim.data.ctrl[:] = 0.0
        else:
            sim.data.ctrl[:] = -1.0
        sim.step()
        time_step += 1
    
    # res0 = sim.render(512, 512, camera_name="cam0")
    # res1 = sim.render(512, 512, camera_name="cam1")
    # res.viewer.cam.distance +=1
    # print(type(res), res.shape)
    # cv2.imshow("res0",res0)
    # cv2.imshow("res1",res1)
    # cv2.waitKey(1)

    resolution = glfw.get_framebuffer_size(sim._render_context_window.window)
    resolution = np.array(resolution)
    resolution = resolution * min(1000 / np.min(resolution), 1)
    resolution = resolution.astype(np.int32)
    resolution -= resolution % 16
    # print(resolution[0],resolution[1])
    


    for _,theta in enumerate(thetas):
        viewer.render()

        data = np.asarray(viewer.read_pixels(resolution[0], resolution[1], depth=False)[::-1, :, :], dtype=np.uint8)
        
        if initial_cam_elevation is None:
             initial_cam_azimuth = viewer.cam.azimuth
             initial_cam_elevation = viewer.cam.elevation

        viewer.cam.azimuth=initial_cam_azimuth + theta
        viewer.cam.elevation=initial_cam_elevation + 20*math.sin(4*math.radians(theta))
        viewer.cam.distance = 4
        # viewer.cam.lookat[0]+=0.01
        # print("Lookat: ", viewer.cam.lookat)
        # print("Azimuth: ", viewer.cam.azimuth)
        # print("Elevation: ", viewer.cam.elevation)
        print("Position: ",sim.data.get_camera_xpos('cam0'))
        print("Position: ",viewer.get_pose())
        # input()

        # save data

        if data is not None:
            cv2img = cv2.cvtColor(data,cv2.COLOR_RGB2BGR)
            cv2.imshow("img:".format(time_step), cv2img)
            k = cv2.waitKey(1)
            if k==ord('r'):
                recording = True
                print('recording')
            elif k==ord('s'):
                recording = False
            elif k==ord('q'):
                quit = True
                break
                
            if recording:
                all_data.append({'time':time_step,'image':data, 'azimuth':viewer.cam.azimuth, \
                'elevation':viewer.cam.elevation, 'distance':viewer.cam.distance,  \
                'lookat':viewer.cam.lookat})


# Saving the objects:
with open('all_data.pkl', 'wb') as f:  # Python 3: open(..., 'wb')
    pickle.dump(all_data, f)


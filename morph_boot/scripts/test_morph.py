# import mujoco_py
# # model = mujoco_py.load_model_from_path('/home/miller/Desktop/projects/derl/morph_boot/morph_xml/derl_xml/0-13-28-01-13-42.xml')
# model = mujoco_py.load_model_from_path('/home/miller/Desktop/projects/derl/morph_boot/morph_xml/chatGPT_xml/six_axis_arm.xml')
# sim = mujoco_py.MjSim(model)
# viewer = mujoco_py.MjViewer(sim)
# while True:
#     viewer.render()
#     # sim.step()

# import mujoco_py
# import os
# import numpy as np
#
# # Load the model
# model_path = "/home/miller/Desktop/projects/derl/morph_boot/morph_xml/chatGPT_xml/three_axis_arm.xml"
# model = mujoco_py.load_model_from_path(model_path)
# sim = mujoco_py.MjSim(model)
#
# # Set up the viewer
# viewer = mujoco_py.MjViewer(sim)
#
# # Set the joint indices
# joint1_idx = sim.model.get_joint_qpos_addr("joint1")
# joint2_idx = sim.model.get_joint_qpos_addr("joint2")
# joint3_idx = sim.model.get_joint_qpos_addr("joint3")
# joint4_idx = sim.model.get_joint_qpos_addr("joint4")
# left_finger_idx = sim.model.get_joint_qpos_addr("left_finger_joint")
# right_finger_idx = sim.model.get_joint_qpos_addr("right_finger_joint")
#
# # Simulate the environment for 1000 steps
# # for _ in range(1000):
# while 1:
#     # Apply random torques to the motors
#     torques = np.zeros(sim.model.nu)
#     torques[joint1_idx] = np.random.uniform(-1, 1)
#     torques[joint2_idx] = np.random.uniform(-1, 1)
#     torques[joint3_idx] = np.random.uniform(-1, 1)
#     torques[joint4_idx] = np.random.uniform(-1, 1)
#     torques[left_finger_idx] = np.random.uniform(-1, 1)
#     torques[right_finger_idx] = np.random.uniform(-1, 1)
#
#     sim.data.ctrl[:] = torques
#
#     # Step the simulation
#     sim.step()
#     viewer.render()


# #/home/miller/Desktop/projects/derl/morph_boot/morph_xml/chatGPT_xml/six_axis_arm.xml
#
# import mujoco_py
# import numpy as np
# import os
# import math
#
# # Load the simplified ABB IRB 120 model
# model = mujoco_py.load_model_from_path("/home/miller/Desktop/projects/derl/morph_boot/morph_xml/chatGPT_xml/six_axis_arm.xml")
# sim = mujoco_py.MjSim(model)
#
# # Create a viewer for visualization
# viewer = mujoco_py.MjViewer(sim)
#
# # Run the simulation
# step = 0
# while True:
#     # Apply sinusoidal control inputs to each joint with different frequencies and phases
#     for i in range(6):
#         sim.data.ctrl[i] = 20 * math.sin(step * 0.1 + i * math.pi / 3)
#
#     # Advance the simulation by one step
#     sim.step()
#
#     # Update the viewer
#     viewer.render()
#
#     # Increment the step counter
#     step += 1

#/home/miller/Desktop/projects/derl/morph_boot/morph_xml/chatGPT_xml/six_axis_arm.xml

import mujoco_py
import numpy as np
import os
import math
import time

# Load the simplified ABB IRB 120 model
model = mujoco_py.load_model_from_path("/home/miller/Desktop/projects/derl/morph_boot/morph_xml/chatGPT_xml/six_axis_arm.xml")
sim = mujoco_py.MjSim(model)

# Create a viewer for visualization
viewer = mujoco_py.MjViewer(sim)

# Define the number of steps per motion
steps_per_motion = 200

# Define joint motion sequences
joint_motion_sequences = [
    np.linspace(0, 2 * np.pi, steps_per_motion),
    np.linspace(-np.pi / 2, np.pi / 2, steps_per_motion),
    np.linspace(-np.pi / 2, np.pi / 2, steps_per_motion),
    np.linspace(-np.pi / 2, np.pi / 2, steps_per_motion),
    np.linspace(-np.pi / 2, np.pi / 2, steps_per_motion),
    np.linspace(-np.pi, np.pi, steps_per_motion)
]

# Move each joint sequentially
while True:
    for joint_idx, motion_sequence in enumerate(joint_motion_sequences):
        for angle in motion_sequence:
            sim.data.ctrl[joint_idx] = 1.5 * math.sin(angle * 2.0) # increase amplitude and frequency of motion
            sim.step()
            viewer.render()
            time.sleep(0.001) # remove pause between motions






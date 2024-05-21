"""
This tutorial is a extension of the robot_arms.ipynb.
The aim of this project is show the mujoco embed GUI and how we can interact with our robot through mouse by utilizing mocap

There is a known issue with Glfw Wayland with error messages like 'Wayland: Window position retrieval not supported'.
A quick solution to this is using X11: Restart the computer and before you login to ubuntu, select the ubuntu.xorg from the setting button on the left-button corner. 
"""
import mujoco
import mujoco.viewer
import numpy as np
import time

# Load the kuka robot
model = mujoco.MjModel.from_xml_path("robot/robot_arms/kuka_iiwa_14/scene.xml")
data = mujoco.MjData(model)

# Target_id
mocap_id = model.body("target").mocapid[0]
# End-effector site we wish to control, in this case a site attached to the last
# link (wrist_3_link) of the robot.
site_id = model.site("attachment_site").id

# Pre-allocate numpy arrays.
M_inv = np.zeros((model.nv, model.nv))
Mx = np.zeros((6, 6))
jac = np.zeros((6, model.nv))
error = np.zeros(6)
error_pos = error[:3]
error_ori = error[3:]
site_quat = np.zeros(4)
target_quat_conj = np.zeros(4)
error_quat = np.zeros(4)

# Construct Kp
# Cartesian impedance control gains.
impedance_pos = np.asarray([100.0, 100.0, 100.0])  # [N/m]
impedance_ori = np.asarray([50.0, 50.0, 50.0])  # [Nm/rad]
Kp = np.concatenate([impedance_pos, impedance_ori], axis=0)
# Construct Kd through damping ratio.
damping_ratio = 1.0
damping_pos = damping_ratio * 2 * np.sqrt(impedance_pos)
damping_ori = damping_ratio * 2 * np.sqrt(impedance_ori)
Kd = np.concatenate([damping_pos, damping_ori], axis=0)

# Define null space preference
q0 = model.key(0).qpos
Kp_null = np.asarray([75.0, 75.0, 50.0, 50.0, 40.0, 25.0, 25.0])
Kd_null = damping_ratio * 2 * np.sqrt(Kp_null)

with mujoco.viewer.launch_passive(model=model,data=data) as viewer:

    # Reset the simulation.
    mujoco.mj_resetDataKeyframe(model, data, 0)
    # Reset the free camera.
    mujoco.mjv_defaultFreeCamera(model, viewer.cam)
    # Enable site frame visualization.
    viewer.opt.frame = mujoco.mjtFrame.mjFRAME_SITE
    viewer.opt.sitegroup[4] = 1
    viewer.opt.geomgroup[4] = 1

    while viewer.is_running():
        step_start = time.time()
        
        # Calculate the error, please refer to robot_arm_mocap for more details
        error_pos[:] = data.site(site_id).xpos - data.mocap_pos[mocap_id]
        target_ori = data.mocap_quat[mocap_id]
        mujoco.mju_negQuat(target_quat_conj, target_ori)
        mujoco.mju_mat2Quat(site_quat, data.site(site_id).xmat)
        mujoco.mju_mulQuat(error_quat, site_quat, target_quat_conj)
        mujoco.mju_quat2Vel(error_ori, error_quat, 1.0)
        mujoco.mj_jacSite(model, data, jac[:3], jac[3:], site_id)

        # Compute the task-space inertia matrix.
        # The below function solve: Mx = y, where x is the return stored in "M_inv" and y is "np.eye(model.nv)", here we explicity solve for M^{-1}
        mujoco.mj_solveM(model, data, M_inv, np.eye(model.nv))
        Mx_inv = jac @ M_inv @ jac.T
        Mx = np.linalg.pinv(Mx_inv)

        # Compute generalized forces.
        tau = jac.T @ Mx @ (-Kp * error - Kd * (jac @ data.qvel)) + data.qfrc_bias

        # Add null space preference
        Jbar = M_inv @ jac.T @ Mx
        ddq = -Kp_null * (data.qpos-q0) - Kd_null * data.qvel
        tau += (np.eye(model.nv) - jac.T @ Jbar.T) @ ddq

        # Set the control signal and step the simulation.
        np.clip(tau, *model.actuator_ctrlrange.T, out=tau)
        data.ctrl = tau
        mujoco.mj_step(model, data)

        viewer.sync()
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)


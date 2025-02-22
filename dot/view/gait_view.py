from typing import Any
from matplotlib import animation
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
import numpy as np

from dot.control.gait import Gait
from dot.control.inverse_kinematics import RobotIK
from dot.sim.quadruped import Quadruped
from dot.view.control_gui import ControlGui
import time

def update_leg_line_plots(model_gait: Gait, leg_line_plots: list[list[Any]]):
    dt = 0.02
    foot_poses = []
    stride_time = model_gait.stride_time()
    model_gait._prev_foot_pose = model_gait.foot_rest_pose.copy()
    model_gait._time = 0
    t = 0

    while t < stride_time:
        foot_poses.append(model_gait.compute_foot_positions(dt))
        t += dt

    for i, plot in enumerate(leg_line_plots):
        points = np.array([f[i] for f in foot_poses] + [foot_poses[0][i]])
        plot.set_data_3d(points.T)

    all_points = np.concatenate(foot_poses)
    return np.min(all_points, axis=0), np.max(all_points, axis=0)

def main():
    model = Quadruped()

    model_ik = RobotIK(
        model.body_length,
        model.body_width,
        model.max_height * 0.7,
        model.hip_offset,
        model.arm_length,
        model.wrist_length,
    )
    
    model_gait = Gait(model_ik.foot_points)
    model_gait_curves = Gait(model_ik.foot_points)

    gui = ControlGui(model_ik, model_gait)
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    leg_line_plots = [ax.plot([], [], color=c)[0] for c in ["r", "g", "b", "r"]]
    foot_point_plot = ax.plot([], [], marker='.', linestyle="None")[0]
    lim_min, lim_max = update_leg_line_plots(model_gait, leg_line_plots)
    ax.set_xlim(lim_min[0], lim_max[0])
    ax.set_ylim(lim_min[1], lim_max[1])
    ax.set_zlim(lim_min[2], lim_max[2])
    
    fps = 5
    dt = 0.02
    def update(frame_num):
        gui.update_model(model_ik, model_gait)
        gui.update_model(model_ik, model_gait_curves)
        lim_min, lim_max = update_leg_line_plots(model_gait_curves, leg_line_plots)
        ax.set_xlim(lim_min[0], lim_max[0])
        ax.set_ylim(lim_min[1], lim_max[1])
        ax.set_zlim(lim_min[2], lim_max[2])
        foot_points = model_gait.compute_foot_positions(dt)
        foot_point_plot.set_data_3d(foot_points.T)
    
    anim = animation.FuncAnimation(fig=fig, func=update, interval = 1000 / fps, cache_frame_data=False)

    gui.launch()
    plt.show()
        
        




if __name__ == "__main__":
    main()

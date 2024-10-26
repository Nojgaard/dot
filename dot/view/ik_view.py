from mpl_toolkits.mplot3d import Axes3D
from matplotlib import animation
import matplotlib.pyplot as plt
import numpy as np

from dot.control.inverse_kinematics import QuadropedIK
from dot.sim.quadruped import Quadruped
from dot.view.control_gui import ControlGui


def main():
    gui = ControlGui()

    spot = Quadruped()
    ik_model = QuadropedIK(
        spot.body_length,
        spot.body_width,
        spot.max_height * 0.7,
        spot.hip_offset,
        spot.shoulder_length,
        spot.wrist_length
    )

    fig = plt.figure()
    
    ax = fig.add_subplot(projection='3d')
    def update(frame_num):
        ax.clear()

        ax.set_xlim(-0.15, 0.15)
        ax.set_ylim(-0.15, 0.15)

        translation = gui.ctrl_translation
        rotation = gui.crtl_rotation

        foot_points = rotation.apply(ik_model.foot_points) + translation
        #foot_points = ik_model.foot_points
        body_points = ik_model.body_points
        #body_points = rotation.apply(ik_model.body_points) + translation
        pts = np.concatenate([body_points, foot_points])
        xs, ys, zs = pts[:, 0], pts[:, 1], pts[:, 2]

        for pt1, pt2 in zip(foot_points, body_points):
            ax.plot([pt1[0], pt1[0], pt2[0]], [pt1[1], pt2[1], pt2[1]], [pt1[2], pt1[2], pt2[2]], color='b')
        return ax.scatter(xs, ys, zs)

    anim = animation.FuncAnimation(fig=fig, func=update, interval = 30, cache_frame_data=False) 
    gui.launch()
    plt.show()


if __name__ == "__main__":
    main()

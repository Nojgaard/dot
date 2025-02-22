import asyncio
import time

import dearpygui.dearpygui as dpg
import dm_control.mjcf as mjcf
import mujoco
import mujoco.viewer as mjv
from dm_control.composer.arena import Arena

from dot.control.inverse_kinematics import RobotIK
from dot.real.comm import SensorReadings
from dot.real.robot_controller import RobotController
from dot.sim.quadruped import Quadruped
import numpy as np


class CalibrateBiasGui:
    def __init__(self, controller: RobotController, physics: mjcf.Physics, joints):
        self._physics = physics
        self._joints = joints
        self._controller = controller
        self.joint_ranges = np.array([j.range for j in self._joints])
        
        specs = Quadruped()
        print(specs.body_width, specs.body_length)
        # shoulder = 11.5
        # wrist = 13.5
        # width = 0.072
        # length = 0.186
        self.robot_ik = RobotIK(
            specs.body_length,
            specs.body_width,
            specs.max_height * 0.7,
            specs.hip_offset,
            specs.arm_length,
            specs.wrist_length,
            translation=np.array([-0.035, 0, 0]),
        )

    @property
    def joint_angles(self):
        return self._physics.bind(self._joints).qpos.copy()
    
    def set_mode(self):
        mode_val = dpg.get_value(self.mode_dropdown)
        joints = self._physics.bind(self._joints)
        if mode_val == "Straight":
            joints.qpos[:] = 0
            self._physics.data.ctrl[:] = 0
        elif mode_val == "Bend":
            angles = self.robot_ik.find_angles().flatten()
            joints.qpos[:] = angles
            self._physics.data.ctrl[:] = angles
            

    def launch(self):
        dpg.create_context()
        dpg.create_viewport(title="Control Inputs", width=1200, height=600)

        with dpg.window(tag="Primary Window"):
            self._joint_angle_sliders = []
            self._joint_invert_checkboxes = []
            calibration = self._controller.servo_driver.calibration
            for i, joint in enumerate(self._joints):
                name: str = joint.name

                self._joint_angle_sliders.append(
                    dpg.add_slider_float(
                        label=name,
                        default_value=calibration.servo_angle_bias[i],
                        min_value=-10,
                        max_value=10,
                    )
                )
                self._joint_invert_checkboxes.append(
                    dpg.add_checkbox(
                        label="invert", default_value=calibration.invert_servo_angle[i]
                    )
                )

            dpg.add_text("Mode")
            self.mode_dropdown = dpg.add_combo(["Straight", "Bend"], default_value="Straight")
            dpg.add_button(label="Set Angles", callback=self.set_mode)
            dpg.add_text("Connect")
            self.connect_checkbox = dpg.add_checkbox(default_value=False)

            self._voltage_text = dpg.add_text("Voltage: None")

        dpg.setup_dearpygui()
        dpg.show_viewport()
        dpg.set_primary_window("Primary Window", True)
        dpg.start_dearpygui()
        dpg.destroy_context()

    def update_controller(self, sensor_readings: SensorReadings):
        dpg.set_value(
            self._voltage_text, f"Voltage: {sensor_readings.battery_voltage:.2f}"
        )

        send_commands = dpg.get_value(self.connect_checkbox)
        if not send_commands:
            return

        joint_angles = self.joint_angles
        joint_ranges = self.joint_ranges

        calibration = self._controller.servo_driver.calibration
        calibration.invert_servo_angle = tuple(
            dpg.get_value(x) for x in self._joint_invert_checkboxes
        )

        calibration.servo_angle_bias = tuple(
            dpg.get_value(x) for x in self._joint_angle_sliders
        )

        servo_angles = self._controller.servo_driver.joint_to_servo_angle(
            joint_angles, joint_ranges
        )

        self._controller.servo_driver.set_servo_angles(servo_angles)


async def launch_viewer(physics):
    with mjv.launch_passive(physics.model.ptr, physics.data.ptr) as viewer:
        while viewer.is_running():
            step_start = time.time()

            mujoco.mj_step(physics.model.ptr, physics.data.ptr)

            # Pick up changes to the physics state, apply perturbations, update options from GUI.
            viewer.sync()

            # Rudimentary time keeping, will drift relative to wall clock.
            time_until_next_step = physics.model.opt.timestep - (
                time.time() - step_start
            )
            if time_until_next_step > 0:
                await asyncio.sleep(time_until_next_step)


async def main():
    spot = Quadruped()
    arena = Arena()
    arena.add_free_entity(spot)
    arena.mjcf_model.option.gravity = "0 0 0"
    physics = mjcf.Physics.from_mjcf_model(arena.mjcf_model)
    joints = spot.mjcf_model.find_all("joint")

    controller = await RobotController.create()
    controller.servo_driver.load_calibration("data/calibration.json")
    print(controller.servo_driver.calibration.bounds_pwm_ms)
    gui = CalibrateBiasGui(controller, physics, joints)
    gui_task = asyncio.create_task(asyncio.to_thread(gui.launch))
    await asyncio.sleep(0.2)

    model_viewer_task = asyncio.create_task(launch_viewer(physics))

    controller_task = asyncio.create_task(
        controller.launch(
            RobotController.Mode.Calibrate, callback=gui.update_controller, fps=20
        )
    )

    done, pending = await asyncio.wait(
        [model_viewer_task, gui_task, controller_task],
        return_when=asyncio.FIRST_COMPLETED,
    )

    print("IM DONE")
    for task in done:
        if task.result() is not None:
            print(f"{task.result()}")

    for task in pending:
        task.cancel()


asyncio.run(main())

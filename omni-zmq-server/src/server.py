# SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: MIT

import dearpygui.dearpygui as dpg

from zmq_handler import ZMQManager
from vision import Vision


class ZMQServerWindow:
    def __init__(self):

        self.init_vision()
        self.init_gui()
        self.init_connections()

    def init_gui(self):
        dpg.create_context()
        dpg.create_viewport(
            title="Omni ZMQ Server", width=720, height=800, resizable=False
        )
        dpg.setup_dearpygui()

        self.font_scale = 20
        with dpg.font_registry():
            font_medium = dpg.add_font("Inter-Medium.ttf", 16 * self.font_scale)

        dpg.set_global_font_scale(1 / self.font_scale)
        dpg.bind_font(font_medium)

        with dpg.texture_registry(show=False):
            dpg.add_raw_texture(
                self.vision.dimmention,
                self.vision.dimmention,
                self.vision.texture_data,
                tag="image_stream",
                format=dpg.mvFormat_Float_rgba,
            )

        with dpg.window(tag="Main Window"):
            dpg.add_image("image_stream", tag="t")

            times = ["sim_dt", "local_dt", "local_hz"]
            for index, t in enumerate(times):
                with dpg.value_registry():
                    dpg.add_string_value(tag=t, default_value="0.0")
                dpg.add_text(
                    " ".join(t.split("_")).capitalize(), pos=(10, 10 + (20 * index))
                )

                dpg.add_text(source=t, label=t, pos=(80, 10 + (20 * index)))

            dpg.add_separator()

            with dpg.group():
                dpg.add_text("Control Camera with arrows")
                with dpg.group(horizontal=True):
                    dpg.add_text("Ground Truth")
                    dpg.add_combo(
                        items=["RGB", "BBOX2D", "DEPTH"],
                        default_value="BBOX2D",
                        width=100,
                        tag="ground_truth_mode",
                    )
                    dpg.add_text("Focal Length")
                    dpg.add_slider_float(
                        tag="zoom",
                        default_value=20,
                        min_value=self.vision.camera_range[0],
                        max_value=self.vision.camera_range[1],
                        width=200,
                    )
                    dpg.add_text("Draw Detection on World")
                    dpg.add_checkbox(default_value=True, tag="draw_detection_on_world")

        with dpg.handler_registry():
            dpg.add_key_down_handler(callback=self.key_press_evnet)
            dpg.add_key_release_handler(callback=self.key_depress_evnet)
            dpg.add_mouse_wheel_handler(callback=self.mouse_wheel_evnet)

        dpg.show_viewport()
        dpg.set_primary_window("Main Window", True)

    def init_vision(self):
        self.vision = Vision()
        self.vision.gpu_preallocate()

    def init_connections(self):
        self.ports = {
            "camera_annotator": 5555,
            "camera_link": 5557,
            "focal_length": 5558,
        }
        self.zmq_manager = ZMQManager()
        self.zmq_manager.recive_from_socket_in_loop(
            "camera_annotator",
            self.ports["camera_annotator"],
            self.vision.receive_images,
        )

        self.zmq_manager.send_from_socket_in_loop(
            "focal_length_commands",
            self.ports["focal_length"],
            self.vision.hz,
            self.vision.focal_lengh_command,
        )

        self.zmq_manager.send_from_socket_in_loop(
            "camera_link_commands",
            self.ports["camera_link"],
            self.vision.hz,
            self.vision.camera_link_command,
        )

    def mouse_wheel_evnet(self, sender, app_data):
        new_value = dpg.get_value("zoom") + (app_data * 5)
        new_value = max(min(new_value, 200), 20)
        dpg.set_value("zoom", new_value)

    def key_press_evnet(self, sender, app_data):
        if dpg.is_key_down(dpg.mvKey_Up):
            self.vision.current_camera_command = [0, 1]
        elif dpg.is_key_down(dpg.mvKey_Down):
            self.vision.current_camera_command = [0, -1]
        elif dpg.is_key_down(dpg.mvKey_Left):
            self.vision.current_camera_command = [1, 0]
        elif dpg.is_key_down(dpg.mvKey_Right):
            self.vision.current_camera_command = [-1, 0]

    def key_depress_evnet(self, sender, app_data):
        self.vision.current_camera_command = [0, 0]

    def run(self):
        while dpg.is_dearpygui_running():
            dpg.render_dearpygui_frame()

    def cleanup(self):
        self.zmq_manager.cleanup()
        dpg.destroy_context()


window = ZMQServerWindow()
window.run()
window.cleanup()

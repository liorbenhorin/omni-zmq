# Copyright (c) 2022-2023, NVIDIA Corporation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


import dearpygui.dearpygui as dpg
import numpy as np
import cv2
import json
import struct
import time
import math
import traceback

from zmq_handler import ZMQManager


class ZMQServerWindow:
    def __init__(self):
        self.ports = {
            "camera_annotator": 5555,
            "camera_link": 5557,
            "focal_length": 5558,
        }
        self.dimmention = 720
        self.expected_size = self.dimmention * self.dimmention * 4
        self.hz = 60
        self.last_time = time.time()
        self.internal_step_time = time.time()
        self.camera_range = [20, 200]
        self.current_camera_f = 20
        self.texture_data = np.zeros(
            (self.dimmention, self.dimmention, 4), dtype=np.float32
        )

        dpg.create_context()
        dpg.create_viewport(
            title="Omni ZMQ Server", width=720, height=840, resizable=True
        )
        dpg.setup_dearpygui()

        self.font_scale = 20
        with dpg.font_registry():
            font_medium = dpg.add_font("Inter-Medium.ttf", 16 * self.font_scale)

        dpg.set_global_font_scale(1 / self.font_scale)
        dpg.bind_font(font_medium)

        with dpg.texture_registry(show=False):
            dpg.add_raw_texture(
                self.dimmention,
                self.dimmention,
                self.texture_data,
                tag="image_stream",
                format=dpg.mvFormat_Float_rgba,
            )

        with dpg.window(tag="Main Window"):
            dpg.add_image("image_stream")
            dpg.add_separator()

            with dpg.group():
                times = ["sim_dt", "local_dt", "local_hz"]
                for t in times:
                    with dpg.value_registry():
                        dpg.add_string_value(tag=t, default_value="0.0")

                with dpg.group(horizontal=True):
                    for t in times:
                        dpg.add_text(" ".join(t.split("_")).capitalize())
                        dpg.add_spacer(width=10)
                        dpg.add_text(source=t, label=t)
                        dpg.add_spacer(width=30)

                dpg.add_separator()
                dpg.add_text("Control Camera with arrows")
                with dpg.group(horizontal=True):
                    dpg.add_text("Ground Truth")
                    dpg.add_combo(
                        items=["RGB", "BBOX2D"], 
                        default_value="RGB", 
                        width=100,
                        tag="ground_truth_mode"
                    )
                    dpg.add_text("Focal Length")
                    dpg.add_slider_float(
                        tag="zoom",
                        default_value=20,
                        min_value=self.camera_range[0],
                        max_value=self.camera_range[1],
                        width=200,
                    )

        with dpg.handler_registry():
            dpg.add_key_down_handler(callback=self.key_press)
            dpg.add_key_release_handler(callback=self.key_depress)
            dpg.add_mouse_wheel_handler(callback=self.mouse_wheel)

        dpg.show_viewport()
        dpg.set_primary_window("Main Window", True)

        self.current_camera_command = [0, 0]
        self.init_connections()

    def init_connections(self):
        self.zmq_manager = ZMQManager()
        self.zmq_manager.recive_from_socket_in_loop(
            "camera_annotator", self.ports["camera_annotator"], self.receive_images
        )

        self.zmq_manager.send_from_socket_in_loop(
            "focal_length_commands",
            self.ports["focal_length"],
            self.hz,
            self.focal_lengh_command,
        )

        self.zmq_manager.send_from_socket_in_loop(
            "camera_link_commands",
            self.ports["camera_link"],
            self.hz,
            self.camera_link_command,
        )

    def mouse_wheel(self, sender, app_data):
        new_value = dpg.get_value("zoom") + (app_data * 5)
        new_value = max(min(new_value, 200), 20)
        dpg.set_value("zoom", new_value)

    def key_press(self, sender, app_data):
        if dpg.is_key_down(dpg.mvKey_Up):
            self.current_camera_command = [0, -1]
        elif dpg.is_key_down(dpg.mvKey_Down):
            self.current_camera_command = [0, 1]
        elif dpg.is_key_down(dpg.mvKey_Left):
            self.current_camera_command = [1, 0]
        elif dpg.is_key_down(dpg.mvKey_Right):
            self.current_camera_command = [-1, 0]

    def key_depress(self, sender, app_data):
        self.current_camera_command = [0, 0]

    def focal_lengh_command(self):
        def smooth_step(current, target, min_step=0.5, max_step=4, smoothness=0.1):
            diff = abs(target - current)
            # Use a sigmoid function for smooth interpolation
            factor = 1 / (1 + math.exp(-diff / smoothness))
            step = min_step + (max_step - min_step) * factor
            return step

        target_zoom = dpg.get_value("zoom")
        if self.current_camera_f != target_zoom:
            step = smooth_step(self.current_camera_f, target_zoom)
            if self.current_camera_f < target_zoom:
                self.current_camera_f = min(self.current_camera_f + step, target_zoom)
            else:
                self.current_camera_f = max(self.current_camera_f - step, target_zoom)

        return {"focal_length": self.current_camera_f}

    def camera_link_command(self):
        factor = np.interp(dpg.get_value("zoom"), self.camera_range, [0.3, 1.2])
        command_x = self.current_camera_command[0] * factor
        command_y = self.current_camera_command[1] * factor
        data_to_send = {"camera_link": [command_x, command_y]}
        return data_to_send

    def receive_images(self, message):
        img_data = message[0]
        bbox2d_data = json.loads(message[1].decode("utf-8"))
        dt = struct.unpack("f", message[2])[0]

        if len(img_data) != self.expected_size:
            print(
                f"Received image data of size {len(img_data)}, expected {self.expected_size}"
            )
            return

        img_array = np.frombuffer(img_data, dtype=np.uint8).reshape(
            self.dimmention, self.dimmention, 4
        )

        if dpg.get_value("ground_truth_mode") == "BBOX2D":
            img_array = self.draw_bounding_boxes(img_array, bbox2d_data)

        np.divide(img_array, 255.0, out=self.texture_data)

        local_dt = time.time() - self.last_time
        self.last_time = time.time()

        dpg.set_value("image_stream", self.texture_data)
        dpg.set_value("sim_dt", str("{:.2f}".format(dt)))
        dpg.set_value("local_dt", str("{:.2f}".format(local_dt)))
        dpg.set_value("local_hz", str("{:.2f}".format(1.0 / local_dt)))

    def draw_bounding_boxes(self, img_array, bbox_data):
        img_with_boxes = img_array.copy()
        bboxes = bbox_data["data"]
        id_to_labels = bbox_data["info"]["idToLabels"]
        color = (0, 0, 0)
        font = cv2.FONT_HERSHEY_SIMPLEX

        for bbox in bboxes:
            semantic_id = bbox[0]
            x_min, y_min = bbox[1], bbox[2]
            x_max, y_max = bbox[3], bbox[4]
            label = id_to_labels.get(str(semantic_id), {}).get("class", "Unknown")

            cv2.rectangle(img_with_boxes, (x_min, y_min), (x_max, y_max), color, 2)
            cv2.putText(img_with_boxes, label, (x_min, y_min - 10), font, 0.9, color, 2)

        return img_with_boxes

    def run(self):
        while dpg.is_dearpygui_running():
            dpg.render_dearpygui_frame()

    def cleanup(self):
        self.zmq_manager.cleanup()
        dpg.destroy_context()


window = ZMQServerWindow()
window.run()
window.cleanup()

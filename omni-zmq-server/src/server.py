import sys
import dearpygui.dearpygui as dpg
import numpy as np
import cv2
import threading
import struct
import time
import traceback


from zmq_handler import ZMQManager


class VisionWindow:
    def __init__(self, predict=True, tracking=True):
        self.ports = {
            "rgb": 5555,
            "camera_link": 5557,
            "focal_length": 5558
        }
        self.dimmention = 720
        self.expected_size = self.dimmention * self.dimmention * 4
        self.hz = 60
        self.last_time = time.time()
        self.internal_step_time = time.time()
        self.camera_range = [20, 200]

        dpg.create_context()
        dpg.create_viewport(title='Omni ZMQ Server', width=720, height=840, resizable=True)
        dpg.setup_dearpygui()

        self.font_scale = 20        
        with dpg.font_registry():
            font_medium = dpg.add_font('Inter-Medium.ttf', 16*self.font_scale)

        dpg.set_global_font_scale(1/self.font_scale)
        dpg.bind_font(font_medium)


        with dpg.texture_registry(show=False):
            dummy_data = np.zeros((self.dimmention, self.dimmention, 4), dtype=np.float32).ravel()
            dpg.add_raw_texture(self.dimmention, self.dimmention, dummy_data, tag="image_stream", format=dpg.mvFormat_Float_rgba)

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
                        dpg.add_text(" ".join(t.split('_')).capitalize())
                        dpg.add_spacer(width=10)
                        dpg.add_text(source=t, label=t)
                        dpg.add_spacer(width=30)

                dpg.add_separator()
                dpg.add_text("Control Camera with arrows")
                with dpg.group(horizontal=True):
                    dpg.add_text("Focal Length")
                    dpg.add_slider_float(tag="zoom",  
                                         default_value=20,  
                                         min_value=self.camera_range[0],  
                                         max_value=self.camera_range[1], 
                                         width=200)

        with dpg.handler_registry():
            dpg.add_key_down_handler(callback=self.key_press)
            dpg.add_key_release_handler(callback=self.key_depress)

        dpg.show_viewport()
        dpg.set_primary_window("Main Window", True)

        self.current_camera_command = [0,0]
        self.init_connections()
     
    def init_connections(self):
        self.zmq_manager = ZMQManager()
        self.zmq_manager.recive_from_socket_in_loop("rgb", 
                                            self.ports["rgb"], 
                                            self.receive_images)

        self.zmq_manager.send_from_socket_in_loop("focal_length_commands", 
                                            self.ports["focal_length"], 
                                            self.hz,
                                            self.focal_lengh_command)

        self.zmq_manager.send_from_socket_in_loop("camera_link_commands", 
                                            self.ports["camera_link"], 
                                            self.hz,
                                            self.camera_link_command)

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
        self.current_camera_command = [0,0]

    def focal_lengh_command(self):
        return {"focal_length": dpg.get_value("zoom")}

    def camera_link_command(self):
        factor = np.interp(dpg.get_value("zoom"), self.camera_range, [0.3, 1.2])    
        command_x = self.current_camera_command[0] * factor
        command_y = self.current_camera_command[1] * factor
        data_to_send = {"camera_link": [command_x, command_y]}
        return data_to_send

    def receive_images(self, message):
        img_data = message[0]
        dt = struct.unpack('f', message[1])[0]

        if len(img_data) != self.expected_size:
            print(f"Received image data of size {len(img_data)}, expected {self.expected_size}")
            return

        img_array = np.frombuffer(img_data, dtype=np.uint8).reshape(self.dimmention, self.dimmention, 4)
        
        data = img_array.ravel()
        data = np.asfarray(data, dtype='f')
        texture_data = np.true_divide(data, 255.0)

        local_dt = time.time() - self.last_time
        self.last_time = time.time()

        dpg.set_value("image_stream", texture_data)
        dpg.set_value("sim_dt", str("{:.2f}".format(dt)))
        dpg.set_value("local_dt", str("{:.2f}".format(local_dt)))
        dpg.set_value("local_hz", str("{:.2f}".format(1.0/local_dt)))

    def run(self):
        while dpg.is_dearpygui_running():
            dpg.render_dearpygui_frame()

    def cleanup(self):
        self.zmq_manager.cleanup()
        dpg.destroy_context()


window = VisionWindow()
window.run()
window.cleanup()
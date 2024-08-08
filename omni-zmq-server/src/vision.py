# SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: MIT

import numpy as np
import torch
import cv2
import json
import struct
import time
import math
import traceback
import time

import dearpygui.dearpygui as dpg

np.set_printoptions(precision=4, suppress=True)


class Vision:
    def __init__(self):
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
        self.depth_data = np.zeros(
            (self.dimmention, self.dimmention, 4), dtype=np.uint8
        )
        self.detection_world_pos = [0, 0, 0]
        self.detection_camera_pos = [0, 0]
        self.current_camera_command = [0, 0]

    def gpu_preallocate(self):
        # Pre-allocate memory on GPU for depth data
        self._depth_data_gpu = torch.zeros(
            (self.dimmention, self.dimmention), device="cuda", dtype=torch.float32
        )
        # Prepare placeholders for matrices on GPU
        self.view_matrix_ros_gpu = None
        self.intrinsics_matrix_gpu = None
        self.inverse_intrinsics_gpu = None

    def update_camera_matrices(self, view_matrix_ros, intrinsics_matrix):
        # Update view matrix on GPU
        if self.view_matrix_ros_gpu is None:
            self.view_matrix_ros_gpu = torch.tensor(
                view_matrix_ros, device="cuda", dtype=torch.float32
            )
        else:
            self.view_matrix_ros_gpu.copy_(
                torch.tensor(view_matrix_ros, device="cuda", dtype=torch.float32)
            )

        # Update intrinsics matrix on GPU and calculate its inverse
        if self.intrinsics_matrix_gpu is None:
            self.intrinsics_matrix_gpu = torch.tensor(
                intrinsics_matrix, device="cuda", dtype=torch.float32
            )
        else:
            self.intrinsics_matrix_gpu.copy_(
                torch.tensor(intrinsics_matrix, device="cuda", dtype=torch.float32)
            )

        # Compute the inverse of the intrinsics matrix on GPU
        try:
            self.inverse_intrinsics_gpu = torch.inverse(self.intrinsics_matrix_gpu)
        except RuntimeError as e:
            print(f"Error computing inverse intrinsics matrix: {e}")
            self.inverse_intrinsics_gpu = None

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
        data_to_send = {
            "camera_link": [command_x, command_y],
            "detection_pos": self.detection_world_pos,
        }
        return data_to_send

    def receive_images(self, message):
        start_time = time.perf_counter()
        img_data = message[0]
        bbox2d_data = json.loads(message[1].decode("utf-8"))
        depth_data = message[2]
        camera_data = json.loads(message[3].decode("utf-8"))
        dt = struct.unpack("f", message[4])[0]

        if len(img_data) != self.expected_size:
            print(
                f"Received image data of size {len(img_data)}, expected {self.expected_size}"
            )
            return

        if dpg.get_value("ground_truth_mode") in ["BBOX2D", "RGB"]:
            img_array = np.frombuffer(img_data, dtype=np.uint8).reshape(
                self.dimmention, self.dimmention, 4
            )
            if dpg.get_value("ground_truth_mode") == "BBOX2D":
                img_array = self.draw_bounding_boxes(img_array, bbox2d_data)
        elif dpg.get_value("ground_truth_mode") == "DEPTH":
            img_array = np.frombuffer(depth_data, dtype=np.float32).reshape(
                self.dimmention, self.dimmention, 1
            )
            try:
                img_array = self.colorize_depth(img_array)
            except:
                print(traceback.format_exc())

        np.divide(img_array, 255.0, out=self.texture_data)

        if dpg.get_value("draw_detection_on_world"):
            self.get_bbox_center_in_world_coords(
                bbox2d_data, depth_data, camera_data, device="cuda"
            )
        else:
            self.detection_world_pos = [0, 0, 0]

        local_dt = time.time() - self.last_time
        self.last_time = time.time()

        dpg.set_value("image_stream", self.texture_data)
        dpg.set_value("sim_dt", str("{:.2f}".format(dt)))
        dpg.set_value("local_dt", str("{:.2f}".format(local_dt)))
        dpg.set_value("local_hz", str("{:.2f}".format(1.0 / local_dt)))

        # print(f'recived image in {(time.perf_counter() - start_time)*10000}')

    def get_bbox_center_in_world_coords(
        self,
        bbox_data: dict,
        depth_data: bytes,
        camera_data: dict,
        device: str = "cuda",
    ) -> None:
        if bbox_data["data"]:
            for bbox in bbox_data["data"]:
                u = int((bbox[1] + bbox[3]) / 2)
                v = int((bbox[2] + bbox[4]) / 2)
                break  # only handle a single detection!
        else:
            self.detection_world_pos = [0, 0, 0]
            return

        self.detection_camera_pos = [u, v]
        point = [u, v]

        depth_array = np.frombuffer(depth_data, dtype=np.float32).reshape(
            self.dimmention, self.dimmention
        )

        # Simplifled implementation of
        # omni.sensor.Camera.get_world_points_from_image_coords()
        # https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.sensor/docs/index.html#omni.isaac.sensor.scripts.camera.Camera.get_world_points_from_image_coords

        if device == "cuda":
            view_matrix_ros = np.array(camera_data["view_matrix_ros"])
            intrinsics_matrix = np.array(camera_data["intrinsics_matrix"])
            self.update_camera_matrices(view_matrix_ros, intrinsics_matrix)
            self.get_bbox_center_in_world_coords_gpu(depth_array, point)
        else:
            self.get_bbox_center_in_world_coords_cpu(depth_array, camera_data, point)

    def get_bbox_center_in_world_coords_gpu(
        self, depth_array: np.ndarray, point: tuple
    ) -> None:
        """
        Calculate the 3D position of a bounding box in the world coordinates based on its location in the image and depth map.

        Args:
            depth_data (np.ndarray): Depth map array
            point (tuple): Image coordinates (u, v) of the bounding box center

        Returns:
            None
        """

        u, v = point
        # Copy the depth array to the GPU
        depth_array = np.copy(depth_array)
        self._depth_data_gpu.copy_(torch.from_numpy(depth_array).cuda())

        # Get the depth value at the specified point (u, v)
        depth_value = (self._depth_data_gpu[v, u] * 10) - 2

        # Create a homogeneous point (u, v, 1.0) as a tensor on the GPU
        homogenous_point = torch.tensor([u, v, 1.0], device="cuda", dtype=torch.float32)

        # Calculate the point's camera coordinates by multiplying the homogeneous point by the inverse intrinsics matrix and the depth value
        point_camera_coords = torch.mv(
            self.inverse_intrinsics_gpu, homogenous_point * depth_value
        )

        # Add a 1.0 to the point's camera coordinates to make it homogeneous
        point_camera_coords_homogenous = torch.cat(
            [
                point_camera_coords,
                torch.tensor([1.0], device="cuda", dtype=torch.float32),
            ]
        )

        # Invert the view matrix on the GPU
        inverse_view_matrix_gpu = torch.inverse(self.view_matrix_ros_gpu)

        # Calculate the point's world coordinates by multiplying the homogeneous point by the inverse view matrix
        point_world_coords_homogenous = torch.mv(
            inverse_view_matrix_gpu, point_camera_coords_homogenous
        )

        self.detection_world_pos = (
            point_world_coords_homogenous[:3].cpu().numpy().tolist()
        )

    def get_bbox_center_in_world_coords_cpu(
        self, depth_array: np.ndarray, camera_data: dict, point: tuple
    ) -> None:
        """
        Calculate the 3D position of a bounding box in the world coordinates based on its location in the image and depth map.

        Args:
            depth_data (np.ndarray): Depth map array
            camera_data (dict): Camera data containing intrinsics and view matrix
            point (tuple): Image coordinates (u, v) of the bounding box center

        Returns:
            None
        """

        u, v = point
        depth_value = (depth_array[v, u] * 10) - 2

        view_matrix_ros = np.array(camera_data["view_matrix_ros"])
        intrinsics_matrix = np.array(camera_data["intrinsics_matrix"])

        # Create a homogeneous point (u, v, 1.0) as a numpy array
        homogenous_point = np.array([u, v, 1.0])

        # Invert the intrinsics matrix
        inverse_intrinsics = np.linalg.inv(intrinsics_matrix)

        # Calculate the point's camera coordinates by multiplying the homogeneous point by the inverse intrinsics matrix and the depth value
        point_camera_coords = inverse_intrinsics @ (homogenous_point * depth_value)

        # Add a 1.0 to the point's camera coordinates to make it homogeneous
        point_camera_coords_homogenous = np.append(point_camera_coords, 1.0)

        # Invert the view matrix
        inverse_view_matrix = np.linalg.inv(view_matrix_ros)

        # Calculate the point's world coordinates by multiplying the homogeneous point by the inverse view matrix
        point_world_coords_homogenous = (
            inverse_view_matrix @ point_camera_coords_homogenous
        )

        point_world_coords = point_world_coords_homogenous[:3].tolist()

        self.detection_world_pos = point_world_coords

    def draw_bounding_boxes(self, img_array: np.ndarray, bbox_data: dict) -> None:
        img_with_boxes = img_array.copy()
        bboxes = bbox_data["data"]
        id_to_labels = bbox_data["info"]["idToLabels"]
        color = (118, 185, 0)
        font = cv2.FONT_HERSHEY_SIMPLEX

        for bbox in bboxes:
            semantic_id = bbox[0]
            x_min, y_min = bbox[1], bbox[2]
            x_max, y_max = bbox[3], bbox[4]

            u = int((bbox[1] + bbox[3]) / 2)
            v = int((bbox[2] + bbox[4]) / 2)

            label = id_to_labels.get(str(semantic_id), {}).get("class", "Unknown")
            cv2.circle(img_with_boxes, (u, v), 10, color, 2)
            cv2.rectangle(img_with_boxes, (x_min, y_min), (x_max, y_max), color, 2)
            cv2.putText(img_with_boxes, label, (x_min, y_min - 10), font, 0.9, color, 2)

        img_with_boxes[:, :, 3] = 255
        return img_with_boxes

    def colorize_depth(self, depth_data: np.ndarray) -> np.ndarray:
        # https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator/programmatic_visualization.html#helper-visualization-functions
        near = 1.0
        far = 50.0

        # if we want the image colorized to a color:
        # depth_data = np.squeeze(depth_data)

        depth_data = np.clip(depth_data, near, far)
        depth_data = (np.log(depth_data) - np.log(near)) / (np.log(far) - np.log(near))
        depth_data = 1.0 - depth_data
        depth_data_uint8 = (depth_data * 255).astype(np.uint8)

        # if we want the image colorized to a color:
        # self.depth_data[:, :, 0] = depth_data_uint8
        # self.depth_data[:, :, 3] = 255
        return depth_data_uint8

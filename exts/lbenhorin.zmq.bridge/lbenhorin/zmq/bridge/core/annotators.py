# SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: MIT

import json
import asyncio
import zmq
import zmq.asyncio
import struct

import omni.replicator.core as rep
from omni.replicator.core.scripts.utils import viewport_manager
from omni.isaac.core.prims import XFormPrim
from omni.isaac.sensor import Camera


class ZMQAnnotator:
    def __init__(
        self,
        socket: zmq.asyncio.Socket,
        camera: str,
        resolution: tuple,
        annotator: str,
    ):

        self.sock = socket

        force_new = False
        name = f"{camera.split('/')[-1]}_rp"
        self.camera_xform = XFormPrim(camera)

        rp = viewport_manager.get_render_product(camera, resolution, force_new, name)
        self.rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
        self.rgb_annot.attach(rp)

        self.bbox2d_annot = rep.AnnotatorRegistry.get_annotator("bounding_box_2d_tight")
        self.bbox2d_annot.attach(rp)

        self.distance_to_camera_annot = rep.AnnotatorRegistry.get_annotator(
            "distance_to_camera"
        )
        self.distance_to_camera_annot.attach(rp)

        self._camera = Camera(
            prim_path=camera,
            render_product_path=rp.path,
            resolution=resolution,
        )
        self._camera.initialize()

    def send(self, dt: float):
        _dt = struct.pack("f", dt)

        # https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator/annotators_details.html#bounding-box-2d-tight
        bbox2d_data = self.bbox2d_annot.get_data()
        _bbox2d_data = {
            "info": {
                "bboxIds": bbox2d_data["info"]["bboxIds"].tolist(),
                "idToLabels": bbox2d_data["info"]["idToLabels"],
                "primPaths": bbox2d_data["info"]["primPaths"],
            },
            "data": bbox2d_data["data"].tolist(),
        }

        _bbox2d_data = json.dumps(_bbox2d_data).encode("utf-8")

        _camera_params = {
            "view_matrix_ros": self._camera.get_view_matrix_ros().tolist(),
            "intrinsics_matrix": self._camera.get_intrinsics_matrix().tolist(),
        }

        _camera_params = json.dumps(_camera_params).encode("utf-8")

        data = [
            self.rgb_annot.get_data().tobytes(),
            _bbox2d_data,
            self.distance_to_camera_annot.get_data().tobytes(),
            _camera_params,
            _dt,
        ]
        asyncio.ensure_future(self.sock.send_multipart(data))

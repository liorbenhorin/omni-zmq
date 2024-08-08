# SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: MIT

import traceback
import json
import asyncio
import struct
import zmq
import zmq.asyncio
from functools import partial


import carb
import omni
import omni.replicator.core as rep
from omni.replicator.core.scripts.utils import viewport_manager

from omni.isaac.core.world import World
from omni.isaac.core.prims import XFormPrim
from omni.isaac.sensor import Camera

import numpy as np

np.set_printoptions(precision=3, suppress=True)


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

        # self.camera_annot = rep.AnnotatorRegistry.get_annotator("CameraParams")
        # self.camera_annot.attach(rp)

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

        # cp = self.camera_annot.get_data()
        _camera_params = {
            # "cameraAperture": cp["cameraAperture"].tolist(),
            # "cameraApertureOffset": cp["cameraApertureOffset"].tolist(),
            # "cameraFisheyeMaxFOV": cp["cameraFisheyeMaxFOV"],
            # "cameraFisheyeNominalHeight": cp["cameraFisheyeNominalHeight"],
            # "cameraFisheyeNominalWidth": cp["cameraFisheyeNominalWidth"],
            # "cameraFisheyeOpticalCentre": cp["cameraFisheyeOpticalCentre"].tolist(),
            # "cameraFisheyePolynomial": cp["cameraFisheyePolynomial"].tolist(),
            # "cameraFocalLength": cp["cameraFocalLength"],
            # "cameraFocusDistance": cp["cameraFocusDistance"],
            # "cameraFStop": cp["cameraFStop"],
            # "cameraModel": cp["cameraModel"],
            # "cameraNearFar": cp["cameraNearFar"].tolist(),
            # "cameraProjection": cp["cameraProjection"].tolist(),
            # "cameraViewTransform": cp["cameraViewTransform"].tolist(),
            # "metersPerSceneUnit": cp["metersPerSceneUnit"],
            # "renderProductResolution": cp["renderProductResolution"].tolist(),
            # "cameraWorldTransform": (
            #     self.camera_xform.get_world_pose()[0].tolist(),
            #     self.camera_xform.get_world_pose()[1].tolist(),
            # ),
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


class ZMQManager:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super().__new__(cls, *args, **kwargs)
        return cls._instance

    def __init__(self):
        self.push_sockets = {}
        self.pull_sockets = {}
        self.phyx_callbacks = {}
        self.annotators = {}
        self._context = None

    def context(self) -> zmq.asyncio.Context:
        """
        Returns the ZMQ context if it has not been initialized yet.
        Initializes and returns the ZMQ context if it has not been initialized.

        :return: The ZMQ context.

        """
        if not self._context:
            self._context = zmq.asyncio.Context()
        return self._context

    def get_annotator(self, port: int, camera: str, resolution: tuple, annotator: str):
        soket = self.get_push_socket(port)
        annot = ZMQAnnotator(soket, camera, resolution, annotator)
        self.annotators[annotator] = annot
        return annot

    def get_push_socket(self, port: int) -> zmq.Socket:
        """
        Creates and returns a ZeroMQ PUSH socket connected to the specified port.

        Args:
            port (int): The port number to connect the socket to.

        Returns:
            zmq.Socket: The created PUSH socket.
        """
        addr = f"tcp://localhost:{port}"
        sock = self.context().socket(zmq.PUSH)
        sock.set_hwm(1)
        sock.connect(addr)
        self.push_sockets[addr] = sock
        return sock

    def get_pull_socket(self, port: int) -> zmq.Socket:
        """
        Creates and returns a ZeroMQ PULL socket connected to the specified port.

        Parameters:
            port (int): The port number to connect the socket to.

        Returns:
            zmq.Socket: The created PULL socket.
        """
        addr = f"tcp://localhost:{port}"
        sock = self.context().socket(zmq.PULL)
        sock.set_hwm(1)
        sock.connect(addr)
        self.pull_sockets[addr] = sock
        return sock

    async def disconnect_all(self) -> None:
        """
        Disconnects all ZeroMQ sockets and terminates the ZeroMQ context.

        This method iterates over all push and pull sockets, disconnects them, and closes them.
        It then clears the socket dictionaries and terminates the ZeroMQ context.

        """

        for addr, sock in self.push_sockets.items():
            await asyncio.sleep(0.1)
            try:
                sock.disconnect(addr)
            except asyncio.CancelledError:
                pass
            except Exception:
                print(traceback.format_exc())

            sock.close()
        for addr, sock in self.pull_sockets.items():
            await asyncio.sleep(0.1)
            try:
                sock.disconnect(addr)
            except asyncio.CancelledError:
                pass
            except Exception:
                print(traceback.format_exc())

            sock.close()

        self.pull_sockets = {}
        self.push_sockets = {}

        self._context.term()
        self._context = None

    async def recive_data(self, sock: zmq.asyncio.Socket) -> object:
        """
        Asynchronously receives data from a ZeroMQ socket.

        Parameters:
            sock (zmq.asyncio.Socket): The ZeroMQ socket to receive data from.

        Returns:
            object: The received data as a Python object.
        """
        return await sock.recv_pyobj()

    def add_physx_step_callback(
        self, name: str, hz: float, fn: callable
    ) -> carb.Subscription:
        """
        Adds a callback function to be executed at every PhysX step.

        Parameters:
            name (str): The name of the callback.
            hz (float): The frequency at which the callback is executed.
            fn (callable): The callback function to be executed.

        Returns:
            object: The subscription object.
        """

        # ignore this snippet for now
        # update_stream = omni.kit.app.get_app().get_update_event_stream()
        # self.sub = update_stream.create_subscription_to_pop(self.on_update, name="Live Stream")

        setattr(self, f"{name}_dt_counter", 0)
        physx_iface = omni.physx.acquire_physx_interface()

        sub = physx_iface.subscribe_physics_step_events(
            partial(self.on_update_physx, name, hz, fn)
        )

        self.phyx_callbacks[name] = (hz, sub)
        return sub

    # phyx step callback, limited to specifed Hz
    def on_update_physx(self, name: str, hz: float, fn: callable, dt: float) -> None:
        """
        function to limit the frequency of a callback based on a specified Hz

        Parameters:
            name (str): The name of the callback.
            hz (float): The frequency at which the callback should be executed.
            fn (callable): The callback function to be executed.
            dt (float): The time delta.

        Returns:
            None

        """
        counter = getattr(self, f"{name}_dt_counter")
        counter += dt
        if counter >= hz:
            fn(counter)
            counter = 0

    def remove_physx_callbacks(self) -> None:
        """
        Removes all registered physics callbacks.

        This function iterates over the `phyx_callbacks` dictionary and unsubscribes each callback.
        It also removes the corresponding `<name>_dt_counter` attribute if it exists.

        """
        for name, (hz, sub) in self.phyx_callbacks.items():
            sub.unsubscribe()
            if hasattr(self, f"{name}_dt_counter"):
                delattr(self, f"{name}_dt_counter")

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


import traceback
import json
import asyncio
import struct
import zmq
import zmq.asyncio
from functools import partial


import omni
import omni.replicator.core as rep
from omni.replicator.core.scripts.utils import viewport_manager


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

        rp = viewport_manager.get_render_product(camera, resolution, force_new, name)
        self.rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
        self.rgb_annot.attach(rp)

        self.bbox2d_annot = rep.AnnotatorRegistry.get_annotator("bounding_box_2d_tight")
        self.bbox2d_annot.attach(rp)

        self.distance_to_camera_annot = rep.AnnotatorRegistry.get_annotator(
            "distance_to_camera"
        )
        self.distance_to_camera_annot.attach(rp)

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

        data = [
            self.rgb_annot.get_data().tobytes(),
            _bbox2d_data,
            self.distance_to_camera_annot.get_data().tobytes(),
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

    def context(self):
        if not self._context:
            self._context = zmq.asyncio.Context()
        return self._context

    def get_annotator(self, port: int, camera: str, resolution: tuple, annotator: str):
        soket = self.get_push_socket(port)
        annot = ZMQAnnotator(soket, camera, resolution, annotator)
        self.annotators[annotator] = annot
        return annot

    def get_push_socket(self, port: int):
        addr = f"tcp://localhost:{port}"
        sock = self.context().socket(zmq.PUSH)
        sock.set_hwm(1)
        sock.connect(addr)
        self.push_sockets[addr] = sock
        return sock

    def get_pull_socket(self, port: int):
        addr = f"tcp://localhost:{port}"
        sock = self.context().socket(zmq.PULL)
        sock.set_hwm(1)
        sock.connect(addr)
        self.pull_sockets[addr] = sock
        return sock

    async def disconnect_all(self):

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

    async def recive_data(self, sock: zmq.asyncio.Socket):
        return await sock.recv_pyobj()

    def add_physx_step_callback(self, name: str, hz: float, fn: callable):
        # ignore this snippet for now
        # update_stream = omni.kit.app.get_app().get_update_event_stream()
        # self.sub = update_stream.create_subscription_to_pop(self.on_update, name="Live Stream")

        physx_iface = omni.physx.acquire_physx_interface()
        setattr(self, f"{name}_dt_counter", 0)
        sub = physx_iface.subscribe_physics_step_events(
            partial(self.on_update_physx, name, hz, fn)
        )
        self.phyx_callbacks[name] = (hz, sub)
        return sub

    # phyx step callback, limited to specifed Hz
    def on_update_physx(self, name: str, hz: float, fn: callable, dt: float):
        counter = getattr(self, f"{name}_dt_counter")
        counter += dt
        if counter >= hz:
            fn(counter)
            counter = 0

    def remove_physx_callbacks(self):
        for name, (hz, sub) in self.phyx_callbacks.items():
            sub.unsubscribe()
            if hasattr(self, f"{name}_dt_counter"):
                delattr(self, f"{name}_dt_counter")

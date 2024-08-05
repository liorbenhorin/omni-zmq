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


import asyncio
import os
from functools import lru_cache
from pathlib import Path

from pxr import Sdf, Gf, Tf
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade

import carb
import omni.usd
import omni.ext
import omni.ui as ui

from omni.isaac.core.world import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.types import ArticulationAction

from .zmq_manager import ZMQManager


class ZMQBridge:
    def __init__(self, zmq_manager: ZMQManager):
        self.zmq_manager = zmq_manager
        self.scene_root = "/World"
        self.receive_commands = False
        self._is_streaming = False

    def set_camera(self):
        stage = omni.usd.get_context().get_stage()
        self.camera = stage.GetPrimAtPath("/World/Xform_frame/frame/Cylinder_01/Camera")

    def _set_focal_length(self, focal_length):
        try:
            self.camera.GetAttribute("focalLength").Set(focal_length)
            self.cur_focal_length = focal_length
        except:
            self.set_camera()

    def reset_world_async(self):
        async def _reset():
            self.world = World()
            await self.world.initialize_simulation_context_async()
            self.robot = Robot(
                prim_path=f"{self.scene_root}/Xform_frame/frame", name="robot"
            )
            self.world.scene.clear(registry_only=True)
            self.world.scene.add(self.robot)
            await self.world.reset_async()
            self.controller = self.robot.get_articulation_controller()

        asyncio.ensure_future(_reset())

    def reset_world(self):
        self.world = World()
        self.robot = Robot(
            prim_path=f"{self.scene_root}/Xform_frame/frame", name="robot"
        )
        self.world.scene.clear(registry_only=True)
        self.world.scene.add(self.robot)
        self.world.reset()
        self.controller = self.robot.get_articulation_controller()

    def _camera_move(self, speeds):
        self.controller.apply_action(
            ArticulationAction(
                joint_positions=None,
                joint_efforts=None,
                joint_velocities=[speeds[0], speeds[1]],
            )
        )

    async def socket_focal_lengh_in_receive_loop(self):
        self.cur_focal_length = 0

        while self.receive_commands:
            data = await self.zmq_manager.recive_data(self.socket_uav_in)
            if "focal_length" in data and data["focal_length"] != self.cur_focal_length:
                self._set_focal_length(data["focal_length"])
        print("stopped listening socket_uav_in.")

    async def socket_camera_link_in_receive_loop(self):
        while self.receive_commands:
            data = await self.zmq_manager.recive_data(self.socket_commands_in)
            if data["camera_link"]:
                j1 = data["camera_link"][0]
                j2 = data["camera_link"][1]
                self._camera_move([j1, j2])
            else:
                self._camera_move([0, 0])

        print("stopped listening")

    def start_streaming(self):
        ports = {
            "camera_annotator": 5555,
            "camera_link": 5557,
            "focal_length": 5558,
        }
        rgb_hz = 1.0 / 60.0
        dimension = 720
        camera_path = f"{self.scene_root}/Xform_frame/frame/Cylinder_01/Camera"

        self.camera_annotator = self.zmq_manager.get_annotator(
            ports["camera_annotator"],
            camera_path,
            (dimension, dimension),
            "camera_annotator",
        )

        self.zmq_manager.add_physx_step_callback(
            "camera_annotator", rgb_hz, self.camera_annotator.send
        )

        self.socket_commands_in = self.zmq_manager.get_pull_socket(ports["camera_link"])
        self.socket_uav_in = self.zmq_manager.get_pull_socket(ports["focal_length"])

        self.receive_commands = True
        asyncio.ensure_future(self.socket_camera_link_in_receive_loop())
        asyncio.ensure_future(self.socket_focal_lengh_in_receive_loop())

    def stop_streaming(self):
        self.receive_commands = False
        self.zmq_manager.remove_physx_callbacks()
        asyncio.ensure_future(self.zmq_manager.disconnect_all())

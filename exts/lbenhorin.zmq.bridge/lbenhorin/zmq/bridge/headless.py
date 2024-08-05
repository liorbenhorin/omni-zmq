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

# to run this file:
# ISAACSIM_PYTHON exts/lbenhorin.zmq.bridge/lbenhorin/zmq/bridge/headless_example.py --ext-folder /home/lbenhorin/workspaces/omni-zmq/exts

import os
from pathlib import Path
import asyncio

import isaacsim
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": True})

import carb
import omni.kit.app
from omni.kit.usd.layers import LayerUtils

import omni.replicator.core as rep


from omni.isaac.core.world import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.types import ArticulationAction


manager = omni.kit.app.get_app().get_extension_manager()
manager.set_extension_enabled_immediate("lbenhorin.zmq.bridge", True)



from lbenhorin.zmq.bridge import ZMQManager, get_data_path


class Setup:

    def __init__(self):
        self.zmq_manager = ZMQManager()

        self.assets_path = Path(get_data_path()).parent.parent.parent / "assets"
        self.context = omni.usd.get_context()
        self.stage = self.context.get_stage()

        self.scene_root = "/World"
        self.receive_commands = False
        self._is_streaming = False

    def import_example(self):
        source_usd = str(self.assets_path / "example_stage.usd")
        root_layer = self.stage.GetRootLayer()
        LayerUtils.insert_sublayer(root_layer, 0, source_usd)

    def set_camera(self):
        stage = omni.usd.get_context().get_stage()
        self.camera = stage.GetPrimAtPath("/World/Xform_frame/frame/Cylinder_01/Camera")

    def _set_focal_length(self, focal_length):
        try:
            self.camera.GetAttribute("focalLength").Set(focal_length)
            self.cur_focal_length = focal_length
        except:
            self.set_camera()

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
            "camera_annotator"
        )

        self.zmq_manager.add_physx_step_callback(
            "camera_annotator", rgb_hz, self.camera_annotator.send, world=self.world
        )

        self.socket_commands_in = self.zmq_manager.get_pull_socket(ports["camera_link"])
        self.socket_uav_in = self.zmq_manager.get_pull_socket(ports["focal_length"])

        self.receive_commands = True
        asyncio.ensure_future(self.socket_camera_link_in_receive_loop())
        asyncio.ensure_future(self.socket_focal_lengh_in_receive_loop())


setup = Setup()
setup.import_example()
setup.reset_world()
setup.start_streaming()


for i in range(20):
    simulation_app.update()
    print(f"Warm up step {i+1}/10")

print("Streaming data...")
rep.orchestrator.wait_until_complete()
simulation_app.close() 
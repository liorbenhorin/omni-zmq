import asyncio
import math
import struct
import asyncio
import zmq
import zmq.asyncio
import threading
import time
import numpy as np

from pxr import Sdf, Gf, Tf
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade

import carb
import omni.usd
import omni.ext
import omni.ui as ui

from omni.isaac.core.world import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.prims import XFormPrim

from .zmq_bridge import ZMQManager


# Any class derived from `omni.ext.IExt` in the top level module (defined in `python.modules` of `extension.toml`) will
# be instantiated when the extension gets enabled, and `on_startup(ext_id)` will be called.
# Later when the extension gets disabled on_shutdown() is called.
class LbenhorinZmqBridgeExtension(omni.ext.IExt):

    def on_startup(self, ext_id):
        print("[lbenhorin.zmq.bridge] Extension startup")

        self.zmq_manager = ZMQManager()
        self.scene_root = "/World"

        self._window = ui.Window("Omni ZMQ Bridge", width=300, height=300)
        with self._window.frame:
            with ui.VStack():
                with ui.CollapsableFrame("Core function example"):
                    with ui.HStack():
                        ui.Button("Start RGB streaming", clicked_fn=self.start_streaming)
                        ui.Button("Stop RGB streaming", clicked_fn=self.stop_streaming)
                with ui.CollapsableFrame("Scene specific function examples"):
                    with ui.HStack():
                        ui.Button("Reset world", clicked_fn=self.reset_world)

        self.receive_commands = False

    def set_camera(self):
        stage = omni.usd.get_context().get_stage()
        self.camera = stage.GetPrimAtPath('/World/Xform_frame/frame/Cylinder_01/Camera')

    def _set_focal_length(self, focal_length):
        try:
            self.camera.GetAttribute('focalLength').Set(focal_length)
            self.cur_focal_length = focal_length
        except:
            self.set_camera()
 
    def reset_world(self):
        async def _reset():
            self.world = World()
            await self.world.initialize_simulation_context_async()
            self.robot = Robot(prim_path=f'{self.scene_root}/Xform_frame/frame', name="robot")
            self.world.scene.clear(registry_only=True)
            self.world.scene.add(self.robot)
            await self.world.reset_async()
            self.controller = self.robot.get_articulation_controller()

        asyncio.ensure_future(_reset())

    def _camera_move(self, speeds):
        self.controller.apply_action(
                ArticulationAction(joint_positions=None,
                                   joint_efforts=None,
                                   joint_velocities=[speeds[0],speeds[1]]))
        
    async def socket_focal_lengh_in_receive_loop(self):
        self.cur_focal_length = 0

        while self.receive_commands:
            data = await self.zmq_manager.recive_data(self.socket_uav_in)
            if 'focal_length' in data and data['focal_length'] != self.cur_focal_length:
                self._set_focal_length(data['focal_length'])
        print('stopped listening socket_uav_in.')

    async def socket_camera_link_in_receive_loop(self):
        while self.receive_commands:
            data = await self.zmq_manager.recive_data(self.socket_commands_in)
            if data['camera_link']: 
                j1 = data['camera_link'][0]
                j2 = data['camera_link'][1]
                self._camera_move([j1, j2])
            else:
                self._camera_move([0,0])

        print('stopped listening')

    def start_streaming(self):
        ports = {
            "rgb": 5555,
            "camera_link": 5557,
            "focal_length": 5558
        }
        rgb_hz = 1.0 / 60.0
        dimension = 720
        camera_path = f"{self.scene_root}/Xform_frame/frame/Cylinder_01/Camera"
        
        self.rgb_annotator = self.zmq_manager.get_annotator(
            ports["rgb"],
            camera_path,
            (dimension, dimension),
            "rgb",
        )

        self.zmq_manager.add_physx_step_callback(
            "rgb",
            rgb_hz,
            self.rgb_annotator.send
        )

        self.socket_commands_in = self.zmq_manager.get_pull_socket(ports['camera_link'])
        self.socket_uav_in = self.zmq_manager.get_pull_socket(ports['focal_length'])

        self.receive_commands = True
        asyncio.ensure_future(self.socket_camera_link_in_receive_loop())
        asyncio.ensure_future(self.socket_focal_lengh_in_receive_loop())

    def stop_streaming(self):
        self.receive_commands = False
        self.zmq_manager.remove_physx_callbacks()
        asyncio.ensure_future(self.zmq_manager.disconnect_all())

    def on_shutdown(self):
        self.zmq_manager.remove_physx_callbacks()
        print("[lbenhorin.zmq.bridge] Extension shutdown")


# SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: MIT

import asyncio
from pathlib import Path
import time
import numpy as np

import omni.usd
from pxr import Sdf, Gf, Tf
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade


import carb
import omni.usd

from omni.isaac.core.world import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.debug_draw import _debug_draw
from omni.kit.usd.layers import LayerUtils
from omni.isaac.franka import Franka
from omni.isaac.franka.controllers.rmpflow_controller import RMPFlowController
from omni.isaac.core.prims import XFormPrim

from .core.annotators import ZMQAnnotator
from .core.mission import Mission


class CameraSurveillanceMission(Mission):
    def __init__(self):
        Mission.__init__(self)

        self.scene_root = "/World"
        self.camera_path = "/World/base_link/y_link/Camera"
        self.draw = _debug_draw.acquire_debug_draw_interface()

        self.receive_commands = False

    def start_mission(self):
        # self.receive_commands = True
        # asyncio.ensure_future(self.rmpf_loop())
        # return

        ports = {
            "camera_annotator": 5555,
            "camera_link": 5557,
            "focal_length": 5558,
        }
        rgb_hz = 1.0 / 60.0
        dimension = 720

        self.socket_camera_out = self.zmq_client.get_push_socket(
            ports["camera_annotator"]
        )
        self.socket_commands_in = self.zmq_client.get_pull_socket(ports["camera_link"])
        self.socket_uav_in = self.zmq_client.get_pull_socket(ports["focal_length"])

        self.camera_annotator = ZMQAnnotator(
            self.socket_camera_out,
            self.camera_path,
            (dimension, dimension),
            "camera_annotator",
        )

        self.zmq_client.add_physx_step_callback(
            "camera_annotator", rgb_hz, self.camera_annotator.send
        )

        self.receive_commands = True
        asyncio.ensure_future(self.socket_camera_link_in_receive_loop())
        asyncio.ensure_future(self.socket_focal_lengh_in_receive_loop())

    def draw_debug_point(self, pos: tuple):
        self.draw.clear_points()
        self.draw.draw_points([pos], [(0, 0, 1, 1)], [10])

    def set_camera(self):
        stage = omni.usd.get_context().get_stage()
        self.camera = stage.GetPrimAtPath(self.camera_path)

    def _set_focal_length(self, focal_length):
        try:
            self.camera.GetAttribute("focalLength").Set(focal_length)
            self.cur_focal_length = focal_length
        except:
            self.set_camera()

    async def _reset(self):
        self.world = World()
        await self.world.initialize_simulation_context_async()
        self.robot = Robot(prim_path=f"/World/base_link", name="robot")
        self.world.scene.clear(registry_only=True)
        self.world.scene.add(self.robot)
        await self.world.reset_async()
        self.controller = self.robot.get_articulation_controller()

    def reset_world_async(self):
        asyncio.ensure_future(self._reset())

    def reset_world(self):
        self.draw.clear_points()
        self.world = World()
        self.robot = Robot(prim_path=f"/World/base_link", name="robot")
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
            data = await self.zmq_client.recive_data(self.socket_uav_in)
            if "focal_length" in data and data["focal_length"] != self.cur_focal_length:
                self._set_focal_length(data["focal_length"])
        print("stopped listening.")

    async def socket_camera_link_in_receive_loop(self):
        while self.receive_commands:
            data = await self.zmq_client.recive_data(self.socket_commands_in)
            if data["camera_link"]:
                j1 = data["camera_link"][0]
                j2 = data["camera_link"][1]
                self._camera_move([j1, j2])
            else:
                self._camera_move([0, 0])

            self.draw_debug_point(data.get("detection_pos", [0, 0, 0]))

        print("stopped listening")

    def stop_mission(self):
        self.receive_commands = False
        self.zmq_client.remove_physx_callbacks()
        asyncio.ensure_future(self.zmq_client.disconnect_all())

    def import_world(self):
        manager = omni.kit.app.get_app().get_extension_manager()
        extension_path = manager.get_extension_path_by_module("lbenhorin.zmq.bridge")
        data_path = Path(extension_path).joinpath("data")

        context = omni.usd.get_context()
        stage = context.get_stage()
        assets_path = data_path.parent.parent.parent / "assets"
        source_usd = str(assets_path / "camera" / "camera_world.usda")
        root_layer = stage.GetRootLayer()
        LayerUtils.insert_sublayer(root_layer, 0, source_usd)


class FrankaVisionMission(CameraSurveillanceMission):
    def __init__(self):
        CameraSurveillanceMission.__init__(self)
        self.last_trigger_time = 0

    def draw_debug_point(self, pos: tuple):
        if pos == (0, 0, 0):
            return
        
        super().draw_debug_point(pos)
        current_time = time.time()
        if current_time - self.last_trigger_time > 5:
            lower_bounds = np.array([0.2, -0.2, 0.1])
            upper_bounds = np.array([.6, 0.2, .5])

            # Generate the random array
            random_array = np.random.uniform(lower_bounds, upper_bounds)
            self.target.set_world_pose(
                position=random_array
            )
        
        
            self.last_trigger_time = current_time

        actions = self.rmpf_controller.forward(
            target_end_effector_position=np.array(pos),
            target_end_effector_orientation=np.array([0, 1, 0, 0]),
        )
        self.franka_articulation_controller.apply_action(actions)

    async def _reset(self):
        await super()._reset()
        self.franka = Franka(prim_path="/World/Franka")
        self.franka.initialize()
        self.rmpf_controller = RMPFlowController(
            name="target_follower_controller", robot_articulation=self.franka
        )
        self.franka_articulation_controller = self.franka.get_articulation_controller()
        self.target = XFormPrim(prim_path="/World/Target")

    def reset_world(self):
        super().reset_world()
        self.frank_articulation = None
        self.franka_ks = None
        self.franka = None
        self.franka_articulation_controller = None
        self.rmpf_controller = None
        self.target = None

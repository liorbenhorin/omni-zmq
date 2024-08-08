# SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: MIT

# to run this file:
# ISAACSIM_PYTHON exts/lbenhorin.zmq.bridge/lbenhorin/zmq/bridge/headless.py --ext-folder /home/lbenhorin/workspaces/omni-zmq/exts


from pathlib import Path

import isaacsim
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": True})

import carb
import omni.kit.app

import omni.replicator.core as rep


manager = omni.kit.app.get_app().get_extension_manager()
manager.set_extension_enabled_immediate("lbenhorin.zmq.bridge", True)


from lbenhorin.zmq.bridge import ZMQManager
from lbenhorin.zmq.bridge.missions import CameraSurveillanceMission


_zmq_manager = ZMQManager()
mission = CameraSurveillanceMission(_zmq_manager)
mission.import_world()
mission.reset_world()
mission.start_mission()


for i in range(20):
    simulation_app.update()
    print(f"Warm up step {i+1}/10")

print("Streaming data...")
rep.orchestrator.wait_until_complete()
simulation_app.close()

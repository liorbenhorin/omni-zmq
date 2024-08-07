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
from omni.kit.usd.layers import LayerUtils

import omni.replicator.core as rep


manager = omni.kit.app.get_app().get_extension_manager()
manager.set_extension_enabled_immediate("lbenhorin.zmq.bridge", True)


from lbenhorin.zmq.bridge import ZMQManager, get_data_path
from lbenhorin.zmq.bridge import ZMQBridge


class ZMQBridgeHeadless(ZMQBridge):
    def __init__(self, zmq_manager: ZMQManager):
        ZMQBridge.__init__(self, zmq_manager)

    def import_example(self):
        context = omni.usd.get_context()
        stage = context.get_stage()
        assets_path = Path(get_data_path()).parent.parent.parent / "assets"
        source_usd = str(assets_path / "camera"/ "camera_world.usda")
        root_layer = stage.GetRootLayer()
        LayerUtils.insert_sublayer(root_layer, 0, source_usd)


_zmq_manager = ZMQManager()
setup = ZMQBridgeHeadless(_zmq_manager)
setup.import_example()
setup.reset_world()
setup.start_streaming()


for i in range(20):
    simulation_app.update()
    print(f"Warm up step {i+1}/10")

print("Streaming data...")
rep.orchestrator.wait_until_complete()
simulation_app.close()

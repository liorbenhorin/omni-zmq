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

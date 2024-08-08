# SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: MIT

import carb
import omni.usd
import omni.ext
import omni.ui as ui

from omni.kit.widget.toolbar import get_instance


from .core.ui import ZMQBridgeButtonGroup
from .example_missions import CameraSurveillanceMission


# Any class derived from `omni.ext.IExt` in the top level module (defined in `python.modules` of `extension.toml`) will
# be instantiated when the extension gets enabled, and `on_startup(ext_id)` will be called.
# Later when the extension gets disabled on_shutdown() is called.
class LbenhorinZmqBridgeExtension(omni.ext.IExt):

    def on_startup(self, ext_id):
        print("[lbenhorin.zmq.bridge] Extension startup")

        self.mission = CameraSurveillanceMission()
        self.toolbar = get_instance()
        self.button_group = ZMQBridgeButtonGroup(self.mission)
        self.toolbar.add_widget(self.button_group, 100, self.toolbar.get_context())

    def on_shutdown(self):
        self.button_group.mission.zmq_manager.remove_physx_callbacks()
        self.toolbar.remove_widget(self.button_group)
        self.button_group = None

        print("[lbenhorin.zmq.bridge] Extension shutdown")

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

from omni.kit.widget.toolbar import WidgetGroup, get_instance

from .zmq_manager import ZMQManager
from .bridge import ZMQBridge


@lru_cache()
def get_data_path() -> Path:
    manager = omni.kit.app.get_app().get_extension_manager()
    extension_path = manager.get_extension_path_by_module("lbenhorin.zmq.bridge")
    return Path(extension_path).joinpath("data")


class ZMQBridgeButtonGroup(WidgetGroup, ZMQBridge):
    def __init__(self, zmq_manager: ZMQManager):
        WidgetGroup.__init__(self)
        ZMQBridge.__init__(self, zmq_manager)

    def clean(self):
        super().clean()
        self._start_stop_button = None
        self._reset_button = None

    def get_style(self):
        return {}

    def on_start_stop_click(self):
        self._is_streaming = not self._is_streaming
        self._start_stop_button.checked = False

        if self._is_streaming:
            self._start_stop_button.image_url = f"{get_data_path()}/stop_stream.svg"
            self._start_stop_button.tooltip = "Stop Streaming"
            print("play...")  # icon has changed to stop
            self.start_streaming()
        else:
            self._start_stop_button.image_url = f"{get_data_path()}/play_stream.svg"
            self._start_stop_button.tooltip = "Start Streaming"
            print("stop...")  # icon has changed to play
            self.stop_streaming()

    def on_reset_click(self):
        self._reset_button.checked = False
        self.reset_world_async()

    def create(self, default_size):
        self._start_stop_button = ui.ToolButton(
            image_url=f"{get_data_path()}/play_stream.svg",
            name="start_stream",
            tooltip=f"Start Streaming",
            width=default_size,
            height=default_size,
            visible=not self._is_streaming,
            clicked_fn=self.on_start_stop_click,
        )

        self._reset_button = ui.ToolButton(
            image_url="${glyphs}/menu_refresh.svg",
            name="resert_world",
            tooltip=f"Reset World",
            width=default_size,
            height=default_size,
            clicked_fn=self.on_reset_click,
        )


# Any class derived from `omni.ext.IExt` in the top level module (defined in `python.modules` of `extension.toml`) will
# be instantiated when the extension gets enabled, and `on_startup(ext_id)` will be called.
# Later when the extension gets disabled on_shutdown() is called.
class LbenhorinZmqBridgeExtension(omni.ext.IExt):

    def on_startup(self, ext_id):
        print("[lbenhorin.zmq.bridge] Extension startup")

        self.zmq_manager = ZMQManager()
        self.toolbar = get_instance()
        self.button_group = ZMQBridgeButtonGroup(self.zmq_manager)
        self.toolbar.add_widget(self.button_group, 100, self.toolbar.get_context())

    def on_shutdown(self):
        self.zmq_manager.remove_physx_callbacks()
        self.toolbar.remove_widget(self.button_group)
        self.button_group = None
        print("[lbenhorin.zmq.bridge] Extension shutdown")

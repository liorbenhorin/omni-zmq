# SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: MIT

import asyncio
import os
from functools import lru_cache
from pathlib import Path

import carb
import omni.usd
import omni.ext
import omni.ui as ui

from omni.kit.widget.toolbar import WidgetGroup, get_instance

from .missions import Mission


@lru_cache()
def get_data_path() -> Path:
    manager = omni.kit.app.get_app().get_extension_manager()
    extension_path = manager.get_extension_path_by_module("lbenhorin.zmq.bridge")
    return Path(extension_path).joinpath("data")


class ZMQBridgeButtonGroup(WidgetGroup):
    def __init__(self, mission: Mission):
        WidgetGroup.__init__(self)
        self._is_streaming = False
        self.mission = mission

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
            self.mission.start_mission()
        else:
            self._start_stop_button.image_url = f"{get_data_path()}/play_stream.svg"
            self._start_stop_button.tooltip = "Start Streaming"
            print("stop...")  # icon has changed to play
            self.mission.stop_mission()

    def on_reset_click(self):
        self._reset_button.checked = False
        self.mission.reset_world_async()

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

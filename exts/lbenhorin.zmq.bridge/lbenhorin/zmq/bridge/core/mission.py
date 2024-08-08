# SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: MIT

from .client import ZMQClient

class Mission:
    def __init__(self):
        self.zmq_client = ZMQClient()

    def reset_world_async(self):
        print("[lbenhorin.zmq.bridge] reset world async: NOT IMPLEMENTED")

    def reset_world(self):
        print("[lbenhorin.zmq.bridge] reset world: NOT IMPLEMENTED")

    def start_mission(self):
        print("[lbenhorin.zmq.bridge] start mission: NOT IMPLEMENTED")

    def stop_mission(self):
        print("[lbenhorin.zmq.bridge] stop mission: NOT IMPLEMENTED")

    def import_world(self):
        print("[lbenhorin.zmq.bridge] import world: NOT IMPLEMENTED")
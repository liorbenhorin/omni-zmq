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


import zmq
import threading
import time


class ZMQManager:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super().__new__(cls, *args, **kwargs)
        return cls._instance

    def __init__(self):
        self.push_sockets = {}
        self.pull_sockets = {}
        self.reciveing_threads = {}
        self.sending_threads = {}

        self._context = None

    def context(self):
        if not self._context:
            self._context = zmq.Context()
        return self._context

    def get_pull_socket(self, port: int):
        addr = f"tcp://*:{port}"
        sock = self.context().socket(zmq.PULL)
        sock.set_hwm(1)
        sock.bind(addr)
        sock.setsockopt(zmq.RCVTIMEO, 1000)  # 1 sec
        poller = zmq.Poller()
        poller.register(sock, zmq.POLLIN)
        self.pull_sockets[addr] = sock
        return sock

    def get_push_socket(self, port: int):
        addr = f"tcp://*:{port}"
        sock = self.context().socket(zmq.PUSH)
        sock.bind(addr)
        self.push_sockets[addr] = sock
        return sock

    def recive_from_socket_in_loop(self, name: str, port: int, fn: callable):
        sock = self.get_pull_socket(port)
        stop_event = threading.Event()

        def loop():
            while not stop_event.is_set():
                try:
                    msg = sock.recv_multipart()
                    fn(msg)
                except zmq.Again:
                    continue
                except:
                    print("unable to unpack from socket...")
                    continue

            sock.close()

        worker = threading.Thread(target=loop)
        self.reciveing_threads[name] = (worker, stop_event)
        worker.start()

    def send_from_socket_in_loop(
        self, name: str, port: int, rate_hz: float, fn: callable
    ):

        sock = self.get_push_socket(port)
        stop_event = threading.Event()

        def loop():
            while not stop_event.is_set():
                sock.send_pyobj(fn())
                time.sleep(1 / rate_hz)

            sock.close()

        worker = threading.Thread(target=loop)
        self.sending_threads[name] = (worker, stop_event)
        worker.start()

    def cleanup(self):
        for name, (worker, stop_event) in self.reciveing_threads.items():
            stop_event.set()
            worker.join()

        for name, (worker, stop_event) in self.sending_threads.items():
            stop_event.set()
            worker.join()

# SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: MIT


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
        sock.setsockopt(zmq.SNDTIMEO, 1000)  # 1 sec
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
                try:
                    sock.send_pyobj(fn())
                except zmq.Again:
                    continue
                except:
                    print("unable to send to socket...")
                    continue
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

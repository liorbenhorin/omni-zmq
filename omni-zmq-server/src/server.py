# SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: MIT


import zmq
import threading
import time


class ZMQServer:
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

    def context(self) -> zmq.Context:
        """
        Returns the ZMQ context instance.
        If the context has not been initialized, it creates a new ZMQ context and assigns it to the `_context` attribute.

        Returns:
            zmq.Context: The ZMQ context instance.
        """
        if not self._context:
            self._context = zmq.Context()
        return self._context

    def get_pull_socket(self, port: int) -> zmq.Socket:
        """
        Creates and returns a new pull socket that is bound to the specified port.

        Args:
            port (int): The port number to bind the socket to.

        Returns:
            zmq.Socket: The newly created pull socket.
        """
        addr = f"tcp://*:{port}"
        sock = self.context().socket(zmq.PULL)
        sock.set_hwm(1)
        sock.bind(addr)
        sock.setsockopt(zmq.RCVTIMEO, 1000)  # 1 sec
        poller = zmq.Poller()
        poller.register(sock, zmq.POLLIN)
        self.pull_sockets[addr] = sock
        return sock

    def get_push_socket(self, port: int) -> zmq.Socket:
        """
        Creates and returns a ZeroMQ PUSH socket bound to the specified port.

        Args:
            port (int): The port number to bind the socket to.

        Returns:
            zmq.Socket: The created PUSH socket.

        """
        addr = f"tcp://*:{port}"
        sock = self.context().socket(zmq.PUSH)
        sock.setsockopt(zmq.SNDTIMEO, 1000)  # 1 sec
        sock.bind(addr)
        self.push_sockets[addr] = sock
        return sock

    def recive_from_socket_in_loop(self, name: str, port: int, fn: callable) -> None:
        """
        Receives messages from a socket in a loop and calls a given function for each message.

        Args:
            name (str): The name of the receiving thread.
            port (int): The port number to receive messages from.
            fn (callable): A callable function that takes a message as input.

        """
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
    ) -> None:
        """
        Sends data from a socket in a loop at a specified rate.

        Args:
            name (str): The name of the sending thread.
            port (int): The port number to send data to.
            rate_hz (float): The rate at which data is sent in Hz.
            fn (callable): A callable function that returns the data to send.
        """
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

    def cleanup(self) -> None:
        """
        Stops and joins all receiving and sending threads.

        This function is used to clean up the threads when they are no longer needed.
        It sets the stop event for each thread and then joins them to ensure they have finished.

        """
        for name, (worker, stop_event) in self.reciveing_threads.items():
            stop_event.set()
            worker.join()

        for name, (worker, stop_event) in self.sending_threads.items():
            stop_event.set()
            worker.join()

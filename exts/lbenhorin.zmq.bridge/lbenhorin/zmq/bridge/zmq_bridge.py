import traceback
import asyncio
import struct
import zmq
import zmq.asyncio
from functools import partial


import omni
import omni.replicator.core as rep
from omni.replicator.core.scripts.utils import viewport_manager


class ZMQManager:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super().__new__(cls, *args, **kwargs)
        return cls._instance

    def __init__(self):
        self.push_sockets = {}
        self.pull_sockets = {}
        self.phyx_callbacks = {}
        self.annotators = {}
        self._context = None

    def context(self):
        if not self._context:
            self._context = zmq.asyncio.Context()
        return self._context

    def get_annotator(self, port: int, camera: str, resolution: tuple, annotator: str):
        annot = ZMQAnnotator(self, port, camera, resolution, annotator)
        self.annotators[annotator] = annot
        return annot

    def get_push_socket(self, port: int):
        addr = f"tcp://localhost:{port}"
        sock = self.context().socket(zmq.PUSH)
        sock.set_hwm(1)
        sock.connect(addr)
        self.push_sockets[addr] = sock
        return sock

    def get_pull_socket(self, port: int):
        addr = f"tcp://localhost:{port}"
        sock = self.context().socket(zmq.PULL)
        sock.set_hwm(1)
        sock.connect(addr)
        self.pull_sockets[addr] = sock
        return sock

    async def disconnect_all(self):

        for addr, sock in self.push_sockets.items():
            await asyncio.sleep(0.1)
            try:
                sock.disconnect(addr)
            except asyncio.CancelledError:
                pass
            except Exception:
                print(traceback.format_exc())

            sock.close()
        for addr, sock in self.pull_sockets.items():
            await asyncio.sleep(0.1)
            try:
                sock.disconnect(addr)
            except asyncio.CancelledError:
                pass
            except Exception:
                print(traceback.format_exc())

            sock.close()

        self.pull_sockets = {}
        self.push_sockets = {}

        self._context.term()
        self._context = None

    async def recive_data(self, sock: zmq.asyncio.Socket):
        return await sock.recv_pyobj()

    def add_physx_step_callback(self, name: str, hz: float, fn: callable):
        # ignore this snippet for now
        # update_stream = omni.kit.app.get_app().get_update_event_stream()
        # self.sub = update_stream.create_subscription_to_pop(self.on_update, name="Live Stream")

        physx_iface = omni.physx.acquire_physx_interface()
        setattr(self, f"{name}_dt_counter", 0)
        sub = physx_iface.subscribe_physics_step_events(
            partial(self.on_update_physx, name, hz, fn)
        )
        self.phyx_callbacks[name] = (hz, sub)
        return sub

    # phyx step callback, limited to specifed Hz
    def on_update_physx(self, name: str, hz: float, fn: callable, dt: float):
        counter = getattr(self, f"{name}_dt_counter")
        counter += dt
        if counter >= hz:
            fn(counter)
            counter = 0

    def remove_physx_callbacks(self):
        for name, (hz, sub) in self.phyx_callbacks.items():
            sub.unsubscribe()
            delattr(self, f"{name}_dt_counter")


class ZMQAnnotator:
    def __init__(
        self,
        manager: ZMQManager,
        port: int,
        camera: str,
        resolution: tuple,
        annotator: str,
    ):

        self.manager = manager
        self.sock = self.manager.get_push_socket(port)

        force_new = False
        name = f"{camera.split('/')[-1]}_rp"

        rp = viewport_manager.get_render_product(camera, resolution, force_new, name)
        self.annot = rep.AnnotatorRegistry.get_annotator(annotator)
        self.annot.attach(rp)

    def send(self, dt: float):
        _dt = struct.pack("f", dt)
        data = [self.annot.get_data().tobytes(), _dt]
        asyncio.ensure_future(self.sock.send_multipart(data))

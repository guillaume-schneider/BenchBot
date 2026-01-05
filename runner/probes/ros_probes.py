from __future__ import annotations
import time
from dataclasses import dataclass
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.qos import qos_profile_sensor_data

from runner.probes.base import Probe, ProbeResult

# TF2
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration


@dataclass
class ProbeContext:
    node: Node


def _spin_until(node: Node, predicate, timeout_s: float, spin_dt_s: float = 0.05) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=spin_dt_s)
        if predicate():
            return True
    return False


class TopicPublishProbe(Probe):
    def __init__(self, topic: str, min_messages: int = 1, timeout_s: float = 10.0):
        self.topic = topic
        self.min_messages = int(min_messages)
        self.timeout_s = float(timeout_s)

    def run(self, ctx: ProbeContext) -> ProbeResult:
        node = ctx.node
        count = {"n": 0}

        # Use AnyMsg-style subscription: easiest is to use std_msgs/String? Not ok.
        # In practice, you should know msg types; for generic probing, use /rosapi is not available.
        # So we probe existence by graph + then count via ros2 topic echo is too heavy.
        # Better: implement per-known topics with types, or provide msg_type in config.
        # Here: require msg_type in params; we'll support that in a second probe below.
        return ProbeResult(
            ok=False,
            message=f"TopicPublishProbe requires msg_type. Use TopicPublishTypedProbe(topic=..., msg_type='sensor_msgs/msg/LaserScan')."
        )


class TopicPublishTypedProbe(Probe):
    def __init__(self, topic: str, msg_type: str, min_messages: int = 1, timeout_s: float = 10.0):
        self.topic = topic
        self.msg_type = msg_type
        self.min_messages = int(min_messages)
        self.timeout_s = float(timeout_s)

    def run(self, ctx: ProbeContext) -> ProbeResult:
        node = ctx.node

        # dynamic import of message type string "pkg/msg/Type"
        try:
            mod_name, cls_name = self._resolve_import(self.msg_type)
            mod = __import__(mod_name, fromlist=[cls_name])
            Msg = getattr(mod, cls_name)
        except Exception as e:
            return ProbeResult(False, f"Cannot import msg_type={self.msg_type}: {e}")

        n = 0

        def cb(_msg):
            nonlocal n
            n += 1

        sub = node.create_subscription(Msg, self.topic, cb, 10)

        ok = _spin_until(node, lambda: n >= self.min_messages, self.timeout_s)
        node.destroy_subscription(sub)

        if ok:
            return ProbeResult(True, f"Received {n} msgs on {self.topic}")
        return ProbeResult(False, f"Timeout waiting {self.min_messages} msgs on {self.topic}", {"received": n})

    @staticmethod
    def _resolve_import(msg_type: str) -> tuple[str, str]:
        # accepts "sensor_msgs/msg/LaserScan" or "sensor_msgs.msg.LaserScan"
        if "/" in msg_type:
            pkg, _, name = msg_type.split("/", 2)
            if name.startswith("msg/"):
                name = name.split("/", 1)[1]
            return f"{pkg}.msg", name
        # dotted
        parts = msg_type.split(".")
        return ".".join(parts[:-1]), parts[-1]


class TopicHzTypedProbe(Probe):
    def __init__(self, topic: str, msg_type: str, min_hz: float, window_s: float = 5.0, timeout_s: float = 20.0):
        self.topic = topic
        self.msg_type = msg_type
        self.min_hz = float(min_hz)
        self.window_s = float(window_s)
        self.timeout_s = float(timeout_s)

    def run(self, ctx: ProbeContext) -> ProbeResult:
        node = ctx.node
        try:
            mod_name, cls_name = TopicPublishTypedProbe._resolve_import(self.msg_type)
            mod = __import__(mod_name, fromlist=[cls_name])
            Msg = getattr(mod, cls_name)
        except Exception as e:
            return ProbeResult(False, f"Cannot import msg_type={self.msg_type}: {e}")

        stamps = []

        def cb(_msg):
            stamps.append(time.time())

        sub = node.create_subscription(Msg, self.topic, cb, qos_profile_sensor_data)

        def enough():
            # keep only window
            now = time.time()
            while stamps and stamps[0] < now - self.window_s:
                stamps.pop(0)
            if len(stamps) < 2:
                return False
            hz = (len(stamps) - 1) / max(1e-6, (stamps[-1] - stamps[0]))
            return hz >= self.min_hz

        ok = _spin_until(node, enough, self.timeout_s)
        node.destroy_subscription(sub)

        if not stamps:
            return ProbeResult(False, f"No messages received on {self.topic}")
        now = time.time()
        w = [s for s in stamps if s >= now - self.window_s]
        hz = 0.0
        if len(w) >= 2:
            hz = (len(w) - 1) / max(1e-6, (w[-1] - w[0]))

        if ok:
            return ProbeResult(True, f"{self.topic} hz={hz:.2f} >= {self.min_hz}")
        return ProbeResult(False, f"{self.topic} hz={hz:.2f} < {self.min_hz}", {"hz": hz})


class TfAvailableProbe(Probe):
    def __init__(self, from_frame: str, to_frame: str, timeout_s: float = 10.0):
        self.from_frame = from_frame
        self.to_frame = to_frame
        self.timeout_s = float(timeout_s)

    def run(self, ctx: ProbeContext) -> ProbeResult:
        node = ctx.node
        buf = Buffer()
        listener = TransformListener(buf, node, spin_thread=False)

        def can():
            try:
                # allow a small timeout per check
                return buf.can_transform(self.from_frame, self.to_frame, rclpy.time.Time(), timeout=Duration(seconds=0.1))
            except Exception:
                return False

        ok = _spin_until(node, can, self.timeout_s)
        # listener is tied to node; no explicit destroy needed beyond letting it go

        if ok:
            return ProbeResult(True, f"TF available {self.from_frame}->{self.to_frame}")
        return ProbeResult(False, f"TF not available {self.from_frame}->{self.to_frame} within {self.timeout_s}s")


class ServiceAvailableProbe(Probe):
    def __init__(self, service: str, srv_type: str, timeout_s: float = 10.0):
        self.service = service
        self.srv_type = srv_type
        self.timeout_s = float(timeout_s)

    def run(self, ctx: ProbeContext) -> ProbeResult:
        node = ctx.node
        try:
            mod_name, cls_name = TopicPublishTypedProbe._resolve_import(self.srv_type)
            # srv import path is pkg.srv
            mod_name = mod_name.replace(".msg", ".srv")
            mod = __import__(mod_name, fromlist=[cls_name])
            Srv = getattr(mod, cls_name)
        except Exception as e:
            return ProbeResult(False, f"Cannot import srv_type={self.srv_type}: {e}")

        client = node.create_client(Srv, self.service)

        ok = _spin_until(node, lambda: client.service_is_ready(), self.timeout_s)
        node.destroy_client(client)

        if ok:
            return ProbeResult(True, f"Service ready: {self.service}")
        return ProbeResult(False, f"Service not ready: {self.service} within {self.timeout_s}s")


class NodePresentProbe(Probe):
    def __init__(self, node_name: str, timeout_s: float = 10.0):
        self.node_name = node_name
        self.timeout_s = float(timeout_s)

    def run(self, ctx: ProbeContext) -> ProbeResult:
        node = ctx.node

        def present():
            names = node.get_node_names()  # without namespaces; we need full names:
            full = node.get_node_names_and_namespaces()
            full_names = [f"{ns.rstrip('/')}/{n}".replace("//", "/") for (n, ns) in full]
            return self.node_name in full_names or self.node_name in names

        ok = _spin_until(node, present, self.timeout_s)
        if ok:
            return ProbeResult(True, f"Node present: {self.node_name}")
        return ProbeResult(False, f"Node not present: {self.node_name} within {self.timeout_s}s")

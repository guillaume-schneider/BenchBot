from __future__ import annotations
from dataclasses import dataclass
from typing import Any


@dataclass
class ProbeResult:
    ok: bool
    message: str
    details: dict[str, Any] | None = None


class Probe:
    def run(self, ctx: "ProbeContext") -> ProbeResult:
        raise NotImplementedError

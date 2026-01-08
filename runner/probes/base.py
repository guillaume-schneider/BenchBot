"""Base classes for ROS 2 readiness probes.

Defines the probe interface and result structure.
"""
from __future__ import annotations
from dataclasses import dataclass
from typing import Any


@dataclass
class ProbeResult:
    """Result of a probe execution.
    
    Attributes:
        ok: Whether the probe succeeded
        message: Human-readable result message
        details: Optional additional data
    """
    ok: bool
    message: str
    details: dict[str, Any] | None = None


class Probe:
    """Base class for all probes.
    
    Subclasses must implement the run() method to perform
    specific readiness checks.
    """
    def run(self, ctx: "ProbeContext") -> ProbeResult:
        """Execute the probe.
        
        Args:
            ctx: Probe context containing ROS 2 node
            
        Returns:
            ProbeResult indicating success or failure
        """
        raise NotImplementedError

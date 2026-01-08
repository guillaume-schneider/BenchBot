"""Process management utilities for orchestrating ROS 2 and simulator processes.

This module provides robust process lifecycle management with proper signal handling
and process group management for clean shutdown of complex process trees.
"""
from __future__ import annotations
import os
import signal
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


@dataclass
class ManagedProcess:
    """Container for a managed subprocess with associated metadata.
    
    Attributes:
        name: Human-readable identifier for the process
        popen: The subprocess.Popen instance
        log_path: Path to the log file capturing stdout/stderr
    """
    name: str
    popen: subprocess.Popen
    log_path: Path


class ProcessManager:
    """Manages the lifecycle of multiple subprocesses with centralized logging.
    
    This class handles starting, monitoring, and gracefully stopping process trees.
    It uses process groups (via os.setsid) to ensure child processes are properly
    terminated when the parent is stopped.
    
    Args:
        logs_dir: Directory where process logs will be written
    """
    
    def __init__(self, logs_dir: str):
        self.logs_dir = Path(logs_dir)
        self.logs_dir.mkdir(parents=True, exist_ok=True)
        self.procs: dict[str, ManagedProcess] = {}

    def start(self, name: str, cmd: list[str], env: dict[str, str] | None = None, cwd: str | None = None) -> None:
        """Start a new managed process.
        
        Args:
            name: Unique identifier for this process
            cmd: Command and arguments as a list
            env: Optional environment variables to merge with os.environ
            cwd: Optional working directory for the process
            
        Raises:
            RuntimeError: If a process with this name already exists
        """
        if name in self.procs:
            raise RuntimeError(f"process already exists: {name}")

        log_path = self.logs_dir / f"{name}.log"
        f = open(log_path, "wb")

        merged_env = os.environ.copy()
        if env:
            merged_env.update({str(k): str(v) for k, v in env.items()})

        # Create a process group so we can SIGINT the whole tree.
        popen = subprocess.Popen(
            cmd,
            stdout=f,
            stderr=subprocess.STDOUT,
            cwd=cwd,
            env=merged_env,
            preexec_fn=os.setsid,   # POSIX only, OK for Ubuntu
        )
        self.procs[name] = ManagedProcess(name=name, popen=popen, log_path=log_path)

    def is_running(self, name: str) -> bool:
        """Check if a managed process is still running.
        
        Args:
            name: Process identifier
            
        Returns:
            True if the process is running, False otherwise
        """
        p = self.procs[name].popen
        return p.poll() is None

    def stop(self, name: str, sigint_timeout_s: float = 8.0, sigterm_timeout_s: float = 4.0) -> None:
        """Gracefully stop a managed process with escalating signals.
        
        Attempts to stop the process using a three-stage approach:
        1. SIGINT (ROS 2 friendly) - wait up to sigint_timeout_s
        2. SIGTERM - wait up to sigterm_timeout_s
        3. SIGKILL - forceful termination
        
        Args:
            name: Process identifier
            sigint_timeout_s: Seconds to wait after SIGINT before escalating
            sigterm_timeout_s: Seconds to wait after SIGTERM before SIGKILL
        """
        mp = self.procs.get(name)
        if not mp:
            return
        p = mp.popen
        if p.poll() is not None:
            return

        pgid = os.getpgid(p.pid)

        # 1) SIGINT (ROS2-friendly)
        os.killpg(pgid, signal.SIGINT)
        t0 = time.time()
        while time.time() - t0 < sigint_timeout_s:
            if p.poll() is not None:
                return
            time.sleep(0.1)

        # 2) SIGTERM
        os.killpg(pgid, signal.SIGTERM)
        t1 = time.time()
        while time.time() - t1 < sigterm_timeout_s:
            if p.poll() is not None:
                return
            time.sleep(0.1)

        # 3) SIGKILL
        os.killpg(pgid, signal.SIGKILL)

    def stop_all(self) -> None:
        """Stop all managed processes.
        
        Iterates through all processes and attempts to stop them gracefully.
        Exceptions during individual process stops are silently ignored.
        """
        for name in list(self.procs.keys()):
            try:
                self.stop(name)
            except Exception:
                pass

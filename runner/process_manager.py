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
    name: str
    popen: subprocess.Popen
    log_path: Path


class ProcessManager:
    def __init__(self, logs_dir: str):
        self.logs_dir = Path(logs_dir)
        self.logs_dir.mkdir(parents=True, exist_ok=True)
        self.procs: dict[str, ManagedProcess] = {}

    def start(self, name: str, cmd: list[str], env: dict[str, str] | None = None, cwd: str | None = None) -> None:
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
        p = self.procs[name].popen
        return p.poll() is None

    def stop(self, name: str, sigint_timeout_s: float = 8.0, sigterm_timeout_s: float = 4.0) -> None:
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
        for name in list(self.procs.keys()):
            try:
                self.stop(name)
            except Exception:
                pass

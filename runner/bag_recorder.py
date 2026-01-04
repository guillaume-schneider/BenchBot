from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


@dataclass
class RosbagConfig:
    storage_id: str = "sqlite3"
    compression: Optional[str] = None   # "zstd" | "lz4"
    max_bag_size_mb: Optional[int] = None
    include_hidden_topics: bool = False
    qos_overrides_path: Optional[str] = None
    topics: list[str] = None


def build_rosbag_cmd(output_dir: str, cfg: RosbagConfig) -> list[str]:
    cmd = ["ros2", "bag", "record"]
    cmd += ["-o", output_dir]
    cmd += ["--storage", cfg.storage_id]

    if cfg.compression:
        cmd += ["--compression-mode", "file", "--compression-format", cfg.compression]

    if cfg.max_bag_size_mb:
        # rosbag expects bytes
        cmd += ["--max-bag-size", str(int(cfg.max_bag_size_mb) * 1024 * 1024)]

    if cfg.include_hidden_topics:
        cmd += ["--include-hidden-topics"]

    if cfg.qos_overrides_path:
        cmd += ["--qos-profile-overrides-path", cfg.qos_overrides_path]

    # topics
    for t in (cfg.topics or []):
        cmd.append(t)

    return cmd

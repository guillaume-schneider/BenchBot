"""Configuration and command building for ROS 2 bag recording.

This module provides utilities to configure and generate rosbag2 record commands
with various options like compression, storage format, and topic filtering.
"""
from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


@dataclass
class RosbagConfig:
    """Configuration for rosbag2 recording.
    
    Attributes:
        storage_id: Storage backend ("sqlite3" or "mcap")
        compression: Optional compression format ("zstd" or "lz4")
        max_bag_size_mb: Maximum size per bag file in megabytes
        include_hidden_topics: Whether to record hidden topics (e.g., /rosout)
        qos_overrides_path: Path to QoS profile overrides YAML
        topics: List of topic names to record
    """
    storage_id: str = "sqlite3"
    compression: Optional[str] = None   # "zstd" | "lz4"
    max_bag_size_mb: Optional[int] = None
    include_hidden_topics: bool = False
    qos_overrides_path: Optional[str] = None
    topics: list[str] = None


def build_rosbag_cmd(output_dir: str, cfg: RosbagConfig) -> list[str]:
    """Build a ros2 bag record command from configuration.
    
    Args:
        output_dir: Directory where the bag will be saved
        cfg: RosbagConfig with recording options
        
    Returns:
        Command as a list of strings ready for subprocess.Popen
        
    Example:
        >>> cfg = RosbagConfig(topics=["/scan", "/odom"], compression="zstd")
        >>> cmd = build_rosbag_cmd("/tmp/mybag", cfg)
        >>> # cmd = ["ros2", "bag", "record", "-o", "/tmp/mybag", ...]
    """
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

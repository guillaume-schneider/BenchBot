"""Benchmark Evaluation Package

This package provides tools for evaluating SLAM benchmark runs against ground truth maps.
"""

from .metrics import (
    load_gt_map,
    read_messages_by_topic,
    occupancy_arrays_from_msgs,
    compute_coverage,
    compute_accessible_coverage,
    compute_iou,
    compute_time_to_coverage,
    compute_path_length,
    align_est_map_to_gt,
    get_trajectory,
    detect_anomalies,
    compute_ssim,
    compute_wall_thickness,
    save_map_image
)

__all__ = [
    'load_gt_map',
    'read_messages_by_topic',
    'occupancy_arrays_from_msgs',
    'compute_coverage',
    'compute_accessible_coverage',
    'compute_iou',
    'compute_time_to_coverage',
    'compute_path_length',
    'align_est_map_to_gt',
    'get_trajectory',
    'detect_anomalies',
    'compute_ssim',
    'compute_wall_thickness',
    'save_map_image'
]

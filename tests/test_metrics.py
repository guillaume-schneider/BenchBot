"""
Unit tests for evaluation metrics module.

Tests coverage for:
- ATE calculation
- Map quality metrics (Coverage, IoU, SSIM, Wall Thickness)
- Anomaly detection
"""

import pytest
import numpy as np
from pathlib import Path
import sys

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from evaluation.metrics import (
    compute_coverage,
    compute_iou,
    compute_ssim,
    compute_wall_thickness,
    detect_anomalies,
    compute_path_length,
)


class TestMapQualityMetrics:
    """Test suite for map quality metrics."""
    
    def test_coverage_empty_maps(self):
        """Coverage should be 0 for empty maps."""
        gt_map = np.full((100, 100), -1, dtype=np.int8)  # All unknown
        est_map = np.full((100, 100), -1, dtype=np.int8)
        
        coverage = compute_coverage(gt_map, est_map)
        assert coverage == 0.0
    
    def test_coverage_perfect_match(self):
        """Coverage should be 1.0 when all GT free cells are known."""
        gt_map = np.zeros((100, 100), dtype=np.int8)  # All free
        est_map = np.zeros((100, 100), dtype=np.int8)  # All free
        
        coverage = compute_coverage(gt_map, est_map)
        assert coverage == 1.0
    
    def test_coverage_partial(self):
        """Coverage should be 0.5 when half of GT is explored."""
        gt_map = np.zeros((100, 100), dtype=np.int8)  # All free
        est_map = np.full((100, 100), -1, dtype=np.int8)  # All unknown
        est_map[:50, :] = 0  # Top half known
        
        coverage = compute_coverage(gt_map, est_map)
        assert 0.49 < coverage < 0.51  # ~0.5
    
    def test_iou_identical_maps(self):
        """IoU should be 1.0 for identical occupied cells."""
        gt_map = np.zeros((100, 100), dtype=np.int8)
        gt_map[40:60, 40:60] = 100  # Occupied square
        
        est_map = gt_map.copy()
        
        iou = compute_iou(gt_map, est_map)
        assert iou == 1.0
    
    def test_iou_no_overlap(self):
        """IoU should be 0.0 for completely different occupied cells."""
        gt_map = np.zeros((100, 100), dtype=np.int8)
        gt_map[10:30, 10:30] = 100  # Top-left square
        
        est_map = np.zeros((100, 100), dtype=np.int8)
        est_map[70:90, 70:90] = 100  # Bottom-right square
        
        iou = compute_iou(gt_map, est_map)
        assert iou == 0.0
    
    def test_iou_partial_overlap(self):
        """IoU should be between 0 and 1 for partial overlap."""
        gt_map = np.zeros((100, 100), dtype=np.int8)
        gt_map[40:60, 40:60] = 100  # 20x20 square
        
        est_map = np.zeros((100, 100), dtype=np.int8)
        est_map[50:70, 50:70] = 100  # Overlapping 20x20 square
        
        iou = compute_iou(gt_map, est_map)
        assert 0.0 < iou < 1.0
        # Intersection: 10x10 = 100, Union: 20x20 + 20x20 - 10x10 = 700
        # IoU = 100/700 ≈ 0.143
        assert 0.10 < iou < 0.20


class TestSSIMMetric:
    """Test suite for SSIM (Structural Similarity Index)."""
    
    def test_ssim_identical_maps(self):
        """SSIM should be ~1.0 for identical maps."""
        map_a = np.random.randint(0, 100, (100, 100), dtype=np.int8)
        map_b = map_a.copy()
        
        ssim_score = compute_ssim(map_a, map_b)
        assert ssim_score > 0.99  # Allow for floating point errors
    
    def test_ssim_completely_different(self):
        """SSIM should be low for completely different maps."""
        map_a = np.zeros((100, 100), dtype=np.int8)
        map_b = np.full((100, 100), 100, dtype=np.int8)
        
        ssim_score = compute_ssim(map_a, map_b)
        assert ssim_score < 0.5
    
    def test_ssim_no_known_cells(self):
        """SSIM should return 0.0 when no cells are known."""
        map_a = np.full((100, 100), -1, dtype=np.int8)
        map_b = np.full((100, 100), -1, dtype=np.int8)
        
        ssim_score = compute_ssim(map_a, map_b)
        assert ssim_score == 0.0


class TestWallThickness:
    """Test suite for wall thickness analysis."""
    
    def test_wall_thickness_no_walls(self):
        """Wall thickness should be 0 for maps without walls."""
        map_no_walls = np.zeros((100, 100), dtype=np.int8)  # All free
        
        thickness = compute_wall_thickness(map_no_walls, resolution=0.05)
        assert thickness == 0.0
    
    def test_wall_thickness_thin_wall(self):
        """Wall thickness should be small for thin walls."""
        map_thin = np.zeros((100, 100), dtype=np.int8)
        map_thin[50, :] = 100  # 1-pixel horizontal wall
        
        thickness = compute_wall_thickness(map_thin, resolution=0.05)
        # Should be approximately 2 * 0.5 * 0.05 = 0.05m = 5cm
        assert 0.03 < thickness < 0.10
    
    def test_wall_thickness_thick_wall(self):
        """Wall thickness should be larger for thick walls."""
        map_thick = np.zeros((100, 100), dtype=np.int8)
        map_thick[45:55, :] = 100  # 10-pixel horizontal wall
        
        thickness = compute_wall_thickness(map_thick, resolution=0.05)
        # Should be significantly larger than thin wall
        assert thickness > 0.10


class TestAnomalyDetection:
    """Test suite for anomaly detection heuristics."""
    
    def create_mock_odom_msgs(self, positions, dt=1.0):
        """Helper to create mock odometry messages."""
        from collections import namedtuple
        
        MockPose = namedtuple('MockPose', ['position'])
        MockPosition = namedtuple('MockPosition', ['x', 'y'])
        MockOdomMsg = namedtuple('MockOdomMsg', ['pose'])
        MockOdomPose = namedtuple('MockOdomPose', ['pose'])
        
        msgs = []
        for i, (x, y) in enumerate(positions):
            pos = MockPosition(x=x, y=y)
            pose = MockPose(position=pos)
            odom_pose = MockOdomPose(pose=pose)
            msg = MockOdomMsg(pose=odom_pose)
            msgs.append((i * dt, msg))
        
        return msgs
    
    def test_detect_stuck_robot(self):
        """Should detect robot that barely moved."""
        # Robot moved only 0.1m
        odom_msgs = self.create_mock_odom_msgs([
            (0.0, 0.0),
            (0.05, 0.0),
            (0.1, 0.0)
        ])
        
        warnings, is_failure = detect_anomalies(odom_msgs)
        
        assert is_failure is True
        assert any("barely moved" in w.lower() for w in warnings)
    
    def test_detect_tf_jump(self):
        """Should detect teleportation (TF jump)."""
        # Robot teleports 10m in 1 second (10 m/s >> 1.5 m/s threshold)
        odom_msgs = self.create_mock_odom_msgs([
            (0.0, 0.0),
            (10.0, 0.0),  # Instant jump
            (10.1, 0.0)
        ], dt=1.0)
        
        warnings, is_failure = detect_anomalies(odom_msgs)
        
        assert is_failure is True
        assert any("jump" in w.lower() for w in warnings)
    
    def test_detect_massive_drift(self):
        """Should detect massive ATE drift."""
        odom_msgs = self.create_mock_odom_msgs([
            (0.0, 0.0),
            (1.0, 0.0),
            (2.0, 0.0)
        ])
        
        ate_rmse = 2.5  # Very high drift
        warnings, is_failure = detect_anomalies(odom_msgs, ate_rmse)
        
        assert is_failure is True
        assert any("drift" in w.lower() for w in warnings)
    
    def test_healthy_run(self):
        """Should not flag a healthy run."""
        # Robot moved 5m smoothly
        odom_msgs = self.create_mock_odom_msgs([
            (0.0, 0.0),
            (1.0, 0.0),
            (2.0, 0.0),
            (3.0, 0.0),
            (4.0, 0.0),
            (5.0, 0.0)
        ], dt=1.0)
        
        ate_rmse = 0.05  # Low drift
        warnings, is_failure = detect_anomalies(odom_msgs, ate_rmse)
        
        assert is_failure is False
        assert len(warnings) == 0


class TestPathLength:
    """Test suite for path length calculation."""
    
    def create_mock_odom_msgs(self, positions):
        """Helper to create mock odometry messages."""
        from collections import namedtuple
        
        MockPose = namedtuple('MockPose', ['position'])
        MockPosition = namedtuple('MockPosition', ['x', 'y'])
        MockOdomMsg = namedtuple('MockOdomMsg', ['pose'])
        MockOdomPose = namedtuple('MockOdomPose', ['pose'])
        
        msgs = []
        for i, (x, y) in enumerate(positions):
            pos = MockPosition(x=x, y=y)
            pose = MockPose(position=pos)
            odom_pose = MockOdomPose(pose=pose)
            msg = MockOdomMsg(pose=odom_pose)
            msgs.append((i, msg))
        
        return msgs
    
    def test_path_length_straight_line(self):
        """Path length should be correct for straight line."""
        odom_msgs = self.create_mock_odom_msgs([
            (0.0, 0.0),
            (1.0, 0.0),
            (2.0, 0.0),
            (3.0, 0.0)
        ])
        
        length = compute_path_length(odom_msgs)
        assert 2.99 < length < 3.01  # 3 meters
    
    def test_path_length_square(self):
        """Path length should be correct for square path."""
        odom_msgs = self.create_mock_odom_msgs([
            (0.0, 0.0),
            (1.0, 0.0),
            (1.0, 1.0),
            (0.0, 1.0),
            (0.0, 0.0)
        ])
        
        length = compute_path_length(odom_msgs)
        assert 3.99 < length < 4.01  # 4 meters (perimeter of unit square)
    
    def test_path_length_empty(self):
        """Path length should be 0 for empty odometry."""
        length = compute_path_length([])
        assert length == 0.0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])

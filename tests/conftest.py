"""
Test configuration and fixtures for pytest.

This file is automatically loaded by pytest and provides:
- Common fixtures
- Test markers
- Configuration options
"""

import pytest
import sys
from pathlib import Path

# Add project root to Python path
PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))


# ============================================================================
# Pytest Configuration
# ============================================================================

def pytest_configure(config):
    """Register custom markers."""
    config.addinivalue_line(
        "markers", "slow: marks tests as slow (deselect with '-m \"not slow\"')"
    )
    config.addinivalue_line(
        "markers", "integration: marks tests as integration tests"
    )
    config.addinivalue_line(
        "markers", "gui: marks tests that require GUI/display"
    )


# ============================================================================
# Common Fixtures
# ============================================================================

@pytest.fixture(scope="session")
def project_root():
    """Return the project root directory."""
    return PROJECT_ROOT


@pytest.fixture(scope="session")
def test_data_dir(project_root):
    """Return the test data directory."""
    test_dir = project_root / "tests" / "data"
    test_dir.mkdir(exist_ok=True)
    return test_dir


@pytest.fixture
def sample_config():
    """Return a sample configuration for testing."""
    return {
        'slam': 'gmapping',
        'dataset': 'tb3_sim_explore_modeA',
        'seed': 0,
        'timeout': 120,
        'paths': {
            'logs_dir': '/tmp/test_logs',
            'bags_dir': '/tmp/test_bags',
            'metrics_json': '/tmp/test_metrics.json'
        }
    }


@pytest.fixture
def sample_metrics():
    """Return sample metrics data for testing."""
    return {
        'ate_rmse': 0.15,
        'coverage': 0.85,
        'map_ssim': 0.92,
        'wall_thickness_m': 0.08,
        'max_cpu_percent': 65.5,
        'max_ram_mb': 512.3,
        'is_failure': False,
        'failure_reasons': []
    }


@pytest.fixture
def sample_ground_truth_map():
    """Return a sample ground truth map for testing."""
    import numpy as np
    
    # Create a simple 100x100 map
    gt_map = np.zeros((100, 100), dtype=np.int8)
    
    # Add some walls (occupied cells)
    gt_map[0, :] = 100  # Top wall
    gt_map[-1, :] = 100  # Bottom wall
    gt_map[:, 0] = 100  # Left wall
    gt_map[:, -1] = 100  # Right wall
    gt_map[50, 20:80] = 100  # Middle horizontal wall
    
    return gt_map


@pytest.fixture
def sample_estimated_map():
    """Return a sample estimated map for testing."""
    import numpy as np
    
    # Create a map similar to GT but with some differences
    est_map = np.zeros((100, 100), dtype=np.int8)
    
    # Add walls (slightly different from GT)
    est_map[0:2, :] = 100  # Thicker top wall
    est_map[-2:, :] = 100  # Thicker bottom wall
    est_map[:, 0:2] = 100  # Thicker left wall
    est_map[:, -2:] = 100  # Thicker right wall
    est_map[49:52, 20:80] = 100  # Thicker middle wall
    
    # Add some unknown cells
    est_map[10:20, 10:20] = -1
    
    return est_map


# ============================================================================
# Cleanup Fixtures
# ============================================================================

@pytest.fixture(autouse=True)
def cleanup_temp_files(tmp_path):
    """Automatically cleanup temporary files after each test."""
    yield
    # Cleanup happens automatically with tmp_path


# ============================================================================
# Skip Conditions
# ============================================================================

def pytest_collection_modifyitems(config, items):
    """Modify test collection to add skip markers."""
    import subprocess
    
    # Check if Docker is available
    docker_available = subprocess.run(
        ["which", "docker"],
        capture_output=True
    ).returncode == 0
    
    # Check if display is available (for GUI tests)
    import os
    display_available = 'DISPLAY' in os.environ
    
    for item in items:
        # Skip Docker tests if Docker is not available
        if "docker" in item.nodeid.lower() and not docker_available:
            item.add_marker(pytest.mark.skip(reason="Docker not available"))
        
        # Skip GUI tests if no display
        if "gui" in item.keywords and not display_available:
            item.add_marker(pytest.mark.skip(reason="No display available"))

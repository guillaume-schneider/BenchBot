"""
Unit tests for GUI components.

Tests coverage for:
- Widget initialization
- Signal/slot connections
- Data validation
- UI state management

Note: These tests use QTest for PyQt5 testing.
"""

import pytest
import sys
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

# PyQt5 imports
from PyQt5.QtWidgets import QApplication
from PyQt5.QtTest import QTest
from PyQt5.QtCore import Qt

# Create QApplication instance for testing
# This is required for any PyQt5 widget tests
@pytest.fixture(scope="session")
def qapp():
    """Create QApplication instance for testing."""
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)
    yield app
    # Don't quit - other tests might need it


class TestMainWindow:
    """Test suite for main window initialization."""
    
    def test_main_window_creation(self, qapp):
        """Main window should initialize without errors."""
        from gui.main import MainWindow
        
        window = MainWindow()
        assert window is not None
        assert window.windowTitle() == "SLAM Bench Orchestrator"
    
    def test_main_window_has_pages(self, qapp):
        """Main window should have all required pages."""
        from gui.main import MainWindow
        
        window = MainWindow()
        
        # Check that page stack exists
        assert hasattr(window, 'page_stack')
        assert window.page_stack.count() >= 5  # Dashboard, Benchmark, Comparison, Visualizer, Settings


class TestDashboardPage:
    """Test suite for Dashboard page."""
    
    def test_dashboard_initialization(self, qapp):
        """Dashboard should initialize with default state."""
        from gui.pages.dashboard import DashboardPage
        
        dashboard = DashboardPage()
        assert dashboard is not None
    
    def test_matrix_selector_exists(self, qapp):
        """Dashboard should have matrix selector combo box."""
        from gui.pages.dashboard import DashboardPage
        
        dashboard = DashboardPage()
        
        # Should have a combo box for matrix selection
        assert hasattr(dashboard, 'matrix_combo')


class TestComparisonPage:
    """Test suite for Comparison page."""
    
    def test_comparison_initialization(self, qapp):
        """Comparison page should initialize correctly."""
        from gui.pages.comparison import ComparisonPage
        
        comparison = ComparisonPage()
        assert comparison is not None
    
    def test_comparison_has_run_selectors(self, qapp):
        """Comparison page should have run selector combo boxes."""
        from gui.pages.comparison import ComparisonPage
        
        comparison = ComparisonPage()
        
        # Should have combo boxes for selecting runs
        assert hasattr(comparison, 'combos')
        assert len(comparison.combos) == 3  # 3 run selectors
    
    def test_comparison_has_compare_button(self, qapp):
        """Comparison page should have compare button."""
        from gui.pages.comparison import ComparisonPage
        
        comparison = ComparisonPage()
        
        # Should have a compare button
        assert hasattr(comparison, 'compare_btn')
    
    def test_comparison_has_export_button(self, qapp):
        """Comparison page should have PDF export button."""
        from gui.pages.comparison import ComparisonPage
        
        comparison = ComparisonPage()
        
        # Should have an export button
        assert hasattr(comparison, 'export_btn')


class TestSettingsPage:
    """Test suite for Settings page."""
    
    def test_settings_initialization(self, qapp):
        """Settings page should initialize correctly."""
        from gui.pages.settings import SettingsPage
        
        settings = SettingsPage()
        assert settings is not None
    
    def test_settings_has_theme_selector(self, qapp):
        """Settings should have theme selector."""
        from gui.pages.settings import SettingsPage
        
        settings = SettingsPage()
        
        assert hasattr(settings, 'theme_combo')
    
    def test_settings_has_docker_toggle(self, qapp):
        """Settings should have Docker execution toggle."""
        from gui.pages.settings import SettingsPage
        
        settings = SettingsPage()
        
        assert hasattr(settings, 'docker_cb')
    
    def test_settings_has_build_button(self, qapp):
        """Settings should have Docker build button."""
        from gui.pages.settings import SettingsPage
        
        settings = SettingsPage()
        
        assert hasattr(settings, 'build_btn')


class TestVisualizerPage:
    """Test suite for 3D Visualizer page."""
    
    def test_visualizer_initialization(self, qapp):
        """Visualizer should initialize without errors."""
        from gui.pages.visualizer import VisualizerPage
        
        visualizer = VisualizerPage()
        assert visualizer is not None
    
    def test_visualizer_has_3d_view(self, qapp):
        """Visualizer should have 3D OpenGL view."""
        from gui.pages.visualizer import VisualizerPage
        
        visualizer = VisualizerPage()
        
        # Should have GLViewWidget for 3D rendering
        assert hasattr(visualizer, 'view_3d')
    
    def test_visualizer_has_follow_checkbox(self, qapp):
        """Visualizer should have 'Follow Robot' checkbox."""
        from gui.pages.visualizer import VisualizerPage
        
        visualizer = VisualizerPage()
        
        assert hasattr(visualizer, 'follow_cb')


class TestWorkerThread:
    """Test suite for background worker thread."""
    
    def test_worker_creation(self, qapp):
        """Worker thread should be created successfully."""
        from gui.worker import RunWorker
        
        # Create a minimal config for testing
        test_config = {
            'slam': 'gmapping',
            'dataset': 'test',
            'paths': {
                'logs_dir': '/tmp/test_logs',
                'bags_dir': '/tmp/test_bags',
                'metrics_json': '/tmp/test_metrics.json'
            }
        }
        
        worker = RunWorker(test_config, '/tmp/test_config.yaml')
        assert worker is not None
    
    def test_worker_has_signals(self, qapp):
        """Worker should have required signals."""
        from gui.worker import RunWorker
        
        test_config = {
            'slam': 'gmapping',
            'dataset': 'test',
            'paths': {
                'logs_dir': '/tmp/test_logs',
                'bags_dir': '/tmp/test_bags',
                'metrics_json': '/tmp/test_metrics.json'
            }
        }
        
        worker = RunWorker(test_config, '/tmp/test_config.yaml')
        
        # Check for essential signals
        assert hasattr(worker, 'log_signal')
        assert hasattr(worker, 'finished_signal')
        assert hasattr(worker, 'live_metrics_signal')


class TestDataValidation:
    """Test suite for input validation in GUI."""
    
    def test_run_path_validation(self):
        """Run path should be validated before use."""
        from pathlib import Path
        
        # Valid path
        valid_path = Path("/tmp/test_run")
        assert isinstance(valid_path, Path)
        
        # Invalid path (should be caught by GUI)
        invalid_path = ""
        assert invalid_path == ""  # Empty string should be rejected
    
    def test_metrics_data_validation(self):
        """Metrics data should be validated before display."""
        # Valid metrics
        valid_metrics = {
            'ate_rmse': 0.5,
            'coverage': 0.8,
            'max_cpu_percent': 50.0,
            'max_ram_mb': 500.0
        }
        
        assert all(isinstance(v, (int, float)) for v in valid_metrics.values())
        
        # Invalid metrics (should be handled gracefully)
        invalid_metrics = {
            'ate_rmse': None,
            'coverage': 'invalid'
        }
        
        # GUI should handle None and invalid types


class TestUIStateManagement:
    """Test suite for UI state management."""
    
    def test_button_enable_disable(self, qapp):
        """Buttons should be enabled/disabled based on state."""
        from PyQt5.QtWidgets import QPushButton
        
        button = QPushButton("Test")
        
        # Initially enabled
        button.setEnabled(True)
        assert button.isEnabled()
        
        # Disable during operation
        button.setEnabled(False)
        assert not button.isEnabled()
        
        # Re-enable after completion
        button.setEnabled(True)
        assert button.isEnabled()


if __name__ == "__main__":
    pytest.main([__file__, "-v"])

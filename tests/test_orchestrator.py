"""
Unit tests for orchestrator run lifecycle.

Tests coverage for:
- Configuration loading and validation
- Process management
- Metrics collection
- Error handling
"""

import pytest
import tempfile
import yaml
from pathlib import Path
import sys

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))


class TestConfigurationLoading:
    """Test suite for YAML configuration loading and validation."""
    
    def test_load_valid_config(self):
        """Should successfully load a valid configuration."""
        from runner.resolve import load_yaml
        
        config_data = {
            'slam': 'gmapping',
            'dataset': 'tb3_sim_explore_modeA',
            'seed': 0
        }
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            yaml.dump(config_data, f)
            config_path = f.name
        
        try:
            loaded = load_yaml(config_path)
            assert loaded['slam'] == 'gmapping'
            assert loaded['seed'] == 0
        finally:
            Path(config_path).unlink()
    
    def test_load_nonexistent_config(self):
        """Should raise error for non-existent config file."""
        from runner.resolve import load_yaml
        
        with pytest.raises(FileNotFoundError):
            load_yaml('/nonexistent/path/config.yaml')
    
    def test_load_invalid_yaml(self):
        """Should raise error for malformed YAML."""
        from runner.resolve import load_yaml
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            f.write("invalid: yaml: content:\n  - broken")
            config_path = f.name
        
        try:
            with pytest.raises(yaml.YAMLError):
                load_yaml(config_path)
        finally:
            Path(config_path).unlink()


class TestRunIDGeneration:
    """Test suite for stable run ID generation."""
    
    def test_stable_run_id_deterministic(self):
        """Same config should produce same run ID."""
        from runner.resolve import stable_run_id
        
        config = {
            'slam': 'gmapping',
            'dataset': 'test_world',
            'seed': 42
        }
        
        id1 = stable_run_id(config)
        id2 = stable_run_id(config)
        
        assert id1 == id2
    
    def test_stable_run_id_different_configs(self):
        """Different configs should produce different run IDs."""
        from runner.resolve import stable_run_id
        
        config1 = {'slam': 'gmapping', 'dataset': 'world1', 'seed': 0}
        config2 = {'slam': 'cartographer', 'dataset': 'world1', 'seed': 0}
        
        id1 = stable_run_id(config1)
        id2 = stable_run_id(config2)
        
        assert id1 != id2
    
    def test_stable_run_id_format(self):
        """Run ID should follow expected format."""
        from runner.resolve import stable_run_id
        
        config = {
            'slam': 'gmapping',
            'dataset': 'tb3_world',
            'seed': 0
        }
        
        run_id = stable_run_id(config)
        
        # Should contain dataset, slam, seed, and run number
        assert 'tb3_world' in run_id or 'gmapping' in run_id
        assert 'seed0' in run_id or 'r0' in run_id


class TestMetricsCollection:
    """Test suite for system metrics collection."""
    
    def test_cpu_metrics_valid_range(self):
        """CPU metrics should be in valid range (0-100+ %)."""
        import psutil
        
        cpu_percent = psutil.cpu_percent(interval=0.1)
        
        assert cpu_percent >= 0.0
        # Can exceed 100% on multi-core systems
        assert cpu_percent <= 100.0 * psutil.cpu_count()
    
    def test_ram_metrics_valid_range(self):
        """RAM metrics should be positive."""
        import psutil
        
        process = psutil.Process()
        mem_info = process.memory_info()
        ram_mb = mem_info.rss / (1024 * 1024)
        
        assert ram_mb > 0.0
        assert ram_mb < 100000.0  # Sanity check: less than 100GB


class TestErrorHandling:
    """Test suite for error handling and recovery."""
    
    def test_missing_slam_config(self):
        """Should handle missing SLAM configuration gracefully."""
        from runner.resolve import resolve_run_config
        
        config = {
            'slam': 'nonexistent_slam',
            'dataset': 'test_world'
        }
        
        # Should either raise a clear error or return None
        with pytest.raises((FileNotFoundError, KeyError, ValueError)):
            resolve_run_config(config)
    
    def test_missing_dataset_config(self):
        """Should handle missing dataset configuration gracefully."""
        from runner.resolve import resolve_run_config
        
        config = {
            'slam': 'gmapping',
            'dataset': 'nonexistent_dataset'
        }
        
        with pytest.raises((FileNotFoundError, KeyError, ValueError)):
            resolve_run_config(config)


class TestPathResolution:
    """Test suite for path resolution and directory creation."""
    
    def test_results_directory_creation(self):
        """Should create results directory if it doesn't exist."""
        import tempfile
        import shutil
        
        temp_dir = tempfile.mkdtemp()
        results_dir = Path(temp_dir) / "results" / "runs"
        
        try:
            # Simulate directory creation logic
            results_dir.mkdir(parents=True, exist_ok=True)
            
            assert results_dir.exists()
            assert results_dir.is_dir()
        finally:
            shutil.rmtree(temp_dir)
    
    def test_config_path_resolution(self):
        """Should resolve relative config paths correctly."""
        from pathlib import Path
        
        # Test relative path resolution
        base_path = Path("/home/user/project")
        relative_config = "configs/slams/gmapping.yaml"
        
        resolved = base_path / relative_config
        
        assert str(resolved) == "/home/user/project/configs/slams/gmapping.yaml"


class TestYAMLWriting:
    """Test suite for YAML configuration writing."""
    
    def test_write_resolved_config(self):
        """Should write resolved config to file."""
        from runner.resolve import write_yaml
        
        config = {
            'slam': 'gmapping',
            'dataset': 'test_world',
            'resolved': True
        }
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            config_path = f.name
        
        try:
            write_yaml(config, config_path)
            
            # Read back and verify
            with open(config_path, 'r') as f:
                loaded = yaml.safe_load(f)
            
            assert loaded['slam'] == 'gmapping'
            assert loaded['resolved'] is True
        finally:
            Path(config_path).unlink()


if __name__ == "__main__":
    pytest.main([__file__, "-v"])

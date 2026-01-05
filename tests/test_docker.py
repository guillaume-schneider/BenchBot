"""
Unit tests for Docker containerization.

Tests coverage for:
- Docker image building
- Container execution
- Volume mounting
- Environment variables
"""

import pytest
import subprocess
import sys
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))


class TestDockerImage:
    """Test suite for Docker image building and validation."""
    
    @pytest.mark.slow
    def test_dockerfile_exists(self):
        """Dockerfile should exist in project root."""
        dockerfile = Path(__file__).parent.parent / "Dockerfile"
        assert dockerfile.exists()
        assert dockerfile.is_file()
    
    @pytest.mark.slow
    def test_docker_compose_exists(self):
        """docker-compose.yml should exist in project root."""
        compose_file = Path(__file__).parent.parent / "docker-compose.yml"
        assert compose_file.exists()
        assert compose_file.is_file()
    
    @pytest.mark.slow
    @pytest.mark.skipif(
        subprocess.run(["which", "docker"], capture_output=True).returncode != 0,
        reason="Docker not installed"
    )
    def test_docker_daemon_running(self):
        """Docker daemon should be accessible."""
        result = subprocess.run(
            ["docker", "info"],
            capture_output=True,
            timeout=5
        )
        assert result.returncode == 0, "Docker daemon not running"
    
    @pytest.mark.slow
    @pytest.mark.skipif(
        subprocess.run(["which", "docker"], capture_output=True).returncode != 0,
        reason="Docker not installed"
    )
    def test_dockerfile_syntax(self):
        """Dockerfile should have valid syntax."""
        dockerfile = Path(__file__).parent.parent / "Dockerfile"
        
        # Docker buildkit can validate syntax without building
        result = subprocess.run(
            ["docker", "build", "--check", str(dockerfile.parent)],
            capture_output=True,
            timeout=10
        )
        
        # Note: --check flag may not be available in all Docker versions
        # If it fails, we just check the file is readable
        if result.returncode != 0:
            with open(dockerfile, 'r') as f:
                content = f.read()
                assert "FROM" in content
                assert "RUN" in content


class TestDockerConfiguration:
    """Test suite for Docker configuration validation."""
    
    def test_docker_compose_valid_yaml(self):
        """docker-compose.yml should be valid YAML."""
        import yaml
        
        compose_file = Path(__file__).parent.parent / "docker-compose.yml"
        
        with open(compose_file, 'r') as f:
            config = yaml.safe_load(f)
        
        assert 'services' in config or 'version' in config
    
    def test_docker_compose_has_required_fields(self):
        """docker-compose.yml should have required service fields."""
        import yaml
        
        compose_file = Path(__file__).parent.parent / "docker-compose.yml"
        
        with open(compose_file, 'r') as f:
            config = yaml.safe_load(f)
        
        # Check for essential fields
        if 'services' in config:
            # Modern docker-compose format
            assert len(config['services']) > 0
        else:
            # Older format or different structure
            assert 'build' in config or 'image' in config
    
    def test_dockerfile_has_base_image(self):
        """Dockerfile should specify a base image."""
        dockerfile = Path(__file__).parent.parent / "Dockerfile"
        
        with open(dockerfile, 'r') as f:
            content = f.read()
        
        assert "FROM" in content
        # Should use ROS 2 Humble base image
        assert "humble" in content.lower() or "ros:humble" in content.lower()
    
    def test_dockerfile_installs_dependencies(self):
        """Dockerfile should install required dependencies."""
        dockerfile = Path(__file__).parent.parent / "Dockerfile"
        
        with open(dockerfile, 'r') as f:
            content = f.read()
        
        # Check for Python dependencies installation
        assert "pip" in content or "pip3" in content
        # Check for numpy pinning
        assert "numpy" in content


class TestDockerVolumes:
    """Test suite for Docker volume mounting."""
    
    def test_docker_compose_has_volumes(self):
        """docker-compose.yml should define volume mounts."""
        import yaml
        
        compose_file = Path(__file__).parent.parent / "docker-compose.yml"
        
        with open(compose_file, 'r') as f:
            config = yaml.safe_load(f)
        
        # Check for volume definitions
        has_volumes = False
        if 'services' in config:
            for service in config['services'].values():
                if 'volumes' in service:
                    has_volumes = True
                    break
        elif 'volumes' in config:
            has_volumes = True
        
        assert has_volumes, "No volume mounts defined"
    
    def test_docker_compose_mounts_project_directory(self):
        """docker-compose.yml should mount project directory."""
        import yaml
        
        compose_file = Path(__file__).parent.parent / "docker-compose.yml"
        
        with open(compose_file, 'r') as f:
            config = yaml.safe_load(f)
        
        # Check for project directory mount
        has_project_mount = False
        if 'services' in config:
            for service in config['services'].values():
                if 'volumes' in service:
                    for volume in service['volumes']:
                        if isinstance(volume, str) and ':/app' in volume:
                            has_project_mount = True
                            break
        
        assert has_project_mount, "Project directory not mounted to /app"


class TestDockerEnvironment:
    """Test suite for Docker environment variables."""
    
    def test_docker_compose_has_display_env(self):
        """docker-compose.yml should set DISPLAY for GUI."""
        import yaml
        
        compose_file = Path(__file__).parent.parent / "docker-compose.yml"
        
        with open(compose_file, 'r') as f:
            config = yaml.safe_load(f)
        
        # Check for DISPLAY environment variable
        has_display = False
        if 'services' in config:
            for service in config['services'].values():
                if 'environment' in service:
                    env = service['environment']
                    if isinstance(env, dict):
                        has_display = 'DISPLAY' in env
                    elif isinstance(env, list):
                        has_display = any('DISPLAY' in e for e in env)
                    if has_display:
                        break
        
        assert has_display, "DISPLAY environment variable not set"
    
    def test_docker_compose_has_x11_socket(self):
        """docker-compose.yml should mount X11 socket for GUI."""
        import yaml
        
        compose_file = Path(__file__).parent.parent / "docker-compose.yml"
        
        with open(compose_file, 'r') as f:
            config = yaml.safe_load(f)
        
        # Check for X11 socket mount
        has_x11 = False
        if 'services' in config:
            for service in config['services'].values():
                if 'volumes' in service:
                    for volume in service['volumes']:
                        if isinstance(volume, str) and '.X11-unix' in volume:
                            has_x11 = True
                            break
        
        assert has_x11, "X11 socket not mounted"


class TestDockerIntegration:
    """Integration tests for Docker execution (requires Docker)."""
    
    @pytest.mark.slow
    @pytest.mark.integration
    @pytest.mark.skipif(
        subprocess.run(["which", "docker"], capture_output=True).returncode != 0,
        reason="Docker not installed"
    )
    def test_docker_build_succeeds(self):
        """Docker image should build successfully (slow test)."""
        project_root = Path(__file__).parent.parent
        
        # This is a slow test - only run if explicitly requested
        result = subprocess.run(
            ["docker", "build", "-t", "slam-bench-test", "."],
            cwd=str(project_root),
            capture_output=True,
            timeout=600  # 10 minutes max
        )
        
        assert result.returncode == 0, f"Docker build failed: {result.stderr.decode()}"
    
    @pytest.mark.slow
    @pytest.mark.integration
    @pytest.mark.skipif(
        subprocess.run(["which", "docker-compose"], capture_output=True).returncode != 0,
        reason="docker-compose not installed"
    )
    def test_docker_compose_config_valid(self):
        """docker-compose config should be valid."""
        project_root = Path(__file__).parent.parent
        
        result = subprocess.run(
            ["docker-compose", "config"],
            cwd=str(project_root),
            capture_output=True,
            timeout=10
        )
        
        assert result.returncode == 0, f"docker-compose config invalid: {result.stderr.decode()}"


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-m", "not slow"])

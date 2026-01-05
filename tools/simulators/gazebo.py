"""Gazebo simulator implementation"""

import subprocess
import shutil
from pathlib import Path
from typing import Dict, Any, Optional

from .base import BaseSimulator


class GazeboSimulator(BaseSimulator):
    """Gazebo Classic / Ignition simulator"""
    
    def __init__(self):
        super().__init__("gazebo")
    
    def is_installed(self) -> bool:
        """Check if Gazebo is available"""
        return shutil.which("gzserver") is not None
    
    def install(self, progress_callback=None) -> bool:
        """Gazebo installation via apt (system package)"""
        if progress_callback:
            progress_callback("Gazebo is a system package. Please install manually:", 10)
            progress_callback("sudo apt install ros-${ROS_DISTRO}-gazebo-ros-pkgs", 50)
        
        # We don't auto-install system packages for security
        return False
    
    def verify_dependencies(self) -> Dict[str, bool]:
        """Check Gazebo dependencies"""
        deps = {}
        deps['gzserver'] = shutil.which("gzserver") is not None
        deps['gzclient'] = shutil.which("gzclient") is not None
        deps['ros_gazebo'] = self._check_ros_package("gazebo_ros")
        return deps
    
    def _check_ros_package(self, package: str) -> bool:
        """Check if ROS package is available"""
        try:
            result = subprocess.run(
                ["ros2", "pkg", "prefix", package],
                capture_output=True,
                timeout=5
            )
            return result.returncode == 0
        except Exception:
            return False
    
    def start(self, world_config: Dict[str, Any]) -> subprocess.Popen:
        """Start Gazebo - handled by scenario launch, not directly"""
        # Gazebo is started via ros2 launch in the scenario processes
        # This method is not used directly
        raise NotImplementedError("Gazebo is started via ros2 launch in scenario")
    
    def stop(self, process: subprocess.Popen) -> None:
        """Stop Gazebo"""
        if process and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
    
    def cleanup(self) -> None:
        """Kill all Gazebo processes"""
        try:
            subprocess.run(["pkill", "-9", "-f", "gzserver"], 
                         stderr=subprocess.DEVNULL, timeout=2)
            subprocess.run(["pkill", "-9", "-f", "gzclient"], 
                         stderr=subprocess.DEVNULL, timeout=2)
        except Exception:
            pass
    
    def get_install_size_mb(self) -> int:
        return 500  # Approximate
    
    def get_version(self) -> Optional[str]:
        """Get Gazebo version"""
        try:
            result = subprocess.run(
                ["gzserver", "--version"],
                capture_output=True,
                text=True,
                timeout=2
            )
            if result.returncode == 0:
                # Parse version from output
                for line in result.stdout.split('\n'):
                    if 'Gazebo' in line:
                        return line.strip()
        except Exception:
            pass
        return None

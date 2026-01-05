"""Base simulator interface"""

from abc import ABC, abstractmethod
from pathlib import Path
from typing import Optional, Dict, Any
import subprocess


class BaseSimulator(ABC):
    """Abstract base class for simulator implementations"""
    
    def __init__(self, name: str):
        self.name = name
        self.install_dir = Path.home() / ".slam_bench" / name
        self.install_dir.mkdir(parents=True, exist_ok=True)
    
    @abstractmethod
    def is_installed(self) -> bool:
        """Check if simulator is installed"""
        pass
    
    @abstractmethod
    def install(self, progress_callback=None) -> bool:
        """Install the simulator
        
        Args:
            progress_callback: Optional callback(message: str, percent: int)
            
        Returns:
            True if installation successful
        """
        pass
    
    @abstractmethod
    def verify_dependencies(self) -> Dict[str, bool]:
        """Verify all dependencies are met
        
        Returns:
            Dict mapping dependency name to availability
        """
        pass
    
    @abstractmethod
    def start(self, world_config: Dict[str, Any]) -> subprocess.Popen:
        """Start the simulator
        
        Args:
            world_config: Configuration dict with world/robot settings
            
        Returns:
            Running process
        """
        pass
    
    @abstractmethod
    def stop(self, process: subprocess.Popen) -> None:
        """Stop the simulator gracefully"""
        pass
    
    @abstractmethod
    def cleanup(self) -> None:
        """Kill any leftover processes"""
        pass
    
    def get_install_size_mb(self) -> int:
        """Get estimated installation size in MB"""
        return 0
    
    def get_version(self) -> Optional[str]:
        """Get installed version"""
        return None

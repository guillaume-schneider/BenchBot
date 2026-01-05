"""Simulator Manager - Central manager for all simulators"""

from typing import Dict, Optional
from pathlib import Path

from .simulators import BaseSimulator, GazeboSimulator, O3DESimulator


class SimulatorManager:
    """Manages multiple simulator implementations"""
    
    def __init__(self):
        self.simulators: Dict[str, BaseSimulator] = {
            'gazebo': GazeboSimulator(),
            'o3de': O3DESimulator()
        }
        self._active_processes = {}
    
    def get_simulator(self, name: str) -> Optional[BaseSimulator]:
        """Get simulator by name"""
        return self.simulators.get(name)
    
    def list_simulators(self) -> Dict[str, Dict[str, any]]:
        """Get info about all simulators"""
        info = {}
        for name, sim in self.simulators.items():
            info[name] = {
                'installed': sim.is_installed(),
                'version': sim.get_version(),
                'install_size_mb': sim.get_install_size_mb(),
                'dependencies': sim.verify_dependencies()
            }
        return info
    
    def ensure_installed(self, sim_name: str, progress_callback=None) -> bool:
        """Ensure simulator is installed, install if needed
        
        Args:
            sim_name: Name of simulator ('gazebo' or 'o3de')
            progress_callback: Optional callback(message, percent)
            
        Returns:
            True if simulator is ready to use
        """
        sim = self.get_simulator(sim_name)
        if not sim:
            if progress_callback:
                progress_callback(f"Unknown simulator: {sim_name}", 0)
            return False
        
        if sim.is_installed():
            if progress_callback:
                progress_callback(f"{sim_name} already installed", 100)
            return True
        
        # Try to install
        return sim.install(progress_callback)
    
    def cleanup_all(self):
        """Cleanup all simulator processes"""
        for sim in self.simulators.values():
            try:
                sim.cleanup()
            except Exception:
                pass

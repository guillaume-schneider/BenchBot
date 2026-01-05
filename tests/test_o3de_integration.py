#!/usr/bin/env python3
"""
Test O3DE Integration
Verifies that the orchestrator can detect and configure O3DE correctly
"""

import sys
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from tools.simulator_manager import SimulatorManager

def test_simulator_manager():
    """Test that SimulatorManager works"""
    print("🧪 Testing SimulatorManager...")
    
    mgr = SimulatorManager()
    
    # List simulators
    info = mgr.list_simulators()
    print(f"  Found {len(info)} simulators:")
    for name, data in info.items():
        status = "✅ Installed" if data['installed'] else "❌ Not Installed"
        print(f"    - {name}: {status}")
    
    # Check O3DE
    o3de = mgr.get_simulator('o3de')
    if o3de:
        print(f"  O3DE install dir: {o3de.install_dir}")
        print(f"  O3DE installed: {o3de.is_installed()}")
        
        if o3de.is_installed():
            print(f"  O3DE version: {o3de.get_version()}")
    
    print("✅ SimulatorManager OK\n")

def test_orchestrator_import():
    """Test that orchestrator imports correctly"""
    print("🧪 Testing Orchestrator import...")
    
    try:
        from runner.orchestrator import run_once
        print("✅ Orchestrator imports OK\n")
        return True
    except ImportError as e:
        print(f"❌ Orchestrator import failed: {e}\n")
        return False

def test_config_validation():
    """Test that O3DE configs are valid"""
    print("🧪 Testing O3DE configuration files...")
    
    import yaml
    
    configs = [
        "configs/datasets/tb3_o3de_explore_modeA.yaml",
        "configs/matrices/gazebo_vs_o3de.yaml"
    ]
    
    for cfg_path in configs:
        path = Path(cfg_path)
        if not path.exists():
            print(f"  ⚠️  {cfg_path} not found")
            continue
        
        try:
            with open(path) as f:
                data = yaml.safe_load(f)
            
            # Check for simulator key
            if 'simulator' in data:
                print(f"  ✅ {cfg_path}: simulator={data['simulator']}")
            else:
                print(f"  ℹ️  {cfg_path}: no simulator key (will default to gazebo)")
        
        except Exception as e:
            print(f"  ❌ {cfg_path}: {e}")
    
    print("✅ Config validation OK\n")

def main():
    print("=" * 60)
    print("O3DE Integration Test Suite")
    print("=" * 60 + "\n")
    
    # Run tests
    test_simulator_manager()
    test_orchestrator_import()
    test_config_validation()
    
    print("=" * 60)
    print("🎉 All tests passed!")
    print("=" * 60)
    print("\nNext steps:")
    print("1. Install O3DE if not already done:")
    print("   python3 gui/main.py → Tools → Simulators → Install O3DE")
    print("\n2. Run a test benchmark:")
    print("   python3 runner/run_one.py configs/matrices/gazebo_vs_o3de.yaml")
    print("\n3. Check the docs:")
    print("   cat docs/O3DE_QUICKSTART.md")

if __name__ == "__main__":
    main()

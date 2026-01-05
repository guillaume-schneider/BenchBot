#!/usr/bin/env python3
"""Quick test script for O3DE headless launch"""

import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from tools.simulators.o3de import O3DESimulator

def test_o3de_launch():
    """Test O3DE headless launch"""
    
    print("=" * 60)
    print("O3DE Headless Launch Test")
    print("=" * 60)
    
    # Initialize simulator
    sim = O3DESimulator()
    
    # Check installation
    if not sim.is_installed():
        print("❌ O3DE is not installed!")
        return False
    
    print("✅ O3DE is installed")
    
    # Check project exists
    project_path = sim.projects_dir / "model_o3de_project"
    if not project_path.exists():
        print(f"❌ Project not found: {project_path}")
        return False
    
    print(f"✅ Project found: {project_path}")
    
    # Check GameLauncher exists
    game_launcher = project_path / "build" / "bin" / "profile" / "model_o3de_project.GameLauncher"
    if not game_launcher.exists():
        print(f"❌ GameLauncher not found: {game_launcher}")
        return False
    
    print(f"✅ GameLauncher found: {game_launcher}")
    
    # Test configuration
    world_config = {
        'project_path': str(project_path),
        'level': 'slam_world',
        'headless': True,
        'env': {
            'ROS_DOMAIN_ID': '0'
        }
    }
    
    print("\n📋 Configuration:")
    print(f"  Project: {world_config['project_path']}")
    print(f"  Level: {world_config['level']}")
    print(f"  Headless: {world_config['headless']}")
    
    print("\n⚠️  This will attempt to launch O3DE in headless mode.")
    print("⚠️  The Asset Processor will start and wait 30 seconds.")
    print("⚠️  Press Ctrl+C to cancel or Enter to continue...")
    
    try:
        input()
    except KeyboardInterrupt:
        print("\n❌ Test cancelled by user")
        return False
    
    print("\n🚀 Starting O3DE...")
    
    try:
        # Start O3DE
        process = sim.start(world_config)
        
        print(f"✅ O3DE process started (PID: {process.pid})")
        
        print("\n⏳ Waiting 10 seconds to check if process is stable...")
        import time
        time.sleep(10)
        
        # Check if still running
        if process.poll() is None:
            print("✅ Process is still running - SUCCESS!")
            
            print("\n📊 Process status:")
            print(f"  Main process PID: {process.pid}")
            if sim.asset_processor_process:
                print(f"  Asset Processor PID: {sim.asset_processor_process.pid}")
            
            print("\n🛑 Stopping O3DE...")
            sim.stop(process)
            
            print("✅ Test completed successfully!")
            return True
        else:
            return_code = process.poll()
            print(f"❌ Process exited with code: {return_code}")
            return False
            
    except Exception as e:
        print(f"❌ Error during test: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        print("\n🧹 Cleaning up...")
        sim.cleanup()
        print("✅ Cleanup complete")

if __name__ == "__main__":
    success = test_o3de_launch()
    sys.exit(0 if success else 1)

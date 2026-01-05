"""O3DE (Open 3D Engine) simulator implementation"""

import subprocess
import shutil
import json
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, Any, Optional, List, Tuple

from .base import BaseSimulator


class O3DESimulator(BaseSimulator):
    """Open 3D Engine simulator with ROS2 support"""
    
    def __init__(self):
        super().__init__("o3de")
        self.projects_dir = self.install_dir / "projects"
        self.projects_dir.mkdir(parents=True, exist_ok=True)
        self.o3de_repo_url = "https://github.com/o3de/o3de.git"
        self.ros2_gem_url = "https://github.com/o3de/o3de-extras.git"
        self.asset_processor_process = None  # Track Asset Processor
    
    def is_installed(self) -> bool:
        """Check if O3DE is installed"""
        o3de_script = self.install_dir / "scripts" / "o3de.sh"
        return o3de_script.exists() and o3de_script.is_file()
    
    def install(self, progress_callback=None) -> bool:
        """Install O3DE and ROS2 Gem
        
        This is a long process (~30-60 min):
        1. Clone O3DE repo
        2. Clone O3DE Extras (for ROS2 Gem)
        3. Build O3DE
        4. Register ROS2 Gem
        """
        try:
            # Check if installation directory exists but is incomplete
            if self.install_dir.exists() and not self.is_installed():
                if progress_callback:
                    progress_callback("⚠️ Cleaning incomplete installation...", 1)
                import shutil
                shutil.rmtree(self.install_dir)
                self.install_dir.mkdir(parents=True, exist_ok=True)
            
            if progress_callback:
                progress_callback("📦 Cloning O3DE repository...", 5)
            
            # 1. Clone O3DE
            if not (self.install_dir / ".git").exists():
                subprocess.run([
                    "git", "clone", "--branch", "main", "--depth", "1",
                    self.o3de_repo_url, str(self.install_dir)
                ], check=True)
            
            if progress_callback:
                progress_callback("📦 Cloning ROS2 Gem...", 15)
            
            # 2. Clone O3DE Extras for ROS2 Gem
            extras_dir = self.install_dir.parent / "o3de-extras"
            if not (extras_dir / ".git").exists():
                subprocess.run([
                    "git", "clone", "--branch", "development",
                    self.ros2_gem_url, str(extras_dir)
                ], check=True)
            
            if progress_callback:
                progress_callback("🔧 Setting up O3DE Python environment...", 25)
            
            # 3. Setup Python environment (using the shell script)
            subprocess.run([
                "bash", str(self.install_dir / "python" / "get_python.sh")
            ], check=True, cwd=str(self.install_dir))
            
            if progress_callback:
                progress_callback("🏗️ Configuring O3DE build (this may take a while)...", 35)
            
            # 4. Configure O3DE with CMake
            build_dir = self.install_dir / "build" / "linux"
            build_dir.mkdir(parents=True, exist_ok=True)
            
            # Use o3de.sh to register the engine
            subprocess.run([
                str(self.install_dir / "scripts" / "o3de.sh"),
                "register", "--this-engine"
            ], check=True, cwd=str(self.install_dir))
            
            if progress_callback:
                progress_callback("🔨 Building O3DE (20-40 minutes, be patient)...", 40)
            
            # Build using CMake (this is the longest step)
            subprocess.run([
                "cmake", "-B", str(build_dir), "-S", str(self.install_dir),
                "-G", "Ninja Multi-Config",
                "-DLY_UNITY_BUILD=ON"  # Speed up compilation
            ], check=True)
            
            subprocess.run([
                "cmake", "--build", str(build_dir),
                "--config", "profile",
                "--target", "Editor",
                "--", "-j", "4"  # Use 4 parallel jobs
            ], check=True)
            
            if progress_callback:
                progress_callback("🔌 Registering ROS2 Gem...", 90)
            
            # 5. Register ROS2 Gem
            subprocess.run([
                str(self.install_dir / "scripts" / "o3de.sh"),
                "register",
                "--gem-path", str(extras_dir / "Gems" / "ROS2")
            ], check=True)
            
            if progress_callback:
                progress_callback("✅ O3DE installation complete!", 100)
            
            return True
            
        except subprocess.CalledProcessError as e:
            if progress_callback:
                progress_callback(f"❌ Installation failed: {e}", 0)
            return False
        except Exception as e:
            if progress_callback:
                progress_callback(f"❌ Unexpected error: {e}", 0)
            return False
    
    def verify_dependencies(self) -> Dict[str, bool]:
        """Check O3DE dependencies"""
        deps = {}
        deps['git'] = shutil.which("git") is not None
        deps['cmake'] = shutil.which("cmake") is not None
        deps['ninja'] = shutil.which("ninja") is not None
        deps['python3'] = shutil.which("python3") is not None
        deps['clang'] = shutil.which("clang++") is not None or shutil.which("g++") is not None
        # Don't include 'o3de_installed' here - it's circular
        return deps
    
    def create_project_from_sdf(self, sdf_path: Path, project_name: str, 
                               progress_callback=None) -> Path:
        """Convert Gazebo SDF world to O3DE project"""
        project_path = self.projects_dir / project_name
        
        if progress_callback:
            progress_callback(f"Creating O3DE project: {project_name}", 10)
        
        # 1. Create project if doesn't exist
        if not project_path.exists():
            subprocess.run([
                str(self.install_dir / "scripts" / "o3de.sh"),
                "create-project",
                "--project-path", str(project_path),
                "--project-name", project_name
            ], check=True)
            
            if progress_callback:
                progress_callback("🔌 Enabling required Gems...", 70)
            
            # Enable LevelGeoreferencing first (required by ROS2)
            try:
                subprocess.run([
                    str(self.install_dir / "scripts" / "o3de.sh"),
                    "enable-gem", "--gem-name", "LevelGeoreferencing",
                    "--project-path", str(project_path)
                ], check=True, cwd=str(self.install_dir))
            except subprocess.CalledProcessError:
                # LevelGeoreferencing might not exist, try with --force on ROS2
                pass
            
            # Enable ROS2 Gem for the project (use --force if needed)
            try:
                subprocess.run([
                    str(self.install_dir / "scripts" / "o3de.sh"),
                    "enable-gem", "--gem-name", "ROS2",
                    "--project-path", str(project_path)
                ], check=True, cwd=str(self.install_dir))
            except subprocess.CalledProcessError:
                # If it fails, try with --force to bypass dependency checks
                subprocess.run([
                    str(self.install_dir / "scripts" / "o3de.sh"),
                    "enable-gem", "--gem-name", "ROS2",
                    "--project-path", str(project_path),
                    "--force"
                ], check=True, cwd=str(self.install_dir))
        
        if progress_callback:
            progress_callback("Parsing SDF world...", 30)
        
        # 2. Parse SDF
        world_data = self._parse_sdf(sdf_path)
        
        if progress_callback:
            progress_callback("Generating O3DE level...", 60)
        
        # 3. Generate O3DE level
        level_path = project_path / "Levels" / "slam_world.prefab"
        self._generate_o3de_level(world_data, level_path)
        
        if progress_callback:
            progress_callback("Project ready!", 100)
        
        return project_path
    
    def _parse_sdf(self, sdf_path: Path) -> dict:
        """Parse SDF file and extract geometry data"""
        import xml.etree.ElementTree as ET
        
        tree = ET.parse(sdf_path)
        root = tree.getroot()
        
        # Check if this is a world or a model
        world_elem = root.find('.//world')
        model_elem = root.find('.//model')
        
        if world_elem is not None:
            # It's a world file
            world_name = world_elem.get('name', 'default')
            models = world_elem.findall('.//model')
        elif model_elem is not None:
            # It's a model file (like model.sdf)
            world_name = model_elem.get('name', 'default')
            models = [model_elem]  # Treat the single model as a list
        else:
            # No world or model found, create empty
            world_name = 'default'
            models = []
        
        world_data = {
            'name': world_name,
            'models': [],
            'lights': [],
            'physics': {}
        }
        
        # Parse models
        for model in models:
            model_data = {
                'name': model.get('name'),
                'static': model.get('static', 'false') == 'true',
                'pose': self._parse_pose(model.find('pose')),
                'links': []
            }
            
            # Parse links and visuals
            for link in model.findall('.//link'):
                for visual in link.findall('visual'):
                    geom = visual.find('.//geometry')
                    if geom is not None:
                        link_data = {
                            'name': visual.get('name', 'visual'),
                            'pose': self._parse_pose(visual.find('pose')),
                            'geometry': self._parse_geometry(geom)
                        }
                        model_data['links'].append(link_data)
            
            world_data['models'].append(model_data)
        
        return world_data
    
    def _parse_pose(self, pose_elem) -> List[float]:
        """Parse SDF pose element"""
        if pose_elem is None or pose_elem.text is None:
            return [0, 0, 0, 0, 0, 0]
        return [float(x) for x in pose_elem.text.strip().split()]
    
    def _parse_geometry(self, geom_elem) -> Dict[str, Any]:
        """Parse SDF geometry element"""
        for child in geom_elem:
            if child.tag == 'box':
                size = child.find('size')
                if size is not None:
                    dims = [float(x) for x in size.text.strip().split()]
                    return {'type': 'box', 'dimensions': dims}
            
            elif child.tag == 'cylinder':
                radius = child.find('radius')
                length = child.find('length')
                return {
                    'type': 'cylinder',
                    'radius': float(radius.text) if radius is not None else 0.5,
                    'length': float(length.text) if length is not None else 1.0
                }
            
            elif child.tag == 'sphere':
                radius = child.find('radius')
                return {
                    'type': 'sphere',
                    'radius': float(radius.text) if radius is not None else 0.5
                }
            
            elif child.tag == 'mesh':
                uri = child.find('uri')
                return {
                    'type': 'mesh',
                    'uri': uri.text if uri is not None else ''
                }
        
        return {'type': 'unknown'}
    
    def _generate_o3de_level(self, world_data: Dict[str, Any], level_path: Path):
        """Generate O3DE level from parsed world data"""
        # O3DE uses JSON-based prefabs
        level_data = {
            "ContainerEntity": {
                "Id": "RootEntity",
                "Name": world_data['name'],
                "Components": {},
                "Entities": []
            }
        }
        
        # Add each model as an entity
        for i, model in enumerate(world_data['models']):
            entity = {
                "Id": f"Entity_{i}",
                "Name": model['name'],
                "Components": {
                    "Transform": {
                        "$type": "AzFramework::TransformComponent",
                        "Position": model['pose'][:3],
                        "Rotation": model['pose'][3:]
                    }
                }
            }
            
            # Add shape components for each link
            for j, link in enumerate(model['links']):
                geom = link['geometry']
                
                if geom['type'] == 'box':
                    entity['Components'][f'BoxShape_{j}'] = {
                        "$type": "LmbrCentral::BoxShapeComponent",
                        "Dimensions": geom['dimensions']
                    }
                elif geom['type'] == 'cylinder':
                    entity['Components'][f'CylinderShape_{j}'] = {
                        "$type": "LmbrCentral::CylinderShapeComponent",
                        "Radius": geom['radius'],
                        "Height": geom['length']
                    }
            
            level_data["ContainerEntity"]["Entities"].append(entity)
        
        # Save level
        level_path.parent.mkdir(parents=True, exist_ok=True)
        with open(level_path, 'w') as f:
            json.dump(level_data, f, indent=2)
    
    def start(self, world_config: Dict[str, Any]) -> subprocess.Popen:
        """Start O3DE with project and Asset Processor"""
        import time
        import logging
        
        project_path = Path(world_config.get('project_path'))
        level_name = world_config.get('level', 'slam_world')
        headless = world_config.get('headless', True)  # Default to headless for benchmarking
        
        logger = logging.getLogger(__name__)
        
        # 1. Start Asset Processor in background
        asset_processor_exe = self.install_dir / "build" / "linux" / "bin" / "profile" / "AssetProcessor"
        if asset_processor_exe.exists():
            logger.info("Starting Asset Processor in background...")
            try:
                self.asset_processor_process = subprocess.Popen(
                    [
                        str(asset_processor_exe),
                        "--zeroAnalysisMode",
                        f"--project-path={project_path}"
                    ],
                    cwd=str(self.install_dir),  # Run from O3DE root, not build dir
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )
                logger.info(f"Asset Processor started (PID: {self.asset_processor_process.pid})")
                
                # Wait for Asset Processor to initialize
                logger.info("Waiting 30s for critical assets to be ready...")
                time.sleep(30)
            except Exception as e:
                logger.warning(f"Failed to start Asset Processor: {e}")
        else:
            logger.warning(f"Asset Processor not found at {asset_processor_exe}")
        
        # 2. Determine executable path (use GameLauncher for projects, Editor for development)
        project_name = project_path.name
        game_launcher = project_path / "build" / "bin" / "profile" / f"{project_name}.GameLauncher"
        
        if game_launcher.exists():
            exe_path = game_launcher
            logger.info(f"Using GameLauncher: {exe_path}")
        else:
            exe_path = self.install_dir / "build" / "linux" / "bin" / "profile" / "Editor"
            logger.info(f"Using Editor (GameLauncher not found): {exe_path}")
        
        # 3. Build command
        cmd = [
            str(exe_path),
            f"--project-path={project_path}",
            f"--level={level_name}"
        ]
        
        # 4. Add headless flags for benchmarking
        if headless:
            cmd.extend([
                "--rhi=null",  # No graphics rendering
                "--regset=/Amazon/AzCore/Bootstrap/wait_for_connect=0"  # Don't wait for debugger
            ])
            logger.info("Launching in HEADLESS mode (no graphics)")
        else:
            logger.info("Launching with GUI")
        
        # 5. Setup environment
        env = world_config.get('env', {})
        merged_env = subprocess.os.environ.copy()
        merged_env.update(env)
        
        logger.info(f"Starting O3DE: {' '.join(cmd)}")
        return subprocess.Popen(cmd, env=merged_env)
    
    def stop(self, process: subprocess.Popen) -> None:
        """Stop O3DE and Asset Processor"""
        import logging
        logger = logging.getLogger(__name__)
        
        # Stop main process
        if process and process.poll() is None:
            logger.info("Stopping O3DE process...")
            process.terminate()
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                logger.warning("O3DE process did not terminate, forcing kill...")
                process.kill()
        
        # Stop Asset Processor
        if self.asset_processor_process and self.asset_processor_process.poll() is None:
            logger.info("Stopping Asset Processor...")
            self.asset_processor_process.terminate()
            try:
                self.asset_processor_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                logger.warning("Asset Processor did not terminate, forcing kill...")
                self.asset_processor_process.kill()
            self.asset_processor_process = None
    
    def cleanup(self) -> None:
        """Kill all O3DE processes"""
        try:
            subprocess.run(["pkill", "-9", "-f", "o3de"],
                         stderr=subprocess.DEVNULL, timeout=2)
            subprocess.run(["pkill", "-9", "-f", "Editor"],
                         stderr=subprocess.DEVNULL, timeout=2)
            subprocess.run(["pkill", "-9", "-f", "AssetProcessor"],
                         stderr=subprocess.DEVNULL, timeout=2)
            subprocess.run(["pkill", "-9", "-f", "GameLauncher"],
                         stderr=subprocess.DEVNULL, timeout=2)
        except Exception:
            pass
        finally:
            self.asset_processor_process = None
    
    def get_install_size_mb(self) -> int:
        return 15000  # ~15GB with dependencies
    
    def get_version(self) -> Optional[str]:
        """Get O3DE version"""
        engine_json = self.install_dir / "engine.json"
        if engine_json.exists():
            try:
                with open(engine_json) as f:
                    data = json.load(f)
                    return data.get('o3de_version', 'Unknown')
            except Exception:
                pass
        return None

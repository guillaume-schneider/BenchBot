import os
import subprocess
import sys
from pathlib import Path

class DependencyManager:
    def __init__(self, log_callback=None):
        self.log_callback = log_callback

    def log(self, message):
        if self.log_callback:
            self.log_callback(message)
        print(f"[DEP_MGR] {message}")

    def ensure_dependencies(self, dependencies_list):
        """
        Processes a list of dependency dictionaries.
        Example:
        {
            'name': 'gmapping',
            'git': 'url',
            'branch': 'master',
            'path': 'deps/gmapping_ws',
            'build': 'colcon build',
            'source': 'install/setup.bash'
        }
        """
        if not dependencies_list:
            return True

        for dep in dependencies_list:
            name = dep.get('name', 'unknown')
            self.log(f"Checking dependency: {name}")

            path = dep.get('path')
            if not path:
                self.log(f"Error: No path specified for {name}")
                return False

            abs_path = Path(path).resolve()
            
            # 1. Clone if missing
            if not abs_path.exists():
                git_url = dep.get('git')
                if not git_url:
                    self.log(f"Error: {name} path missing and no git URL provided.")
                    return False

                branch = dep.get('branch')
                cmd = ["git", "clone"]
                if branch:
                    cmd += ["-b", branch]
                cmd += [git_url, str(abs_path)]

                self.log(f"Cloning {name} into {path}...")
                try:
                    subprocess.run(cmd, check=True)
                except subprocess.CalledProcessError as e:
                    self.log(f"Failed to clone {name}: {e}")
                    return False

            # 2. Build if required
            build_cmd = dep.get('build')
            if build_cmd:
                # Check if already built (check for 'source' file)
                source_file = dep.get('source')
                if source_file and (abs_path / source_file).exists():
                    self.log(f"{name} is already built. Skipping build step.")
                else:
                    self.log(f"Building {name}...")
                    # We might need to source ROS before building
                    # For simplicity, we assume the environment is set up or colcon finds it
                    try:
                        # Split build_cmd to support arguments
                        subprocess.run(build_cmd, shell=True, cwd=str(abs_path), check=True)
                    except subprocess.CalledProcessError as e:
                        self.log(f"Build failed for {name}: {e}")
                        return False

        return True

    def get_source_commands(self, dependencies_list):
        """Returns a list of setup.bash paths to source."""
        cmds = []
        if not dependencies_list:
            return cmds

        for dep in dependencies_list:
            path = dep.get('path')
            source_file = dep.get('source')
            if path and source_file:
                abs_source = (Path(path) / source_file).resolve()
                if abs_source.exists():
                    cmds.append(str(abs_source))
        return cmds

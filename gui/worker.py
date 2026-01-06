from PyQt5.QtCore import QThread, pyqtSignal, QSettings
import subprocess
import sys
import os
import signal
import selectors
from pathlib import Path
from runner.resolve import load_yaml, resolve_run_config, stable_run_id, write_yaml
from gt_map.generator import generate_map

PROJECT_ROOT = Path(__file__).parent.parent

class RunWorker(QThread):
    log_signal = pyqtSignal(str) # Log message
    progress_signal = pyqtSignal(int, int, str)
    finished_signal = pyqtSignal()
    result_ready = pyqtSignal(str) # Path
    
    # New signals for config tracking
    config_started = pyqtSignal(str) # config_path
    config_finished = pyqtSignal(str) # config_path
    live_metrics_signal = pyqtSignal(str, dict) # config_path, {cpu: float, ram: float}

    def __init__(self, configs_paths, use_gui, options=None):
        super().__init__()
        self.configs_paths = configs_paths
        self.use_gui = use_gui
        self.options = options or {}
        self.is_cancelled = False
        self.current_process = None

    def run(self):
        print("DEBUG: RunWorker.run() started")
        try:
            total_resolved_jobs = []
            print(f"DEBUG: Processing {len(self.configs_paths)} configs.")
            
            # 1. Resolve all jobs first
            for matrix_path in self.configs_paths:
                print(f"DEBUG: Processing matrix: {matrix_path}")
                self.config_started.emit(str(matrix_path))
                try:
                    print("DEBUG: Loading yaml...")
                    matrix = load_yaml(matrix_path)
                    print("DEBUG: Yaml loaded.")
                    
                    # Auto-GT Logic
                    print("DEBUG: Starting Auto-GT check...")
                    for ds in matrix.get("datasets", []):
                        wm_path = ds.get("world_model")
                        print(f"DEBUG: Checking dataset {ds.get('id')}, wm: {wm_path}")
                        if wm_path:
                            # Resolve paths
                            wm_path_obj = Path(wm_path)
                            if not wm_path_obj.is_absolute():
                                wm_path_obj = PROJECT_ROOT / wm_path
                                
                            gt_dir = PROJECT_ROOT / "maps" / "gt"
                            gt_dir.mkdir(parents=True, exist_ok=True)
                            
                            gt_name = wm_path_obj.stem
                            print(f"DEBUG: GT Name: {gt_name}")
                            gt_base = gt_dir / gt_name
                            gt_yaml = gt_base.with_suffix(".yaml")
                            print(f"DEBUG: GT path: {gt_yaml}, Exists: {gt_yaml.exists()}")
                            
                            if not gt_yaml.exists():
                                print("DEBUG: GT missing, invoking generator...")
                                self.log_signal.emit(f"Generating GT Map for {gt_name}...")
                                try:
                                    success, msg = generate_map(
                                        sdf_path=str(wm_path_obj),
                                        resolution=0.05,
                                        laser_z=0.2, # Standard height
                                        padding=2.0,
                                        output_name=str(gt_base),
                                        gen_png=True,
                                        gen_debug=False
                                    )
                                    if success:
                                        print("DEBUG: GT Generation SUCCESS")
                                        self.log_signal.emit(f"GT Generated: {gt_yaml}")
                                    else:
                                        print(f"DEBUG: GT Generation FAILED: {msg}")
                                        self.log_signal.emit(f"GT Generation Failed: {msg}")
                                except Exception as e:
                                    print(f"DEBUG: GT Generation EXCEPTION: {e}")
                                    import traceback
                                    traceback.print_exc()
                                    self.log_signal.emit(f"GT Gen Error: {e}")
                            else:
                                print("DEBUG: GT exists, skipping generation.")
                                self.log_signal.emit(f"Using existing GT Map: {gt_yaml.name}")

                            # Inject into dataset for this run
                            if gt_yaml.exists():
                                print(f"DEBUG: Injecting GT path into dataset: {gt_yaml}")
                                ds["ground_truth"] = {"map_path": str(gt_yaml.relative_to(PROJECT_ROOT))}
                            
                    print("DEBUG: Auto-GT check finished. Starting Job Resolution...")

                    output_root = matrix.get("output", {}).get("root_dir", "results/runs")
                    Path(output_root).mkdir(parents=True, exist_ok=True)
                    slams_map = {s["id"]: s for s in matrix.get("slams", [])}
                    datasets_map = {d["id"]: d for d in matrix.get("datasets", [])}
                    
                    for inc in matrix.get("matrix", {}).get("include", []):
                        d_id = inc["dataset"]
                        print(f"DEBUG: Resolving include for dataset {d_id}")
                        dataset_def = datasets_map.get(d_id)
                        if not dataset_def: 
                            print(f"DEBUG: Dataset {d_id} not found in map")
                            continue
                        
                        for s_id in inc.get("slams", []):
                            print(f"DEBUG: Resolving SLAM {s_id}")
                            slam_entry = slams_map.get(s_id)
                            if not slam_entry: 
                                print(f"DEBUG: SLAM {s_id} not found in map")
                                continue
                            
                            profile_path = PROJECT_ROOT / slam_entry["profile"]
                            print(f"DEBUG: Loading profile {profile_path}")
                            slam_profile = load_yaml(profile_path)
                            
                            for seed in inc.get("seeds", [0]):
                                for r in range(inc.get("repeats", 1)):
                                    run_id = stable_run_id(d_id, s_id, seed, r)
                                    print(f"DEBUG: Resolving run {run_id}")
                                    try:
                                        resolved = resolve_run_config(
                                            matrix=matrix, dataset_obj=dataset_def,
                                            slam_entry=slam_entry, slam_profile=slam_profile,
                                            combo_overrides=inc.get("overrides"),
                                            slam_overrides=slam_entry.get("overrides"),
                                            dataset_overrides=dataset_def.get("overrides"),
                                            seed=seed, repeat_index=r, run_id=run_id, output_root=output_root
                                        )
                                        
                                        # Inject Options
                                        print(f"DEBUG: Injecting options. self.options={self.options}")
                                        
                                        if self.options.get("use_gazebo"):
                                            print("DEBUG: Enabling Gazebo GUI (removing headless/gui:=False)")
                                            sc = resolved.get("dataset", {}).get("scenario", {})
                                            
                                            # Helper to replace in cmd list
                                            def replace_headless(cmd):
                                                def swap(s):
                                                    res = s.replace("headless:=True", "headless:=False")
                                                    res = res.replace("gui:=False", "gui:=True")
                                                    if res != s:
                                                        print(f"DEBUG: Swapped arg '{s}' -> '{res}'")
                                                    return res
                                                    
                                                if isinstance(cmd, list):
                                                    return [swap(c) for c in cmd]
                                                elif isinstance(cmd, str):
                                                    return swap(cmd)
                                                return cmd

                                            if "processes" in sc:
                                                for p in sc["processes"]:
                                                    if "cmd" in p: 
                                                        print(f"DEBUG: Checking process {p.get('name')} cmd: {p['cmd']}")
                                                        p["cmd"] = replace_headless(p["cmd"])
                                                        print(f"DEBUG: Modified process cmd: {p['cmd']}")
                                            elif "launch" in sc and "cmd" in sc["launch"]:
                                                sc["launch"]["cmd"] = replace_headless(sc["launch"]["cmd"])

                                        # Handle RViz (True OR False)
                                        use_rviz = self.options.get("use_rviz", False)
                                        target_arg = f"use_rviz:={use_rviz}"
                                        print(f"DEBUG: Enforcing RViz -> {target_arg}")
                                        
                                        sc = resolved.get("dataset", {}).get("scenario", {})
                                        
                                        def enforce_rviz(cmd):
                                            # Only apply use_rviz to 'ros2 launch' commands, not 'ros2 run'
                                            if isinstance(cmd, list):
                                                 # Check if this is a 'ros2 run' command
                                                 if len(cmd) >= 2 and cmd[0] == "ros2" and cmd[1] == "run":
                                                     print(f"DEBUG: Skipping use_rviz for 'ros2 run' command")
                                                     return cmd
                                                 
                                                 # Check for replace
                                                 for i, c in enumerate(cmd):
                                                     if "use_rviz:=" in c:
                                                         print(f"DEBUG: Replacing existing {c} with {target_arg}")
                                                         cmd[i] = target_arg
                                                         return cmd

                                                 # Not found, append (only for launch commands)
                                                 if len(cmd) >= 2 and cmd[0] == "ros2" and cmd[1] == "launch":
                                                     print(f"DEBUG: Appending {target_arg} to ros2 launch cmd list")
                                                     cmd.append(target_arg)

                                            elif isinstance(cmd, str):
                                                 # Skip for 'ros2 run' string commands
                                                 if "ros2 run" in cmd:
                                                     print(f"DEBUG: Skipping use_rviz for 'ros2 run' string command")
                                                     return cmd
                                                     
                                                 if "use_rviz:=" in cmd:
                                                     # Regex replace would be better but simple string parsing usually sufficient for key:=val
                                                     import re
                                                     return re.sub(r"use_rviz:=(True|False)", target_arg, cmd)
                                                 
                                                 # Only append for launch commands
                                                 if "ros2 launch" in cmd:
                                                     print(f"DEBUG: Appending {target_arg} to ros2 launch cmd string")
                                                     return cmd + " " + target_arg
                                            return cmd

                                        if "processes" in sc:
                                            for p in sc["processes"]:
                                                if "cmd" in p: 
                                                    print(f"DEBUG: Checking process {p.get('name')} for RViz enforcement...")
                                                    p["cmd"] = enforce_rviz(p["cmd"])
                                        elif "launch" in sc and "cmd" in sc["launch"]:
                                            sc["launch"]["cmd"] = enforce_rviz(sc["launch"]["cmd"])
                                        
                                        print("DEBUG: Run resolved successfully and options injected")
                                    except Exception as re:
                                        print(f"DEBUG: Resolution failed for {run_id}: {re}")
                                        raise re
                                    
                                    

                                    # Legacy gui flag fallback
                                    if self.use_gui and not self.options:
                                        # Only if no options passed, maintain old behavior
                                        pass
                                                
                                    config_path = Path(output_root) / run_id / "config_resolved.yaml"
                                    total_resolved_jobs.append((run_id, config_path, resolved, str(matrix_path)))
                except Exception as e:
                    self.log_signal.emit(f"ERROR processing config {matrix_path}: {e}")
                    print(f"DEBUG: Error processing config {matrix_path}: {e}")

            # 2. Execute jobs
            print(f"DEBUG: Resolution complete. Total jobs: {len(total_resolved_jobs)}")
            total = len(total_resolved_jobs)
            for i, (run_id, config_path, resolved, origin_path) in enumerate(total_resolved_jobs):
                print(f"DEBUG: Starting job {i+1}/{total}: {run_id}")
                if self.is_cancelled: 
                    print("DEBUG: Cancelled before job start")
                    break
                
                self.progress_signal.emit(i + 1, total, run_id)
                self.log_signal.emit(f"INFO: Starting benchmark {run_id} ({i+1}/{total})...")
                
                # Write resolved config
                try:
                    config_path.parent.mkdir(parents=True, exist_ok=True)
                    write_yaml(config_path, resolved)
                except Exception as e:
                     self.log_signal.emit(f"ERROR writing config: {e}")
                     continue

                cmd = [sys.executable, "-u", "-m", "runner.run_one", str(config_path)]
                
                # Check for Docker execution
                settings = QSettings("SlamBench", "Orchestrator")
                if settings.value("run_in_docker", "false") == "true":
                    self.log_signal.emit("DOCKER: Wrapping run in container...")
                    # docker run -v .:/app -e DISPLAY=$DISPLAY ... slam-bench-orchestrator python3 runner/run_one.py ...
                    # We use relative config path because /app is mapped to PROJECT_ROOT
                    rel_config = config_path.relative_to(PROJECT_ROOT)
                    cmd = [
                        "docker", "run", "--rm",
                        "--network", "host",
                        "--ipc", "host",
                        "-v", f"{str(PROJECT_ROOT)}:/app",
                        "-e", f"DISPLAY={os.environ.get('DISPLAY', '')}",
                        "-e", "QT_X11_NO_MITSHM=1",
                        "slam-bench-orchestrator:latest",
                        "python3", "-u", "-m", "runner.run_one", str(rel_config)
                    ]

                # Start in a new process group to allow killing the entire tree
                try:
                    process = subprocess.Popen(
                        cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, 
                        cwd=str(PROJECT_ROOT), universal_newlines=True,
                        preexec_fn=os.setsid
                    )
                except Exception as pe:
                    self.log_signal.emit(f"Popen failed: {pe}")
                    continue
                    
                self.current_process = process
                
                # Reading output
                sel = selectors.DefaultSelector()
                sel.register(process.stdout, selectors.EVENT_READ)
                
                try:
                    while True:
                        if self.is_cancelled:
                            self.log_signal.emit("WARN: Cancellation requested. Terminating process group...")
                            if self.current_process:
                                os.killpg(os.getpgid(self.current_process.pid), signal.SIGTERM)
                            break
                        
                        events = sel.select(timeout=0.1)
                        if events:
                            line = process.stdout.readline()
                            if not line: # EOF
                                break
                            
                            line_str = line.strip()
                            if "[LIVE_METRICS]" in line_str:
                                try:
                                    import json
                                    data_str = line_str.split("[LIVE_METRICS]")[1].strip()
                                    metrics = json.loads(data_str)
                                    print(f"DEBUG: Received LIVE_METRICS: {metrics} for {origin_path}")
                                    self.live_metrics_signal.emit(origin_path, metrics)
                                except:
                                    pass
                            else:
                                self.log_signal.emit(line_str)
                        
                        if process.poll() is not None:
                            # Flush remaining
                            for line in process.stdout:
                                line_str = line.strip()
                                if "[LIVE_METRICS]" in line_str: continue
                                self.log_signal.emit(line_str)
                            break
                finally:
                    sel.unregister(process.stdout)
                    sel.close()
                
                process.wait()
                self.current_process = None
                
                if self.is_cancelled:
                    self.log_signal.emit(f"CANCELLED: {run_id}")
                    break
                
                if process.returncode == 0:
                    self.log_signal.emit(f"SUCCESS: {run_id} complete.")
                    run_dir = Path(output_root) / run_id
                    self.result_ready.emit(str(run_dir))
                else:
                    self.log_signal.emit(f"FAILURE: {run_id} failed with code {process.returncode}.")
            
            for matrix_path in self.configs_paths:
                self.config_finished.emit(str(matrix_path))

        except Exception as e:
            print(f"DEBUG: RunWorker Exception detected: {e}")
            import traceback
            traceback.print_exc()
            self.log_signal.emit(f"ERROR: {str(e)}")
            self.log_signal.emit(traceback.format_exc())
        finally:
            self.finished_signal.emit()

    def cancel(self):
        self.is_cancelled = True

from __future__ import annotations

import time
import json
import sys
from pathlib import Path
import yaml

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry

from runner.process_manager import ProcessManager
from runner.bag_recorder import RosbagConfig, build_rosbag_cmd
from runner.probes.ros_probes import (
    ProbeContext,
    TfAvailableProbe,
    TopicPublishTypedProbe,
    TopicHzTypedProbe,
    ServiceAvailableProbe,
    NodePresentProbe,
)


def load_run_config(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def ensure_dirs(cfg: dict) -> None:
    for k in ["run_dir", "logs_dir", "bags_dir"]:
        Path(cfg["paths"][k]).mkdir(parents=True, exist_ok=True)


def build_probe(p: dict):
    t = p["type"]
    if t == "tf_available":
        return TfAvailableProbe(p["from_frame"], p["to_frame"], p.get("timeout_s", 10))
    if t == "topic_publish":
        return TopicPublishTypedProbe(
            p["topic"], p["msg_type"], p.get("min_messages", 1), p.get("timeout_s", 10)
        )
    if t == "topic_hz":
        return TopicHzTypedProbe(
            p["topic"], p["msg_type"], p["min_hz"], p.get("window_s", 5), p.get("timeout_s", 20)
        )
    if t == "service_available":
        return ServiceAvailableProbe(p["service"], p["srv_type"], p.get("timeout_s", 10))
    if t == "node_present":
        return NodePresentProbe(p["node"], p.get("timeout_s", 10))
    raise ValueError(f"Unknown probe type: {t}")


def run_once(config_path: str) -> int:
    cfg = load_run_config(config_path)
    ensure_dirs(cfg)

    logs_dir = cfg["paths"]["logs_dir"]
    bags_dir = cfg["paths"]["bags_dir"]

    # O3DE simulator variables (declared at function scope for try/except access)
    o3de_simulator_instance = None
    o3de_world_config = None
    o3de_process_handle = None

    # ========== SIMULATOR DETECTION & SETUP ==========
    sim_type = cfg.get("dataset", {}).get("simulator", "gazebo")
    
    if sim_type == "o3de":
        print(f"[ORCHESTRATOR] Detected O3DE simulator, setting up...")
        
        # Import simulator manager
        try:
            from tools.simulator_manager import SimulatorManager
        except ImportError:
            print("[ERROR] SimulatorManager not found. O3DE support requires tools/simulator_manager.py")
            return 3
        
        sim_mgr = SimulatorManager()
        o3de_sim = sim_mgr.get_simulator('o3de')
        
        # Verify O3DE is installed
        if not o3de_sim.is_installed():
            print("[ERROR] O3DE is not installed!")
            print("Please install O3DE via GUI: Tools → Simulators → Install O3DE")
            return 3
        
        print(f"[ORCHESTRATOR] O3DE found at: {o3de_sim.install_dir}")
        
        # Get world SDF path (it's at dataset level, not scenario level)
        world_sdf = cfg.get("dataset", {}).get("world_model")
        if not world_sdf:
            print("[ERROR] No 'world_model' specified in dataset scenario")
            return 3
        
        world_sdf_path = Path(world_sdf)
        if not world_sdf_path.exists():
            print(f"[ERROR] World SDF not found: {world_sdf}")
            return 3
        
        # Convert SDF to O3DE project (cached)
        project_name = world_sdf_path.stem + "_o3de_project"
        print(f"[ORCHESTRATOR] Converting SDF world to O3DE project: {project_name}")
        
        try:
            project_path = o3de_sim.create_project_from_sdf(
                world_sdf_path,
                project_name,
                progress_callback=lambda msg, pct: print(f"  [{pct}%] {msg}")
            )
            print(f"[ORCHESTRATOR] O3DE project ready at: {project_path}")
        except Exception as e:
            print(f"[ERROR] Failed to convert SDF to O3DE: {e}")
            import traceback
            traceback.print_exc()
            return 3
        
        # Get O3DE world configuration
        world_config = {
            'project_path': str(project_path),
            'level': 'slam_world',
            'headless': True,  # Always headless for benchmarking
            'env': {'ROS_DOMAIN_ID': '0'}
        }
        
        # Store O3DE simulator reference for later cleanup
        o3de_simulator_instance = o3de_sim
        o3de_world_config = world_config
        
        # Get existing scenario processes
        scenario_processes = cfg.get("dataset", {}).get("scenario", {}).get("processes", [])
        
        # Create placeholder process that will be started via ProcessManager
        # The actual O3DE launch will be handled by starting the simulator
        o3de_process = {
            "name": "o3de_sim",
            "cmd": ["echo", "O3DE will be started via SimulatorManager"],  # Placeholder
            "env": {"ROS_DOMAIN_ID": "0"},
            "cwd": None,
            "_o3de_managed": True  # Mark this as special
        }
        
        # Insert O3DE as first process
        scenario_processes.insert(0, o3de_process)
        cfg["dataset"]["scenario"]["processes"] = scenario_processes
        
        print(f"[ORCHESTRATOR] O3DE simulator configured successfully")
    
    # Preventive cleanup: kill any leftover Gazebo/O3DE/Nav2 processes
    # This ensures a clean slate before starting
    print(f"[ORCHESTRATOR] Starting run_once for {config_path}...")
    sys.stdout.flush()

    # NOTE: Use specific process names to avoid killing this Python script!
    # Preventive cleanup: kill any leftover Gazebo/O3DE/Nav2/ROS processes
    # This ensures a clean slate before starting
    try:
        import subprocess
        targets = [
            "gzserver", "gzclient", "ruby",  # Gazebo
            "Editor", "GameLauncher", "AssetProcessor",  # O3DE
            "nav2_manager", "component_container", "component_container_isolated", "lifecycle_manager",  # Nav2
            "map_server", "amcl", "bt_navigator", "planner_server", "controller_server",
            "rviz2", "robot_state_publisher"
        ]
        
        # Kill command construction
        cmd = ["pkill", "-9", "-f"]
        
        for t in targets:
            subprocess.run(cmd + [t], stderr=subprocess.DEVNULL, timeout=2)
            
        time.sleep(2.0)  # Let the system fully clean up ports and resources
    except Exception:
        pass

    pm = ProcessManager(logs_dir=logs_dir)

    # Start ROS probe node
    rclpy.init()
    node = Node("slam_bench_probe_node")
    ctx = ProbeContext(node=node)

    # --------- Mode A additions: explore pause/resume ----------
    explore_resume_topic = "/explore/resume"
    explore_pub = node.create_publisher(Bool, explore_resume_topic, 10)

    # --------- Pose Tracking for Live Plotting ---------
    latest_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
    
    def odom_callback(msg: Odometry):
        nonlocal latest_pose
        latest_pose["x"] = msg.pose.pose.position.x
        latest_pose["y"] = msg.pose.pose.position.y
        # Basic yaw extraction if needed
        import math
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        latest_pose["yaw"] = math.atan2(siny_cosp, cosy_cosp)

    node.create_subscription(Odometry, "/odom", odom_callback, 10)

    def set_explore(enabled: bool) -> None:
        msg = Bool()
        msg.data = bool(enabled)
        explore_pub.publish(msg)
        # ensure the message is actually sent
        for _ in range(8):
            rclpy.spin_once(node, timeout_sec=0.05)

    # --------- Robot Degradation Logic ---------
    robot_settings_path = Path("configs/robot_settings.json")
    degradation_active = False
    if robot_settings_path.exists():
        try:
            with open(robot_settings_path) as f:
                rs = json.load(f)
                if rs.get("enabled", False):
                    degradation_active = True
                    degrader_cmd = [
                        "python3", "tools/sensor_degrader.py",
                        "--ros-args",
                        "-p", f"max_range:={rs.get('max_range', 10.0)}",
                        "-p", f"noise_std:={rs.get('noise_std', 0.0)}",
                        "-p", f"speed_scale:={rs.get('speed_scale', 1.0)}",
                        "-p", "enabled:=True"
                    ]
        except Exception as e:
            node.get_logger().warn(f"Failed to load robot settings: {e}")

    # --------- Scenario: multi-process support ----------
    scenario = cfg.get("dataset", {}).get("scenario", {}) or {}
    scenario_processes = scenario.get("processes", None)
    
    # Apply degradation to scenario processes if active
    if degradation_active:
        node.get_logger().info("Applying Robot/Sensor DEGRADATION for this run.")
        if not scenario_processes:
            # Handle legacy or default wrap
            pass # We'll handle it below when it's created
        else:
            for proc in scenario_processes:
                if "sim" in proc["name"].lower():
                    # Remap simulator outputs to _raw so degrader can intercept
                    proc["cmd"] += ["--ros-args", "--remap", "scan:=scan_raw", "--remap", "cmd_vel:=cmd_vel_raw"]
            
            scenario_processes.append({
                "name": "degrader",
                "cmd": degrader_cmd,
                "env": {},
                "cwd": str(Path.cwd())
            })

    # Backward compatible: old config had scenario.launch.cmd
    if not scenario_processes:
        launch = scenario.get("launch", {}) or {}
        cmd = launch.get("cmd", None)
        if not cmd:
            raise RuntimeError(
                "No scenario processes found. Expected dataset.scenario.processes "
                "or legacy dataset.scenario.launch.cmd."
            )
        scenario_processes = [{
            "name": "scenario",
            "cmd": cmd,
            "env": launch.get("env", {}) or {},
            "cwd": launch.get("cwd", None),
        }]

    # SLAM process (Mode A can be noop)
    slam_cmd = cfg.get("slam", {}).get("launch", {}).get("cmd", None)
    slam_env = cfg.get("slam", {}).get("launch", {}).get("env", {}) or {}
    slam_cwd = cfg.get("slam", {}).get("launch", {}).get("cwd", None)
    slam_id = (cfg.get("slam", {}) or {}).get("id", "")

    # rosbag config
    bag_cfg = cfg["recording"]["rosbag2"]
    rosbag_cfg = RosbagConfig(
        storage_id=bag_cfg.get("storage_id", "sqlite3"),
        compression=bag_cfg.get("compression", None),
        max_bag_size_mb=bag_cfg.get("max_bag_size_mb", None),
        include_hidden_topics=bag_cfg.get("include_hidden_topics", False),
        qos_overrides_path=bag_cfg.get("qos_overrides_path", None),
        topics=bag_cfg.get("topics", []),
    )

    bag_out_dir = str(Path(bags_dir) / "output")
    bag_cmd = build_rosbag_cmd(bag_out_dir, rosbag_cfg)

    warmup_s = float(cfg["run_control"]["warmup_s"])
    drain_s = float(cfg["run_control"]["drain_s"])
    timeout_s = float(cfg["run_control"]["timeout_s"])

    import threading
    import psutil
    from runner.dependency_manager import DependencyManager

    # Instantiate DependencyManager
    dep_mgr = DependencyManager(log_callback=lambda m: node.get_logger().info(f"[DEPENDENCIES] {m}"))

    # Collect dependencies from dataset and slam
    all_deps = []
    dataset_cfg = cfg.get("dataset", {})
    all_deps.extend(dataset_cfg.get("dependencies", []) or [])
    slam_cfg = cfg.get("slam", {})
    all_deps.extend(slam_cfg.get("dependencies", []) or [])

    # Ensure dependencies are met
    if all_deps:
        node.get_logger().info("Ensuring dependencies are met...")
        if not dep_mgr.ensure_dependencies(all_deps):
            node.get_logger().error("Failed to satisfy dependencies.")
            return 4
        node.get_logger().info("Dependencies satisfied.")

    # Get setup paths to source
    extra_sources = dep_mgr.get_source_commands(all_deps)
    
    def wrap_cmd_with_sourcing(cmd_input):
        """Wraps a command (string or list) with sourcing of extra dependencies."""
        if not extra_sources:
            return cmd_input
            
        # Convert list to string if needed
        if isinstance(cmd_input, list):
            cmd_str = " ".join(f'"{c}"' if " " in c else c for c in cmd_input)
        else:
            cmd_str = cmd_input
            
        sourcing = " && ".join([f"source {s}" for s in extra_sources])
        # Use bash -c to execute the command in the sourced environment
        return ["bash", "-c", f"{sourcing} && exec {cmd_str}"]

    # Start monitoring thread
    system_metrics = {"max_cpu": 0.0, "max_ram": 0.0}
    monitor_active = True
    
    def monitor_loop():
        # Wait for processes to spin up
        time.sleep(2.0)
        
        # Cache psutil.Process objects to maintain state for cpu_percent()
        # Key: PID, Value: psutil.Process
        proc_cache = {}
        
        while monitor_active:
            try:
                total_cpu = 0.0
                total_rss_bytes = 0
                
                # Identify all target PIDs (roots + children)
                target_pids = set()
                
                # 1. Collect Root PIDs from ProcessManager
                root_pids = []
                for name, managed_proc in list(pm.procs.items()):
                    if managed_proc.popen.poll() is None:
                        root_pids.append(managed_proc.popen.pid)
                
                # 2. Collect Root PID from O3DE if applicable
                if o3de_simulator_instance and o3de_process_handle:
                    root_pids.append(o3de_process_handle.pid)

                # 3. Expand to children (using temporary process objects if not in cache yet)
                # We need to be careful not to re-create objects if we already have them,
                # BUT we need to find children. psutil.Process.children() requires an instance.
                
                current_cycle_procs = []
                
                for pid in root_pids:
                    # Get or create process object for root
                    if pid not in proc_cache:
                        try:
                            proc_cache[pid] = psutil.Process(pid)
                            # First call to initialize timer (returns 0.0)
                            proc_cache[pid].cpu_percent(interval=None)
                        except (psutil.NoSuchProcess, psutil.AccessDenied):
                            continue
                    
                    root_proc = proc_cache[pid]
                    current_cycle_procs.append(root_proc)
                    
                    # Get children
                    try:
                        children = root_proc.children(recursive=True)
                        for child in children:
                            c_pid = child.pid
                            if c_pid not in proc_cache:
                                try:
                                    proc_cache[c_pid] = child
                                    # Initialize timer
                                    proc_cache[c_pid].cpu_percent(interval=None)
                                except (psutil.NoSuchProcess, psutil.AccessDenied):
                                    continue
                            current_cycle_procs.append(proc_cache[c_pid])
                    except (psutil.NoSuchProcess, psutil.AccessDenied):
                        pass

                # 4. Measure metrics on cached objects
                for p in current_cycle_procs:
                    try:
                        # cpu_percent is non-blocking with interval=None
                        # It returns usage since last call (on this object)
                        cpu = p.cpu_percent(interval=None)
                        rss = p.memory_info().rss
                        
                        total_cpu += cpu
                        total_rss_bytes += rss
                    except (psutil.NoSuchProcess, psutil.AccessDenied):
                        # Process died between discovery and measurement
                        # Remove from cache to be safe?
                        # Actually we can leave it, we re-discover roots next time
                        pass
                
                # Cleanup cache: remove PIDs that weren't in current_cycle_procs?
                # Optional, but good for long runs.
                # However, current_cycle_procs refinds them every time. 
                # Let's simple prune cache to pids in current_cycle_procs
                active_pids = {p.pid for p in current_cycle_procs}
                bad_pids = [pid for pid in proc_cache if pid not in active_pids]
                for pid in bad_pids:
                    del proc_cache[pid]

                total_ram_mb = total_rss_bytes / (1024 * 1024)
                
                # Live Reporting for GUI
                metrics_payload = {
                    'cpu': round(total_cpu, 1), 
                    'ram': round(total_ram_mb, 1),
                    'pose': latest_pose
                }
                print(f"[LIVE_METRICS] {json.dumps(metrics_payload)}")
                sys.stdout.flush()

                if total_cpu > system_metrics["max_cpu"]:
                    system_metrics["max_cpu"] = total_cpu
                if total_ram_mb > system_metrics["max_ram"]:
                    # Filter crazy spikes? No, valid reading.
                    system_metrics["max_ram"] = total_ram_mb
                    
            except Exception as e:
                # Don't crash the monitor
                print(f"Monitor error: {e}")
                pass
                
            time.sleep(1.0) # 1Hz sampling

    monitor_thread = threading.Thread(target=monitor_loop, daemon=True)
    monitor_thread.start()

    try:
        # START_SCENARIO (multi-process)
        for proc in scenario_processes:
            # Special handling for O3DE-managed process
            if proc.get("_o3de_managed"):
                print("[ORCHESTRATOR] Starting O3DE via SimulatorManager...")
                try:
                    o3de_process_handle = o3de_simulator_instance.start(o3de_world_config)
                    print(f"[O3DE] Process started (PID: {o3de_process_handle.pid})")
                    # Store the handle for later cleanup - wrap in ManagedProcess
                    from runner.process_manager import ManagedProcess
                    mp = ManagedProcess(
                        name="o3de_sim",
                        popen=o3de_process_handle,
                        log_path=pm.logs_dir / "o3de_sim.log"
                    )
                    pm.procs["o3de_sim"] = mp
                except Exception as e:
                    print(f"[ERROR] Failed to start O3DE: {e}")
                    import traceback
                    traceback.print_exc()
                    return 3
            else:
                pm.start(
                    proc["name"],
                    wrap_cmd_with_sourcing(proc["cmd"]),
                    env=proc.get("env", {}) or {},
                    cwd=proc.get("cwd", None),
                )

        # Pause exploration for stable probes/warmup
        set_explore(False)

        # START_SLAM (Mode A: may be noop or omitted)
        # If you want to allow "no slam", keep slam_cmd optional.
        if slam_cmd and slam_id != "noop":
            pm.start("slam", wrap_cmd_with_sourcing(slam_cmd), env=slam_env, cwd=slam_cwd)
        elif slam_cmd and slam_id == "noop":
            # still launch noop for logging consistency (optional)
            pm.start("slam", wrap_cmd_with_sourcing(slam_cmd), env=slam_env, cwd=slam_cwd)

        # START_ROSBAG
        if cfg["recording"]["enabled"]:
            pm.start("rosbag", wrap_cmd_with_sourcing(bag_cmd), env={}, cwd=None)

        # WAIT_READY (probes)
        for p in cfg["probes"]["ready"]:
            probe = build_probe(p)
            res = probe.run(ctx)
            if not res.ok:
                node.get_logger().error(f"[PROBE FAIL] {res.message}")
                return 2
            node.get_logger().info(f"[PROBE OK] {res.message}")

        # WARMUP (still paused)
        t0 = time.time()
        while time.time() - t0 < warmup_s:
            rclpy.spin_once(node, timeout_sec=0.1)

        # RUN: resume exploration now
        set_explore(True)

        t1 = time.time()
        while time.time() - t1 < timeout_s:
            # Health probes optional (light)
            for p in (cfg.get("probes", {}).get("health") or []):
                probe = build_probe(p)
                res = probe.run(ctx)
                if not res.ok:
                    node.get_logger().warn(f"[HEALTH WARN] {res.message}")
            rclpy.spin_once(node, timeout_sec=0.2)

        # DRAIN: pause and wait a bit so last map/TF can flush
        set_explore(False)
        t2 = time.time()
        
        # Stop resource monitoring
        monitor_active = False
        if monitor_thread:
            monitor_thread.join(timeout=2.0)
            
        return 0

    finally:
        monitor_active = False # Ensure loop stops
        
        # STOP (reverse-ish)
        try:
            set_explore(False)
        except Exception:
            pass

        try:
            pm.stop("rosbag")
        except Exception:
            pass

        try:
            pm.stop("slam")
        except Exception:
            pass

        # Stop scenario processes in reverse order (explore then nav2_sim, etc.)
        try:
            for proc in reversed(scenario_processes):
                # Special handling for O3DE cleanup
                if proc.get("_o3de_managed"):
                    if o3de_simulator_instance is not None and o3de_process_handle is not None:
                        print("[ORCHESTRATOR] Stopping O3DE via SimulatorManager...")
                        try:
                            o3de_simulator_instance.stop(o3de_process_handle)
                            o3de_simulator_instance.cleanup()
                        except Exception as e:
                            print(f"[WARN] O3DE cleanup error: {e}")
                else:
                    pm.stop(proc["name"])
        except Exception:
            pass
        
        # Ensure ALL processes are stopped (safety net)
        try:
            pm.stop_all()
        except Exception:
            pass

        # EVALUATE (after stop)
        try:
            node.get_logger().info("Starting post-run evaluation...")
            from tools.benchmark import run_benchmark
            
            bag_path = cfg["paths"]["bags_dir"] + "/output"
            metrics_path = cfg["paths"]["metrics_json"]
            
            rmse = run_benchmark(bag_path)
            
            # Additional analysis for anomalies and coverage
            from evaluation import read_messages_by_topic, detect_anomalies, load_gt_map, occupancy_arrays_from_msgs, compute_coverage
            
            msgs = read_messages_by_topic(bag_path, ["/odom", "/map"])
            odom_msgs = msgs.get("/odom", [])
            map_msgs = msgs.get("/map", [])
            
            warnings, is_failure = detect_anomalies(odom_msgs, rmse)
            
            coverage = 0.0
            # Try to get GT map for coverage
            try:
                # Find GT map from config (already loaded above as cfg)
                gt_path_rel = None
                for ds in cfg.get("datasets", []): # Matrix might have multiple, but cfg resolved has one
                    pass # Simplified: Use the one in the resolved cfg
                
                # In resolved cfg, it's usually directly in dataset:
                gt_def = cfg.get("dataset", {}).get("ground_truth", {})
                if gt_def:
                    gt_map, gt_res, gt_origin = load_gt_map(str(Path.cwd() / gt_def["map_path"]))
                    est_on_gt = occupancy_arrays_from_msgs(map_msgs, gt_map, gt_res, gt_origin)
                    coverage = compute_coverage(gt_map, est_on_gt)
            except Exception as e:
                node.get_logger().warn(f"Could not compute coverage: {e}")

            # Save to metrics.json
            metrics = {
                "ate_rmse": rmse,
                "max_cpu_percent": system_metrics["max_cpu"],
                "max_ram_mb": system_metrics["max_ram"],
                "coverage": float(coverage),
                "is_failure": is_failure,
                "failure_reasons": warnings
            }
            with open(metrics_path, "w") as f:
                json.dump(metrics, f, indent=4)
            
            status_text = "SUCCESS" if not is_failure else "POTENTIAL FAILURE"
            node.get_logger().info(f"Evaluation finished [{status_text}]. RMSE: {rmse}, Coverage: {coverage*100:.1f}%, CPU: {metrics['max_cpu_percent']:.1f}%, RAM: {metrics['max_ram_mb']:.1f}MB")
            if warnings:
                for w in warnings:
                    node.get_logger().warn(f" [!] {w}")
        except Exception as e:
            node.get_logger().error(f"Evaluation failed: {e}")

        node.destroy_node()
        rclpy.shutdown()
        
        # Final killall for stubborn processes (Gazebo/O3DE are notorious)
        # This is a nuclear option but necessary for reliability
        # NOTE: Use specific process names to avoid killing other Python scripts!
        try:
            import subprocess
            subprocess.run(["pkill", "-9", "gzserver"], stderr=subprocess.DEVNULL, timeout=2)
            subprocess.run(["pkill", "-9", "gzclient"], stderr=subprocess.DEVNULL, timeout=2)
            subprocess.run(["pkill", "-9", "Editor"], stderr=subprocess.DEVNULL, timeout=2)
            subprocess.run(["pkill", "-9", "GameLauncher"], stderr=subprocess.DEVNULL, timeout=2)
            subprocess.run(["pkill", "-9", "AssetProcessor"], stderr=subprocess.DEVNULL, timeout=2)
            time.sleep(2.0)  # Increased from 0.5s - let the system fully clean up
        except Exception:
            pass

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
import tf2_ros

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

from utils.logger import get_logger, LogContext, log_exceptions

# Setup logger for this module
logger = get_logger("orchestrator")


def recursive_substitute(obj, mapping):
    if isinstance(obj, dict):
        return {k: recursive_substitute(v, mapping) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [recursive_substitute(x, mapping) for x in obj]
    elif isinstance(obj, str):
        for k, v in mapping.items():
            obj = obj.replace(k, v)
        return obj
    else:
        return obj

def load_run_config(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f) or {}
    
    # Define substitutions
    # We assume the orchestrator is run from the project root
    mapping = {
        "${PROJECT_ROOT}": str(Path.cwd()),
        "${HOME}": str(Path.home())
    }
    
    return recursive_substitute(config, mapping)


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
    logger.info("=" * 80)
    logger.info("RUN_ONCE CALLED")
    logger.info(f"RUN_ONCE CALLED - Config: {config_path}")
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
            logger.error("SimulatorManager not found. O3DE support requires tools/simulator_manager.py")
            return 3
        
        sim_mgr = SimulatorManager()
        o3de_sim = sim_mgr.get_simulator('o3de')
        
        # Verify O3DE is installed
        if not o3de_sim.is_installed():
            logger.error("O3DE is not installed!")
            logger.error("Please install O3DE via GUI: Tools → Simulators → Install O3DE")
            return 3
        
        logger.info(f"O3DE found at: {o3de_sim.install_dir}")
        
        # Get world SDF path (it's at dataset level, not scenario level)
        world_sdf = cfg.get("dataset", {}).get("world_model")
        if not world_sdf:
            logger.error("No 'world_model' specified in dataset scenario")
            return 3
        
        world_sdf_path = Path(world_sdf)
        if not world_sdf_path.exists():
            logger.error(f"World SDF not found: {world_sdf}")
            return 3
        
        # Convert SDF to O3DE project (cached)
        project_name = world_sdf_path.stem + "_o3de_project"
        logger.info(f"Converting SDF world to O3DE project: {project_name}")
        
        try:
            project_path = o3de_sim.create_project_from_sdf(
                world_sdf_path,
                project_name,
                progress_callback=lambda msg, pct: logger.debug(f"[{pct}%] {msg}")
            )
            logger.info(f"O3DE project ready at: {project_path}")
        except Exception as e:
            logger.error(f"Failed to convert SDF to O3DE: {e}", exc_info=True)
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
        # 1. Kill ROS 2 daemon to clear discovery cache (crucial for clean re-registration)
        subprocess.run(["ros2", "daemon", "stop"], stderr=subprocess.DEVNULL, timeout=5)
        
        targets = [
            "gzserver", "gzclient", "ruby", "spawn_entity", # Gazebo
            "Editor", "GameLauncher", "AssetProcessor",  # O3DE
            "nav2_manager", "component_container", "component_container_isolated", "lifecycle_manager",  # Nav2
            "map_server", "amcl", "bt_navigator", "planner_server", "controller_server", "behavior_server",
            "smoother_server", "waypoint_follower", "velocity_smoother",
            "rviz2", "robot_state_publisher", "slam_gmapping", "sync_slam_toolbox_node", "explore_node"
        ]
        
        # Kill command construction
        cmd = ["pkill", "-9", "-f"]
        
        for t in targets:
            subprocess.run(cmd + [t], stderr=subprocess.DEVNULL, timeout=2)
            
        time.sleep(3.0)  # Increased: Let the system fully clean up ports, DDS and resources
    except Exception:
        pass

    pm = ProcessManager(logs_dir=logs_dir)

    # Start ROS probe node
    rclpy.init()
    
    # Enable sim time if specified in config
    use_sim_time = cfg.get("run_control", {}).get("use_sim_time", False)
    from rcl_interfaces.msg import ParameterDescriptor
    from rclpy.parameter import Parameter
    
    node = Node("slam_bench_probe_node", parameter_overrides=[
        Parameter("use_sim_time", Parameter.Type.BOOL, use_sim_time)
    ])
    ctx = ProbeContext(node=node)

    # --------- Mode A additions: explore pause/resume ----------
    explore_resume_topic = "/explore/resume"
    explore_pub = node.create_publisher(Bool, explore_resume_topic, 10)

    # --------- Pose Tracking for Live Plotting via TF ---------
    latest_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
    
    # Use TF for real position (map -> base_footprint)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)
    
    def update_pose_from_tf():
        nonlocal latest_pose
        try:
            # We look for map -> base_footprint which is the standard SLAM output
            # Use the node's clock which handles use_sim_time correctly
            now = node.get_clock().now()
            trans = tf_buffer.lookup_transform('map', 'base_footprint', now, timeout=rclpy.duration.Duration(seconds=0.1))
            
            latest_pose["x"] = trans.transform.translation.x
            latest_pose["y"] = trans.transform.translation.y
            
            q = trans.transform.rotation
            import math
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            latest_pose["yaw"] = math.atan2(siny_cosp, cosy_cosp)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # Fallback to odom if map is not yet available
            pass

    # Create a timer to poll TF regularly (10Hz)
    node.create_timer(0.1, update_pose_from_tf)

    def set_explore(enabled: bool) -> None:
        msg = Bool()
        msg.data = bool(enabled)
        explore_pub.publish(msg)
        # ensure the message is actually sent
        for _ in range(8):
            rclpy.spin_once(node, timeout_sec=0.05)

    # --------- Robot Degradation Logic ---------

    # --------- Robot Degradation Logic ---------
    # Priority: 1. Config (Matrix/Run), 2. Global Settings (GUI)
    degradation_conf = cfg.get("degradation", {})
    robot_settings_path = Path("configs/robot_settings.json")
    
    if not degradation_conf and robot_settings_path.exists():
        try:
            with open(robot_settings_path) as f:
                degradation_conf = json.load(f)
        except Exception as e:
            node.get_logger().warn(f"Failed to load global robot settings: {e}")

    degradation_active = degradation_conf.get("enabled", False)
    
    if degradation_active:
        # Save effective settings to config for traceability
        cfg["degradation"] = degradation_conf
        
        rs = degradation_conf
        degrader_cmd = [
            "python3", "tools/sensor_degrader.py",
            "--ros-args",
            "-p", f"max_range:={rs.get('max_range', 10.0)}",
            "-p", f"noise_std:={rs.get('noise_std', 0.0)}",
            "-p", f"speed_scale:={rs.get('speed_scale', 1.0)}",
            "-p", "enabled:=True"
        ]
        
    # --------- Scenario: multi-process support ----------
    scenario = cfg.get("dataset", {}).get("scenario", {}) or {}
    scenario_processes = scenario.get("processes", None)
    
    # Apply degradation to scenario processes if active
    if degradation_active:
        node.get_logger().info("Applying Robot/Sensor DEGRADATION for this run.")
        if not scenario_processes:
            # Handle legacy or default wrap (will be created below if None)
            pass 
        else:
            for proc in scenario_processes:
                if "sim" in proc["name"].lower():
                    # Remap simulator outputs to _raw so degrader can intercept
                    proc["cmd"] += ["--ros-args", "--remap", "scan:=scan_raw", "--remap", "cmd_vel:=cmd_vel_raw"]
            
            # Add degrader process
            scenario_processes.append({
                "name": "degrader",
                "cmd": degrader_cmd,
                "env": {},
                "cwd": str(Path.cwd())
            })
            
        # Save updated config with degradation params/processes to disk
        try:
            with open(config_path, "w") as f:
                yaml.dump(cfg, f, default_flow_style=False)
        except Exception as e:
            node.get_logger().error(f"Failed to save resolved config: {e}")

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
            # Check for optional startup delay
            delay_s = proc.get("delay_s", 0.0)
            if delay_s > 0:
                node.get_logger().info(f"[STARTUP] Waiting {delay_s}s before starting '{proc['name']}'...")
                time.sleep(delay_s)
            
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

        # Wait for all processes to initialize (especially explorer with delay_s)
        node.get_logger().info("[STARTUP] Waiting for all processes to initialize...")
        time.sleep(2.0)
        
        # Pause exploration for stable probes/warmup (now that explorer is running)
        set_explore(False)

        # WAIT_READY (probes)
        for p in cfg["probes"]["ready"]:
            probe = build_probe(p)
            res = probe.run(ctx)
            if not res.ok:
                node.get_logger().error(f"[PROBE FAIL] {res.message}")
                return 2
                logger.error(f"PROBE FAILED - EXITING WITH CODE 2 (evaluation will NOT run)")
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
        logger.info(f"[EVAL] Run completed. Duration: {t2-t1:.1f}s")
        
        # Stop resource monitoring
        monitor_active = False
        if monitor_thread:
            monitor_thread.join(timeout=2.0)
        logger.info(f"[EVAL] Monitoring stopped. Max CPU: {system_metrics['max_cpu']:.1f}%, Max RAM: {system_metrics['max_ram']:.1f}MB")
            
        return 0

    finally:
        try:
            node.get_logger().info("[EVAL] Saving system metrics immediately (Deferring full evaluation)...")
            
            metrics_path = cfg["paths"]["metrics_json"]
            
            # Extract degradation settings with defaults
            degradation_cfg = cfg.get("degradation", {})
            lidar_noise = 0.0
            lidar_range = 10.0
            speed_scale = 1.0
            
            if degradation_cfg and degradation_cfg.get("enabled", False):
                lidar_noise = degradation_cfg.get("noise_std", 0.0)
                lidar_range = degradation_cfg.get("max_range", 10.0)
                speed_scale = degradation_cfg.get("speed_scale", 1.0)
            
            logger.info(f"[EVAL] Degradation values: noise={lidar_noise}, range={lidar_range}, scale={speed_scale}")
            
            # Create metrics dictionary with just system info
            metrics = {
                "duration_s": float(t2 - t1),
                "max_cpu_percent": system_metrics["max_cpu"],
                "max_ram_mb": system_metrics["max_ram"],
                "degradation": degradation_cfg if degradation_cfg else None,
                "lidar_noise": lidar_noise,
                "lidar_range": lidar_range,
                "speed_scale": speed_scale,
                
                # Placeholders explicitly set to None to indicate they need computation
                "ate_rmse": None,
                "coverage": None,
                "accessible_coverage": None,
                "occupancy_iou": None,
                "map_ssim": None,
                "wall_thickness_m": None,
                "is_failure": False, # Default
                "failure_reasons": []
            }
            
            logger.info(f"[EVAL] Writing partial metrics to: {metrics_path}")
            with open(metrics_path, "w") as f:
                json.dump(metrics, f, indent=4)
            logger.info(f"[EVAL] ✅ Partial metrics saved successfully.")
            
        except Exception as e:
            node.get_logger().error(f"Failed to save system metrics: {e}")
            import traceback
            traceback.print_exc()
            logger.error(f"[EVAL] Traceback printed above")
        
        node.get_logger().info("[EVAL] Cleaning up ROS node...")
        # Cleanup ROS node
        node.destroy_node()
        rclpy.shutdown()

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

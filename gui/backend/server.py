from fastapi import FastAPI, HTTPException, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
import os
import yaml
import json
import subprocess
import asyncio
import sys
from pathlib import Path

# Add project root to sys.path
PROJECT_ROOT = Path(__file__).parent.parent.parent
sys.path.append(str(PROJECT_ROOT))

from runner.resolve import load_yaml, resolve_run_config, stable_run_id, write_yaml

app = FastAPI(title="SLAM Bench Orchestrator API")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# In-memory status for running jobs
class JobStatus:
    def __init__(self):
        self.is_running = False
        self.logs = []
        self.current_job = 0
        self.total_jobs = 0
        self.current_id = None

status_manager = JobStatus()

@app.get("/api/config/matrix")
async def get_matrix():
    matrix_path = PROJECT_ROOT / "configs/matrix.yaml"
    if not matrix_path.exists():
        raise HTTPException(status_code=404, detail="matrix.yaml not found")
    return load_yaml(matrix_path)

@app.post("/api/config/matrix")
async def update_matrix(data: Dict[str, Any]):
    matrix_path = PROJECT_ROOT / "configs/matrix.yaml"
    write_yaml(matrix_path, data)
    return {"status": "success"}

@app.get("/api/slams")
async def list_slams():
    slams_dir = PROJECT_ROOT / "configs/slams"
    if not slams_dir.exists():
        return []
    slams = []
    for f in slams_dir.glob("*.yaml"):
        slams.append({"id": f.stem, "profile": f"configs/slams/{f.name}"})
    return slams

@app.get("/api/datasets")
async def list_datasets():
    # Datasets are usually defined in matrix.yaml, but we might have a registry
    matrix = await get_matrix()
    return matrix.get("datasets", [])

@app.get("/api/results")
async def list_results():
    results_dir = PROJECT_ROOT / "results/runs"
    if not results_dir.exists():
        return []
    
    runs = []
    # Each sub dir is a run
    for d in sorted(results_dir.iterdir(), reverse=True):
        if d.is_dir():
            metrics_path = d / "metrics.json"
            config_path = d / "config_resolved.yaml"
            run_data = {"id": d.name, "metrics": None, "config": None}
            if metrics_path.exists():
                with open(metrics_path, "r") as f:
                    run_data["metrics"] = json.load(f)
            if config_path.exists():
                run_data["config"] = load_yaml(config_path)
            runs.append(run_data)
    return runs

async def run_jobs_task(matrix_path: str, use_gui: bool):
    global status_manager
    status_manager.is_running = True
    status_manager.logs = []
    
    matrix = load_yaml(matrix_path)
    output_root = matrix.get("output", {}).get("root_dir", "results/runs")
    Path(output_root).mkdir(parents=True, exist_ok=True)

    jobs = []
    slams_map = {s["id"]: s for s in matrix.get("slams", [])}
    datasets_map = {d["id"]: d for d in matrix.get("datasets", [])}

    for inc in matrix.get("matrix", {}).get("include", []):
        d_id = inc["dataset"]
        dataset_def = datasets_map.get(d_id)
        if not dataset_def:
            continue
            
        for s_id in inc.get("slams", []):
            slam_entry = slams_map.get(s_id)
            if not slam_entry:
                continue
            
            profile_path = PROJECT_ROOT / slam_entry["profile"]
            slam_profile = load_yaml(profile_path)
            
            for seed in inc.get("seeds", [0]):
                for r in range(inc.get("repeats", 1)):
                    run_id = stable_run_id(d_id, s_id, seed, r)
                    resolved = resolve_run_config(
                        matrix=matrix,
                        dataset_obj=dataset_def,
                        slam_entry=slam_entry,
                        slam_profile=slam_profile,
                        combo_overrides=inc.get("overrides"),
                        slam_overrides=slam_entry.get("overrides"),
                        dataset_overrides=dataset_def.get("overrides"),
                        seed=seed,
                        repeat_index=r,
                        run_id=run_id,
                        output_root=output_root
                    )
                    
                    if use_gui:
                        for proc in resolved.get("dataset", {}).get("scenario", {}).get("processes", []):
                            if proc.get("name") == "nav2_sim":
                                cmd = proc.get("cmd", [])
                                new_cmd = [arg.replace("headless:=True", "headless:=False").replace("gui:=False", "gui:=True") for arg in cmd]
                                proc["cmd"] = new_cmd

                    config_path = Path(output_root) / run_id / "config_resolved.yaml"
                    jobs.append((run_id, config_path, resolved))

    status_manager.total_jobs = len(jobs)
    
    for i, (run_id, config_path, resolved) in enumerate(jobs):
        status_manager.current_job = i + 1
        status_manager.current_id = run_id
        write_yaml(config_path, resolved)
        
        cmd = [sys.executable, "-m", "runner.run_one", str(config_path)]
        process = await asyncio.create_subprocess_exec(
            *cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
            cwd=str(PROJECT_ROOT)
        )
        
        # Read output
        async def read_stream(stream, prefix):
            while True:
                line = await stream.readline()
                if not line:
                    break
                status_manager.logs.append(f"{prefix}: {line.decode().strip()}")
                # Keep only last 1000 lines
                if len(status_manager.logs) > 1000:
                    status_manager.logs.pop(0)

        await asyncio.gather(
            read_stream(process.stdout, f"[{run_id}]"),
            read_stream(process.stderr, f"[{run_id}] ERR")
        )
        
        await process.wait()

    status_manager.is_running = False
    status_manager.current_id = None

@app.post("/api/run")
async def start_run(background_tasks: BackgroundTasks, gui: bool = False):
    if status_manager.is_running:
        raise HTTPException(status_code=400, detail="Job already running")
    
    matrix_path = PROJECT_ROOT / "configs/matrix.yaml"
    background_tasks.add_task(run_jobs_task, str(matrix_path), gui)
    return {"status": "started"}

@app.get("/api/status")
async def get_status():
    return {
        "is_running": status_manager.is_running,
        "current_job": status_manager.current_job,
        "total_jobs": status_manager.total_jobs,
        "current_id": status_manager.current_id,
        "logs": status_manager.logs[-100:] # Latest 100 logs
    }

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)

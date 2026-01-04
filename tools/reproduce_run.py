from __future__ import annotations
import argparse
from runner.orchestrator import run_once

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True, help="path to config_resolved.yaml")
    args = ap.parse_args()
    rc = run_once(args.config)
    raise SystemExit(rc)

if __name__ == "__main__":
    main()

import sys
from pathlib import Path
root_dir = Path(__file__).resolve().parent.parent
if str(root_dir) not in sys.path:
    sys.path.insert(0, str(root_dir))
import argparse
from runner.orchestrator import run_once

def main():
    parser = argparse.ArgumentParser(description="Run a single SLAM benchmark run from a resolved config.")
    parser.add_argument("config_path", help="Path to config_resolved.yaml")
    args = parser.parse_args()

    sys.exit(run_once(args.config_path))

if __name__ == "__main__":
    main()

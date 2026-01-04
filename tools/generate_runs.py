from __future__ import annotations
from pathlib import Path
import argparse
import yaml

from runner.resolve import load_yaml, resolve_run_config, write_yaml, stable_run_id


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--matrix", required=True, help="configs/matrix.yaml")
    ap.add_argument("--output-root", default="results/runs")
    args = ap.parse_args()

    matrix = load_yaml(args.matrix)

    # Index datasets by id
    datasets = {d["id"]: d for d in (matrix.get("datasets") or [])}
    slams = {s["id"]: s for s in (matrix.get("slams") or [])}

    out_root = args.output_root
    Path(out_root).mkdir(parents=True, exist_ok=True)

    run_count = 0
    for inc in matrix.get("matrix", {}).get("include", []) or []:
        dataset_id = inc["dataset"]
        slam_ids = inc["slams"]
        repeats = int(inc.get("repeats", 1))
        seeds = inc.get("seeds", [0])
        tags = inc.get("tags", [])
        combo_overrides = inc.get("overrides", {}) or {}
        combo_overrides["tags"] = tags

        dataset_obj = datasets[dataset_id]
        dataset_overrides = (dataset_obj.get("overrides") or {})

        for slam_id in slam_ids:
            slam_entry = slams[slam_id]
            slam_profile = load_yaml(Path("configs") / slam_entry["profile"])  # assumes run from repo root
            slam_overrides = slam_entry.get("overrides") or {}

            for r in range(repeats):
                for seed in seeds:
                    run_id = stable_run_id(dataset_id, slam_id, int(seed), int(r))
                    resolved = resolve_run_config(
                        matrix=matrix,
                        dataset_obj=dataset_obj,
                        slam_entry=slam_entry,
                        slam_profile=slam_profile,
                        combo_overrides=combo_overrides,
                        slam_overrides=slam_overrides,
                        dataset_overrides=dataset_overrides,
                        seed=int(seed),
                        repeat_index=int(r),
                        run_id=run_id,
                        output_root=out_root,
                    )
                    write_yaml(Path(out_root) / run_id / "config_resolved.yaml", resolved)
                    run_count += 1
                    print(f"[OK] wrote {out_root}/{run_id}/config_resolved.yaml")

    print(f"Generated {run_count} runs.")


if __name__ == "__main__":
    main()

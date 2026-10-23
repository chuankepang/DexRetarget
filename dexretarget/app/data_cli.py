"""python -m dexretarget.dataset: inspect, simulate, align, export and reversible deletion."""

import argparse
import json
import time
from pathlib import Path

from dexretarget.dataset.config import DEFAULT_CONFIG, load_config, project_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ["inspect", "validate", "align"]:
        command = sub.add_parser(name)
        command.add_argument("episode", type=Path)
    sub.add_parser("list")
    for name, argument in [("trash", "episode_id"), ("restore", "trash_id")]:
        command = sub.add_parser(name)
        command.add_argument(argument)
    simulation = sub.add_parser("simulate")
    simulation.add_argument("--seconds", type=float, default=3)
    simulation.add_argument("--real-cameras", action="store_true")
    simulation.add_argument("--output-root", type=Path)
    export = sub.add_parser("export")
    export.add_argument("episodes", nargs="+", type=Path)
    export.add_argument("--output", type=Path, required=True)
    export.add_argument("--repo-id", required=True)
    export.add_argument("--allow-simulated", action="store_true")
    export.add_argument("--include-failed", action="store_true")
    args = parser.parse_args()
    config = load_config(args.config)
    root = project_path(config["output_root"]) / config["dataset_name"]
    if args.command == "list":
        for path in sorted(root.glob("episode_*")):
            meta = json.loads((path / "metadata.json").read_text())
            print(path.name, "success=" + str(meta["success"]), meta.get("quality", {}), meta["task"])
    elif args.command in {"inspect", "validate", "align"}:
        from dexretarget.dataset.processing import validate_episode
        from dexretarget.dataset.alignment import align_episode
        from dexretarget.dataset.storage import write_json_atomic
        if args.command == "inspect":
            result = json.loads((args.episode / "metadata.json").read_text())
        else:
            result = validate_episode(args.episode)
            if args.command == "align":
                result["quality"] = align_episode(args.episode / "episode.h5", result)
                write_json_atomic(args.episode / "metadata.json", result)
        print(json.dumps(result, indent=2, ensure_ascii=False))
    elif args.command == "trash":
        from dexretarget.dataset.storage import trash_episode
        print(trash_episode(root, args.episode_id))
    elif args.command == "restore":
        from dexretarget.dataset.storage import restore_episode
        print(restore_episode(root, args.trash_id))
    elif args.command == "export":
        from dexretarget.dataset.processing import export_lerobot
        print(export_lerobot(args.episodes, args.output, args.repo_id, args.allow_simulated, args.include_failed))
    elif args.command == "simulate":
        from .simulation import run_simulation
        run_simulation(args)

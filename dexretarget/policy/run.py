"""Register pi05_rokae before running an official OpenPI script, without editing it."""

import argparse
import os
from pathlib import Path
import runpy
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--openpi-root", type=Path, required=True)
    parser.add_argument("--repo-id", required=True)
    parser.add_argument("--dataset-root", type=Path, required=True, help="Parent of repo-id directories")
    parser.add_argument("script", choices=["compute_norm_stats", "train", "train_pytorch", "serve_policy"])
    parser.add_argument("script_args", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    script = args.openpi_root.resolve() / "scripts" / (args.script + ".py")
    if not script.is_file():
        parser.error(f"Script not found: {script}")
    os.environ["HF_LEROBOT_HOME"] = str(args.dataset_root.resolve())
    sys.path.insert(0, str(args.openpi_root.resolve() / "src"))
    from openpi.training import config
    from .rokae_policy import make_train_config

    custom = make_train_config(args.repo_id)
    config._CONFIGS_DICT[custom.name] = custom
    sys.argv = [str(script), *args.script_args]
    runpy.run_path(str(script), run_name="__main__")


if __name__ == "__main__":
    main()

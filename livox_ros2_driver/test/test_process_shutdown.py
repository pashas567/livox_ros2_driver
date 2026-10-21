"""Exercise executable teardown, including SDK static logger destruction."""

import argparse
import os
import signal
import subprocess
import time
from pathlib import Path


def check_shutdown(executable: Path, config: Path) -> None:
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = "84"
    process = subprocess.Popen(
        [str(executable), "--ros-args", "-p", f"user_config_path:={config}"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        env=environment,
    )
    try:
        time.sleep(1)
        process.send_signal(signal.SIGINT)
        output, _ = process.communicate(timeout=3)
        print(output.decode(errors="replace"), flush=True)
        if process.returncode != 0:
            raise RuntimeError(
                f"Driver exited with status {process.returncode}"
            )
        if b"Init lds lidar success" not in output:
            raise RuntimeError("Driver did not initialize before SIGINT")
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--executable", required=True, type=Path)
    parser.add_argument("--config", required=True, type=Path)
    args = parser.parse_args()
    for _ in range(2):
        check_shutdown(args.executable, args.config)


if __name__ == "__main__":
    main()

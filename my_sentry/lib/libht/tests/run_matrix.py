"""Run the standalone native library graph under both codec backends/standards."""
import argparse
import json
from pathlib import Path
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
args.output = args.output.resolve()
args.output.mkdir(parents=True, exist_ok=True)
report = []
for backend in ("zpp", "pfr"):
    for standard in ("cxx20", "cxx23"):
        name = backend + "-" + standard
        commands = [["xmake", "f", "-P", str(root), "-m", "debug", "--tests=y", "--wire_backend=" + backend, "--wire_standard=" + standard],
                    ["xmake", "test", "-P", str(root), "-j", "4", "-g", "tests"]]
        entry = {"name": name, "passed": False, "commands": commands}
        report.append(entry)
        with (args.output / (name + ".log")).open("w") as log:
            for command in commands:
                result = subprocess.run(command, cwd=root, stdout=log, stderr=subprocess.STDOUT)
                if result.returncode:
                    entry["failed_command"] = command
                    (args.output / "matrix.json").write_text(json.dumps(report, indent=2) + "\n")
                    raise SystemExit(name + " failed; see " + str(log.name))
        entry["passed"] = True
        (args.output / "matrix.json").write_text(json.dumps(report, indent=2) + "\n")
        print(name + ": passed", flush=True)

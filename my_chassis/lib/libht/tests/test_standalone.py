"""Verify a copied LibHT checkout without its consumer, runtime or modules."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

parser = argparse.ArgumentParser()
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
source = Path(__file__).resolve().parents[1]
args.output = args.output.resolve()
args.output.mkdir(parents=True, exist_ok=True)
# Ignore only generated state and an unrelated documentation theme, never headers.
with tempfile.TemporaryDirectory(prefix="libht standalone ") as temporary:
    root = Path(temporary) / "libht"
    shutil.copytree(source, root, ignore=shutil.ignore_patterns(".git", ".xmake", "build", "__pycache__", "compile_commands.json", "doxygen-awesome-css"))
    digest = hashlib.sha256()
    eigen = root / "lib/eigen"
    for file in sorted(p for p in eigen.rglob("*") if p.is_file()):
        digest.update(file.relative_to(eigen).as_posix().encode() + b"\0" + hashlib.sha256(file.read_bytes()).digest())
    assert digest.hexdigest() == json.loads((root / "lib/versions.json").read_text())["eigen"]["snapshot_sha256"]
    for file in root.rglob("*"):
        relative = file.relative_to(root)
        if any(part in ("lib", "third_party") for part in relative.parts):
            continue
        if file.is_dir():
            assert file.name == file.name.lower(), relative
        if file.suffix in (".cpp", ".hpp", ".tpp", ".lua") or file.name == "CMakeLists.txt":
            text = file.read_text()
            assert "LibXR::" not in text and "libxr/" not in text, relative
    environment = os.environ.copy()
    environment.pop("PYTHONPATH", None)
    environment["XMAKE_GLOBALDIR"] = str(Path(temporary) / "global")
    library_environment = environment.copy()
    library_environment["PKG_CONFIG_PATH"] = str(Path(temporary) / "no-packages")
    library_environment["PKG_CONFIG_LIBDIR"] = library_environment["PKG_CONFIG_PATH"]
    subprocess.run(["xmake", "f", "-P", str(root), "-m", "debug"], cwd=root, env=library_environment, check=True)
    subprocess.run(["xmake", "build", "-P", str(root), "rmlink"], cwd=root, env=library_environment, check=True)
    # Run the matrix using the caller's Python; only codec probes use configured python3.
    command = [os.sys.executable, str(root / "tests/run_matrix.py"), "--output", str(args.output / "matrix")]
    subprocess.run(command, cwd=root, env=environment, check=True)
    database = []
    for file in sorted((root / "build").glob("*/debug/compile_commands.json")):
        records = json.loads(file.read_text())
        assert records, file
        database.extend(records)
        destination = args.output / file.relative_to(root / "build")
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(file, destination)
    assert database, "Target builds did not generate compile_commands.json"
    for entry in database:
        command = entry.get("command", " ".join(entry.get("arguments", [])))
        assert "LIBXR_" not in command and "/libxr/" not in command and "src/adaptation" not in command, command
    (args.output / "isolation.json").write_text(json.dumps({"passed": True, "compiler_records": len(database), "eigen_sha256": digest.hexdigest(), "consumer_present": False}, indent=2) + "\n")
print("Standalone copy, Eigen snapshot, lowercase paths and dependency isolation passed")

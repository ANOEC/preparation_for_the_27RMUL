"""Compile isolated codec objects and preserve size, stack, calls and provenance."""
import argparse
import hashlib
import json
import pathlib
import re
import shutil
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument("--output", type=pathlib.Path, required=True)
args = parser.parse_args()
root = pathlib.Path(__file__).resolve().parents[1]
args.output.mkdir(parents=True, exist_ok=True)
compiler = shutil.which("arm-none-eabi-g++")
if not compiler:
    raise SystemExit("arm-none-eabi-g++ is required on PATH")
tool_prefix = str(pathlib.Path(compiler).with_name("arm-none-eabi-"))
version = subprocess.check_output([compiler, "--version"], text=True)
report = {"compiler": version, "dependencies": json.loads((root / "third_party/versions.json").read_text()),
          "source_sha256": {}, "objects": []}
for path in (root / "include/wire/codec.hpp", root / "benchmarks/resource_probe.cpp",
             root / "benchmarks/reference_can.hpp", root / "benchmarks/payloads.hpp",
             root / "tests/bounded_payload.hpp", root / "include/wire/bits.hpp"):
    report["source_sha256"][str(path.relative_to(root))] = hashlib.sha256(path.read_bytes()).hexdigest()

for backend in ("zpp", "pfr"):
    for optimization in ("O2", "Os"):
        output = args.output / (backend + "-" + optimization + ".o")
        flags = [compiler, "-std=c++23", "-mcpu=cortex-m7", "-mthumb", "-mfpu=fpv5-d16",
                 "-mfloat-abi=hard", "-fno-exceptions", "-fno-rtti", "-ffunction-sections",
                 "-fdata-sections", "-fstack-usage", "-fcallgraph-info=su", "-" + optimization,
                 "-I" + str(root / "include")]
        if backend == "zpp":
            flags += ["-DWIRE_BACKEND_ZPP=1", "-isystem", str(root / "third_party/zpp")]
        else:
            flags += ["-DWIRE_BACKEND_PFR=1", "-DBOOST_PFR_USE_CPP17=1", "-DBOOST_PFR_USE_CPP26=0",
                      "-DBOOST_PFR_USE_LOOPHOLE=0", "-isystem", str(root / "third_party/pfr/include")]
        command = flags + ["-c", str(root / "benchmarks/resource_probe.cpp"), "-o", str(output)]
        subprocess.run(command, check=True)
        sizes = subprocess.check_output([tool_prefix + "size", "-A", str(output)], text=True)
        symbols = subprocess.check_output([tool_prefix + "nm", "-S", str(output)], text=True)
        assembly = subprocess.check_output([tool_prefix + "objdump", "-dr", str(output)], text=True)
        output.with_suffix(".sizes.txt").write_text(sizes)
        output.with_suffix(".symbols.txt").write_text(symbols)
        output.with_suffix(".assembly.txt").write_text(assembly)
        sections = {}
        for line in sizes.splitlines():
            fields = line.split()
            if len(fields) == 3 and fields[0].startswith("."):
                sections[fields[0]] = int(fields[1])
        function_sizes = {}
        for line in symbols.splitlines():
            match = re.match(r"[0-9a-f]+\s+([0-9a-f]+)\s+[tTwW]\s+(\S+)", line)
            if match:
                function_sizes[match[2]] = int(match[1], 16)
        graph = output.with_suffix(".ci").read_text()
        nodes, calls = {}, {}
        for match in re.finditer(r'node: \{ title: "([^"\n]+)" label: "((?:\\.|[^"\\])*)"', graph):
            size = re.search(r"(\d+) bytes", match[2])
            nodes[match[1]] = int(size[1]) if size else None
        for match in re.finditer(r'edge: \{ sourcename: "([^"]+)" targetname: "([^"]+)"', graph):
            calls.setdefault(match[1], set()).add(match[2])

        def analyze(name):
            seen, unknown = set(), set()
            def visit(node, active):
                if node in active:
                    unknown.add("recursive:" + node)
                    return 0
                seen.add(node)
                if nodes.get(node) is None:
                    unknown.add(node)
                child = max((visit(target, active | {node}) for target in calls.get(node, ())), default=0)
                return (nodes.get(node) or 0) + child
            stack = visit(name, set())
            reachable_bytes = sum(function_sizes.get(node.split(":")[-1], 0) for node in seen)
            return {"function_bytes": function_sizes.get(name), "owned_reachable_function_bytes": reachable_bytes,
                    "owned_callchain_stack_bytes": stack, "unmeasured_external_callees": sorted(unknown)}

        entry_points = {}
        for name in ("classic", "gimbal", "chassis", "vqf"):
            for direction in ("encode", "decode"):
                for prefix in ("", "reference_"):
                    function = prefix + name + "_" + direction
                    entry_points[function] = analyze(function)
        for direction in ("encode", "decode"):
            function = "bounded_" + direction
            entry_points[function] = analyze(function)
        forbidden = [line for line in symbols.splitlines() if re.search(r"\bU\s+(malloc|calloc|realloc|_Zn\w+|__cxa_throw)\b", line)]
        if forbidden:
            raise SystemExit("Unexpected allocation/throw symbols: " + str(forbidden))
        item = {"backend": backend, "optimization": optimization, "command": command,
                "sha256": hashlib.sha256(output.read_bytes()).hexdigest(),
                "text_bytes": sum(v for k, v in sections.items() if k.startswith(".text")),
                "rodata_bytes": sum(v for k, v in sections.items() if k.startswith(".rodata")),
                "data_bytes": sum(v for k, v in sections.items() if k.startswith(".data")),
                "bss_bytes": sum(v for k, v in sections.items() if k.startswith(".bss")),
                "entry_points": entry_points}
        report["objects"].append(item)
        print(backend, optimization, "text", item["text_bytes"], "data", item["data_bytes"], "bss", item["bss_bytes"])
report["scope"] = "Isolated object sizes include reference and selected backend functions. Stack sums cover known owned calls; external callees are listed, not assumed measured. No board timing."
(args.output / "resources.json").write_text(json.dumps(report, indent=2) + "\n")

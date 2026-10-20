import argparse
import pathlib
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument("--compiler", required=True)
parser.add_argument("--backend", choices=("zpp", "pfr"), required=True)
parser.add_argument("--arm", action="store_true")
args = parser.parse_args()
root = pathlib.Path(__file__).resolve().parents[1]
flags = [args.compiler, "-fsyntax-only", "-fno-exceptions", "-fno-rtti", "-I" + str(root / "include")]
if args.backend == "zpp":
    flags += ["-DWIRE_BACKEND_ZPP=1", "-isystem", str(root / "third_party/zpp")]
else:
    flags += ["-DWIRE_BACKEND_PFR=1", "-DBOOST_PFR_USE_CPP17=1", "-DBOOST_PFR_USE_CPP26=0",
              "-DBOOST_PFR_USE_LOOPHOLE=0", "-isystem", str(root / "third_party/pfr/include")]
if args.arm:
    flags += ["-mcpu=cortex-m7", "-mthumb", "-mfpu=fpv5-d16", "-mfloat-abi=hard"]
for standard in ("c++20", "c++23"):
    subprocess.run(flags + ["-std=" + standard, str(root / "tests/compile_probe.cpp")], check=True)
    subprocess.run(flags + ["-std=" + standard, str(root / "tests/usage_example.cpp")], check=True)
    for declaration in ("int* field;", "std::vector<int> field;", "const int field{};",
                        "int& field;", "int field[3];"):
        source = '#include <wire/codec.hpp>\n#include <vector>\nstruct Bad {' + declaration + '};\n'
        source += 'void check(const Bad& value, std::span<std::byte> bytes) { (void)wire::Encode(value, bytes); }\n'
        result = subprocess.run(flags + ["-std=" + standard, "-x", "c++", "-"],
                                input=source, text=True, capture_output=True)
        if result.returncode == 0:
            raise SystemExit("Unexpectedly accepted: " + declaration)
print(args.backend, "C++20/C++23 probes and rejected types passed")

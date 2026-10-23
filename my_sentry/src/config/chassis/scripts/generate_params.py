#!/usr/bin/env python3
"""Generate C++ chassis parameters from a TOML file.

The default invocation is intentionally path-independent::

    python src/config/chassis/scripts/generate_params.py

It reads ``../params/chassis.toml`` and writes
``../generated/chassis_params.hpp``. Input and output paths may also be passed
explicitly, which is how CMake invokes the generator.

TOML rules are compatible with the parameter generator in the reference
project:

* ``[[default.<category>]]`` supplies fields omitted by instances in that
  category.
* ``variable = "n"`` makes an instance compile-time-only (``constexpr``).
* ``variable`` is generator metadata and is not emitted as a C++ parameter.
"""

from __future__ import annotations

import argparse
import math
import os
import re
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any

try:
    import tomllib
except ModuleNotFoundError:  # Python < 3.11
    try:
        import tomli as tomllib  # type: ignore[no-redef]
    except ModuleNotFoundError as exc:  # pragma: no cover - depends on host Python
        raise SystemExit(
            "Python 3.11 or newer is required (or install the 'tomli' package)."
        ) from exc


@dataclass(frozen=True)
class ParamInstance:
    fields: dict[str, Any]
    constexpr: bool


CPP_KEYWORDS = {
    "alignas", "alignof", "and", "and_eq", "asm", "auto", "bitand",
    "bitor", "bool", "break", "case", "catch", "char", "char8_t",
    "char16_t", "char32_t", "class", "compl", "concept", "const",
    "consteval", "constexpr", "constinit", "const_cast", "continue",
    "co_await", "co_return", "co_yield", "decltype", "default", "delete",
    "do", "double", "dynamic_cast", "else", "enum", "explicit", "export",
    "extern", "false", "float", "for", "friend", "goto", "if", "inline",
    "int", "long", "mutable", "namespace", "new", "noexcept", "not",
    "not_eq", "nullptr", "operator", "or", "or_eq", "private", "protected",
    "public", "register", "reinterpret_cast", "requires", "return", "short",
    "signed", "sizeof", "static", "static_assert", "static_cast", "struct",
    "switch", "template", "this", "thread_local", "throw", "true", "try",
    "typedef", "typeid", "typename", "union", "unsigned", "using", "virtual",
    "void", "volatile", "wchar_t", "while", "xor", "xor_eq",
}
IDENTIFIER_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


def validate_identifier(name: str, description: str) -> None:
    if not IDENTIFIER_RE.fullmatch(name) or name in CPP_KEYWORDS:
        raise ValueError(f"{description} {name!r} is not a valid C++ identifier")


def single_table(value: Any, path: str) -> dict[str, Any]:
    """Normalize ``[table]`` and one-element ``[[array-of-table]]`` values."""
    if isinstance(value, dict):
        return value
    if isinstance(value, list) and len(value) == 1 and isinstance(value[0], dict):
        return value[0]
    raise ValueError(f"{path} must be a table or a one-element array of tables")


def parameter_fields(table: dict[str, Any], path: str) -> tuple[dict[str, Any], bool]:
    marker = table.get("variable")
    constexpr = isinstance(marker, str) and marker.lower() == "n"
    fields: dict[str, Any] = {}

    for name, value in table.items():
        if name == "variable":
            continue
        validate_identifier(name, f"parameter name in {path}")
        if isinstance(value, (dict, list)) or value is None:
            raise ValueError(f"{path}.{name} has unsupported value {value!r}")
        if not isinstance(value, (bool, int, float, str)):
            raise ValueError(f"{path}.{name} has unsupported type {type(value).__name__}")
        if isinstance(value, float) and not math.isfinite(value):
            raise ValueError(f"{path}.{name} must be a finite floating-point number")
        fields[name] = value

    if not fields:
        raise ValueError(f"{path} does not contain any parameters")
    return fields, constexpr


def parse_config(document: dict[str, Any]) -> dict[str, dict[str, ParamInstance]]:
    defaults: dict[str, dict[str, Any]] = {}
    raw_defaults = document.get("default", {})
    if not isinstance(raw_defaults, dict):
        raise ValueError("default must be a TOML table")

    for category, raw_table in raw_defaults.items():
        validate_identifier(category, "default category")
        table = single_table(raw_table, f"default.{category}")
        fields, _ = parameter_fields(table, f"default.{category}")
        defaults[category] = fields

    groups: dict[str, dict[str, ParamInstance]] = {}
    for category, raw_group in document.items():
        if category == "default":
            continue
        validate_identifier(category, "category")

        # A list denotes one unnamed category instance (for example [[imu]]).
        if isinstance(raw_group, list):
            table = single_table(raw_group, category)
            fields, constexpr = parameter_fields(table, category)
            groups[category] = {
                "": ParamInstance({**defaults.get(category, {}), **fields}, constexpr)
            }
            continue

        if not isinstance(raw_group, dict):
            raise ValueError(f"{category} must be a TOML table")

        # A mapping whose values are tables represents named instances such as
        # [[pid.motor1]].  Otherwise it is one unnamed [category] instance.
        named = bool(raw_group) and all(
            isinstance(value, (dict, list)) for value in raw_group.values()
        )
        if not named:
            fields, constexpr = parameter_fields(raw_group, category)
            groups[category] = {
                "": ParamInstance({**defaults.get(category, {}), **fields}, constexpr)
            }
            continue

        instances: dict[str, ParamInstance] = {}
        for instance_name, raw_table in raw_group.items():
            validate_identifier(instance_name, f"instance name in {category}")
            path = f"{category}.{instance_name}"
            table = single_table(raw_table, path)
            fields, constexpr = parameter_fields(table, path)
            instances[instance_name] = ParamInstance(
                {**defaults.get(category, {}), **fields}, constexpr
            )
        groups[category] = instances

    if not groups:
        raise ValueError("the TOML file does not define any parameter groups")
    return groups


def cpp_string(value: str) -> str:
    escaped = (
        value.replace("\\", "\\\\")
        .replace('"', '\\"')
        .replace("\n", "\\n")
        .replace("\r", "\\r")
        .replace("\t", "\\t")
    )
    return f'"{escaped}"'


def cpp_type_and_value(value: Any) -> tuple[str, str]:
    # bool is a subclass of int, so it must be checked first.
    if isinstance(value, bool):
        return "bool", "true" if value else "false"
    if isinstance(value, int):
        return "int", str(value)
    if isinstance(value, float):
        rendered = repr(value)
        if "." not in rendered and "e" not in rendered.lower():
            rendered += ".0"
        return "float", rendered + "f"
    if isinstance(value, str):
        return "const char*", cpp_string(value)
    raise AssertionError(f"unvalidated parameter value: {value!r}")


def emit_struct(lines: list[str], indent: str, name: str, instance: ParamInstance) -> None:
    lines.append(f"{indent}struct {name}_t {{")
    for field_name in sorted(instance.fields):
        cpp_type, cpp_value = cpp_type_and_value(instance.fields[field_name])
        prefix = "static constexpr " if instance.constexpr else ""
        lines.append(f"{indent}    {prefix}{cpp_type} {field_name} = {cpp_value};")
    lines.append(f"{indent}}};")
    qualifier = "inline constexpr" if instance.constexpr else "inline"
    lines.append(f"{indent}{qualifier} {name}_t {name} {{}};")


def generate_header(config_name: str, groups: dict[str, dict[str, ParamInstance]]) -> str:
    validate_identifier(config_name, "configuration file name")
    lines = [
        "#pragma once",
        "",
        "/// @file chassis_params.hpp",
        "/// @brief Auto-generated parameters from chassis.toml.",
        "/// @warning DO NOT EDIT. Modify chassis.toml and regenerate instead.",
        "",
        f"namespace param::{config_name} {{",
        "",
    ]

    for category in sorted(groups):
        instances = groups[category]
        if set(instances) == {""}:
            emit_struct(lines, "    ", category, instances[""])
            lines.append("")
            continue

        lines.append(f"    namespace {category} {{")
        lines.append("")
        for instance_name in sorted(instances):
            emit_struct(lines, "        ", instance_name, instances[instance_name])
            lines.append("")
        lines.append(f"    }} // namespace {category}")
        lines.append("")

    lines.append(f"}} // namespace param::{config_name}")
    lines.append("")
    return "\n".join(lines)


def write_if_changed(path: Path, content: str) -> bool:
    encoded = content.encode("utf-8")
    if path.exists() and path.read_bytes() == encoded:
        return False

    path.parent.mkdir(parents=True, exist_ok=True)
    # Replace atomically so a cancelled build cannot leave a truncated header.
    fd, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(encoded)
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()
    return True


def parse_args(argv: list[str]) -> argparse.Namespace:
    script_dir = Path(__file__).resolve().parent
    config_dir = script_dir.parent
    parser = argparse.ArgumentParser(
        description="Generate generated/chassis_params.hpp from chassis.toml."
    )
    parser.add_argument(
        "input", nargs="?", type=Path,
        default=config_dir / "params" / "chassis.toml",
        help="input TOML file (default: ../params/chassis.toml)",
    )
    parser.add_argument(
        "output", nargs="?", type=Path,
        default=config_dir / "generated" / "chassis_params.hpp",
        help="output C++ header (default: ../generated/chassis_params.hpp)",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    input_path: Path = args.input.resolve()
    output_path: Path = args.output.resolve()

    try:
        with input_path.open("rb") as stream:
            document = tomllib.load(stream)
        groups = parse_config(document)
        content = generate_header(input_path.stem, groups)
        changed = write_if_changed(output_path, content)
    except (OSError, tomllib.TOMLDecodeError, ValueError) as exc:
        print(f"generate_params.py: error: {exc}", file=sys.stderr)
        return 1

    action = "Generated" if changed else "Unchanged"
    print(f"{action}: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

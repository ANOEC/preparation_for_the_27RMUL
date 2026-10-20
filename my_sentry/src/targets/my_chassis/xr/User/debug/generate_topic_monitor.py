#!/usr/bin/env python3
"""Generate the TopicDebugApp monitor registry from TOML."""

from __future__ import annotations

import argparse
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
    except ModuleNotFoundError as exc:  # pragma: no cover
        raise SystemExit(
            "Python 3.11 or newer is required (or install the 'tomli' package)."
        ) from exc


IDENTIFIER_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
CPP_TYPE_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_:<> ,*&]*$")
CPP_BUILTIN_TYPES = {
    "bool", "char", "signed char", "unsigned char", "short",
    "unsigned short", "int", "unsigned int", "long", "unsigned long",
    "long long", "unsigned long long", "float", "double", "long double",
}


@dataclass(frozen=True)
class Topic:
    identifier: str
    name: str
    cpp_type: str
    printer: str


def require_string(table: dict[str, Any], key: str, index: int) -> str:
    value = table.get(key)
    if not isinstance(value, str) or not value:
        raise ValueError(f"topic[{index}].{key} must be a non-empty string")
    return value


def parse_topics(document: dict[str, Any]) -> list[Topic]:
    if set(document) != {"topic"}:
        unknown = sorted(set(document) - {"topic"})
        detail = f": {', '.join(unknown)}" if unknown else ""
        raise ValueError(f"only [[topic]] tables are allowed{detail}")

    raw_topics = document.get("topic")
    if not isinstance(raw_topics, list) or not raw_topics:
        raise ValueError("at least one [[topic]] table is required")

    topics: list[Topic] = []
    identifiers: set[str] = set()
    names: set[str] = set()
    allowed_keys = {"id", "name", "type", "printer"}
    for index, raw in enumerate(raw_topics):
        if not isinstance(raw, dict):
            raise ValueError(f"topic[{index}] must be a table")
        unknown = sorted(set(raw) - allowed_keys)
        missing = sorted(allowed_keys - set(raw))
        if unknown:
            raise ValueError(f"topic[{index}] has unknown fields: {', '.join(unknown)}")
        if missing:
            raise ValueError(f"topic[{index}] is missing fields: {', '.join(missing)}")

        identifier = require_string(raw, "id", index)
        name = require_string(raw, "name", index)
        cpp_type = require_string(raw, "type", index)
        printer = require_string(raw, "printer", index)
        if not IDENTIFIER_RE.fullmatch(identifier):
            raise ValueError(f"topic[{index}].id {identifier!r} is not a C++ identifier")
        if not IDENTIFIER_RE.fullmatch(printer):
            raise ValueError(f"topic[{index}].printer {printer!r} is not a C++ identifier")
        if not CPP_TYPE_RE.fullmatch(cpp_type):
            raise ValueError(f"topic[{index}].type {cpp_type!r} is not a supported C++ type")
        if any(character in name for character in "\0\r\n"):
            raise ValueError(f"topic[{index}].name contains a control character")
        if identifier in identifiers:
            raise ValueError(f"duplicate topic id {identifier!r}")
        if name in names:
            raise ValueError(f"duplicate topic name {name!r}")
        identifiers.add(identifier)
        names.add(name)
        topics.append(Topic(identifier, name, cpp_type, printer))
    return topics


def cpp_string(value: str) -> str:
    return '"' + value.replace("\\", "\\\\").replace('"', '\\"') + '"'


def explicit_type(cpp_type: str) -> str:
    """Qualify private TopicDebugApp aliases for namespace-scope instantiation."""
    if cpp_type in CPP_BUILTIN_TYPES or "::" in cpp_type:
        return cpp_type
    return f"TopicDebugApp::{cpp_type}"


def generate(topics: list[Topic]) -> str:
    lines = [
        "// Generated from topic_monitor.toml. DO NOT EDIT.",
        "// This file is intentionally included twice with different mode macros.",
        "",
        "#if defined(USER_APP_TOPIC_MONITOR_MEMBERS)",
        f"static constexpr std::size_t MONITOR_COUNT = {len(topics)};",
        "",
    ]
    for topic in topics:
        lines.append(
            f"Monitor<{topic.cpp_type}> {topic.identifier}_{{{cpp_string(topic.name)}, "
            f"{topic.printer}}};"
        )
    lines.extend(["", "MonitorBase* monitors_[MONITOR_COUNT] = {"])
    lines.extend(f"    &{topic.identifier}_," for topic in topics)
    lines.extend(["};", "", "#elif defined(USER_APP_TOPIC_MONITOR_INSTANTIATIONS)"])

    seen_types: set[str] = set()
    for topic in topics:
        if topic.cpp_type in seen_types:
            continue
        seen_types.add(topic.cpp_type)
        lines.append(
            f"template class TopicDebugApp::Monitor<{explicit_type(topic.cpp_type)}>;"
        )
    lines.extend([
        "#else",
        '#error "Define a topic monitor generation mode before including this file"',
        "#endif",
        "",
    ])
    return "\n".join(lines)


def write_if_changed(path: Path, content: str) -> bool:
    encoded = content.encode("utf-8")
    if path.exists() and path.read_bytes() == encoded:
        return False
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(encoded)
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()
    return True


def parse_args(argv: list[str]) -> argparse.Namespace:
    script_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "input", nargs="?", type=Path, default=script_dir / "topic_monitor.toml"
    )
    parser.add_argument(
        "output", nargs="?", type=Path,
        default=script_dir / "generated" / "topic_monitor.hpp",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        with args.input.resolve().open("rb") as stream:
            document = tomllib.load(stream)
        content = generate(parse_topics(document))
        changed = write_if_changed(args.output.resolve(), content)
    except (OSError, tomllib.TOMLDecodeError, ValueError) as exc:
        print(f"generate_topic_monitor.py: error: {exc}", file=sys.stderr)
        return 1
    action = "Generated" if changed else "Unchanged"
    print(f"{action}: {args.output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

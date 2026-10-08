#!/usr/bin/env python3
"""Log gimbal SYS_ID serial data to disk and mirror it to VOFA over TCP."""

from __future__ import annotations

import argparse
import csv
import re
import select
import socket
import sys
from datetime import datetime
from pathlib import Path

import serial
from serial.tools import list_ports


SYS_ID_PATTERN = re.compile(
    rb"gimbal_sys_id:"
    rb"([-+]?\d+(?:\.\d+)?),"
    rb"(\d+),"
    rb"([-+]?\d+(?:\.\d+)?),"
    rb"([-+]?\d+(?:\.\d+)?),"
    rb"([-+]?\d+(?:\.\d+)?)"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Read the STM32 serial stream, save SYS_ID samples as CSV, and "
            "mirror the unchanged stream to a TCP client such as VOFA+."
        )
    )
    parser.add_argument("--port", help="Serial port, for example COM7")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--listen-host", default="127.0.0.1")
    parser.add_argument("--listen-port", type=int, default=1347)
    parser.add_argument("--output-dir", type=Path, default=Path("logs"))
    parser.add_argument(
        "--list-ports", action="store_true", help="List serial ports and exit"
    )
    return parser.parse_args()


def print_ports() -> None:
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return
    for port in ports:
        print(f"{port.device}: {port.description}")


def remove_client(clients: list[socket.socket], client: socket.socket) -> None:
    try:
        clients.remove(client)
    except ValueError:
        pass
    try:
        client.close()
    except OSError:
        pass


def main() -> int:
    args = parse_args()
    if args.list_ports:
        print_ports()
        return 0
    if not args.port:
        print("error: --port is required (use --list-ports first)", file=sys.stderr)
        return 2

    args.output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = args.output_dir / f"gimbal_sys_id_{stamp}.csv"
    raw_path = args.output_dir / f"serial_raw_{stamp}.log"

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((args.listen_host, args.listen_port))
    server.listen()
    server.setblocking(False)

    clients: list[socket.socket] = []
    line_buffer = bytearray()
    sample_count = 0

    try:
        with serial.Serial(args.port, args.baud, timeout=0.02) as uart, \
                raw_path.open("ab", buffering=0) as raw_file, \
                csv_path.open("w", newline="", encoding="utf-8", buffering=1) as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(
                [
                    "host_time",
                    "time_s",
                    "axis",
                    "sys_id_input_nm",
                    "angle_rad",
                    "angular_velocity_rad_s",
                ]
            )

            print(f"Serial: {args.port} @ {args.baud} baud")
            print(f"CSV:    {csv_path.resolve()}")
            print(f"Raw:    {raw_path.resolve()}")
            print(
                f"VOFA+:  TCP client -> {args.listen_host}:{args.listen_port} "
                "(use the same protocol engine as before)"
            )
            print("Press Ctrl+C to stop safely.")

            while True:
                readable, _, _ = select.select([server, *clients], [], [], 0)
                for sock in readable:
                    if sock is server:
                        client, address = server.accept()
                        client.setblocking(False)
                        clients.append(client)
                        print(f"VOFA client connected: {address[0]}:{address[1]}")
                    else:
                        try:
                            command = sock.recv(4096)
                        except OSError:
                            command = b""
                        if command:
                            uart.write(command)
                        else:
                            remove_client(clients, sock)
                            print("VOFA client disconnected")

                data = uart.read(max(uart.in_waiting, 1))
                if not data:
                    continue

                raw_file.write(data)
                for client in clients.copy():
                    try:
                        client.sendall(data)
                    except OSError:
                        remove_client(clients, client)

                line_buffer.extend(data)
                while b"\n" in line_buffer:
                    raw_line, _, remainder = line_buffer.partition(b"\n")
                    line_buffer = bytearray(remainder)
                    match = SYS_ID_PATTERN.search(raw_line.rstrip(b"\r"))
                    if match is None:
                        continue
                    writer.writerow(
                        [
                            datetime.now().astimezone().isoformat(
                                timespec="milliseconds"
                            ),
                            match.group(1).decode("ascii"),
                            match.group(2).decode("ascii"),
                            match.group(3).decode("ascii"),
                            match.group(4).decode("ascii"),
                            match.group(5).decode("ascii"),
                        ]
                    )
                    sample_count += 1
    except KeyboardInterrupt:
        print(f"\nStopped. Saved {sample_count} SYS_ID samples to {csv_path}")
        return 0
    except serial.SerialException as error:
        print(f"serial error: {error}", file=sys.stderr)
        return 1
    finally:
        for client in clients:
            remove_client(clients, client)
        server.close()


if __name__ == "__main__":
    raise SystemExit(main())

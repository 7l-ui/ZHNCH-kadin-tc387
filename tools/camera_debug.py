#!/usr/bin/env python3
"""Tune SCC8660 parameters and request one-shot frames over DEBUG UART."""

from __future__ import annotations

import argparse
import csv
import ctypes
from ctypes import wintypes
import math
from pathlib import Path
import struct
import sys
import time
import zlib

import numpy as np
from PIL import Image, ImageDraw


MAGIC = b"CIMG"
HEADER_SIZE = 48


class DCB(ctypes.Structure):
    _fields_ = [
        ("DCBlength", wintypes.DWORD),
        ("BaudRate", wintypes.DWORD),
        ("flags", wintypes.DWORD),
        ("wReserved", wintypes.WORD),
        ("XonLim", wintypes.WORD),
        ("XoffLim", wintypes.WORD),
        ("ByteSize", wintypes.BYTE),
        ("Parity", wintypes.BYTE),
        ("StopBits", wintypes.BYTE),
        ("XonChar", ctypes.c_char),
        ("XoffChar", ctypes.c_char),
        ("ErrorChar", ctypes.c_char),
        ("EofChar", ctypes.c_char),
        ("EvtChar", ctypes.c_char),
        ("wReserved1", wintypes.WORD),
    ]


class COMMTIMEOUTS(ctypes.Structure):
    _fields_ = [
        ("ReadIntervalTimeout", wintypes.DWORD),
        ("ReadTotalTimeoutMultiplier", wintypes.DWORD),
        ("ReadTotalTimeoutConstant", wintypes.DWORD),
        ("WriteTotalTimeoutMultiplier", wintypes.DWORD),
        ("WriteTotalTimeoutConstant", wintypes.DWORD),
    ]


class WinSerial:
    GENERIC_READ = 0x80000000
    GENERIC_WRITE = 0x40000000
    OPEN_EXISTING = 3
    FILE_ATTRIBUTE_NORMAL = 0x80
    PURGE_TXABORT = 0x0001
    PURGE_RXABORT = 0x0002
    PURGE_TXCLEAR = 0x0004
    PURGE_RXCLEAR = 0x0008
    INVALID_HANDLE_VALUE = ctypes.c_void_p(-1).value

    def __init__(self, port: str, baud: int):
        if sys.platform != "win32":
            raise RuntimeError("This serial backend currently supports Windows only")

        self.kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        self.kernel32.CreateFileW.restype = wintypes.HANDLE
        self.kernel32.CreateFileW.argtypes = [
            wintypes.LPCWSTR,
            wintypes.DWORD,
            wintypes.DWORD,
            wintypes.LPVOID,
            wintypes.DWORD,
            wintypes.DWORD,
            wintypes.HANDLE,
        ]
        self.kernel32.ReadFile.argtypes = [
            wintypes.HANDLE,
            wintypes.LPVOID,
            wintypes.DWORD,
            ctypes.POINTER(wintypes.DWORD),
            wintypes.LPVOID,
        ]
        self.kernel32.WriteFile.argtypes = [
            wintypes.HANDLE,
            wintypes.LPCVOID,
            wintypes.DWORD,
            ctypes.POINTER(wintypes.DWORD),
            wintypes.LPVOID,
        ]
        self.kernel32.SetupComm.argtypes = [
            wintypes.HANDLE,
            wintypes.DWORD,
            wintypes.DWORD,
        ]
        self.kernel32.GetCommState.argtypes = [
            wintypes.HANDLE,
            ctypes.POINTER(DCB),
        ]
        self.kernel32.SetCommState.argtypes = [
            wintypes.HANDLE,
            ctypes.POINTER(DCB),
        ]
        self.kernel32.SetCommTimeouts.argtypes = [
            wintypes.HANDLE,
            ctypes.POINTER(COMMTIMEOUTS),
        ]
        self.kernel32.PurgeComm.argtypes = [wintypes.HANDLE, wintypes.DWORD]
        self.kernel32.CloseHandle.argtypes = [wintypes.HANDLE]

        device = port if port.startswith("\\\\.\\") else f"\\\\.\\{port}"
        self.handle = self.kernel32.CreateFileW(
            device,
            self.GENERIC_READ | self.GENERIC_WRITE,
            0,
            None,
            self.OPEN_EXISTING,
            self.FILE_ATTRIBUTE_NORMAL,
            None,
        )
        if self.handle == self.INVALID_HANDLE_VALUE:
            raise ctypes.WinError(ctypes.get_last_error())

        try:
            self._configure(baud)
        except Exception:
            self.close()
            raise

    def _check(self, result: int) -> None:
        if not result:
            raise ctypes.WinError(ctypes.get_last_error())

    def _configure(self, baud: int) -> None:
        self._check(self.kernel32.SetupComm(self.handle, 65536, 65536))
        dcb = DCB()
        dcb.DCBlength = ctypes.sizeof(DCB)
        self._check(self.kernel32.GetCommState(self.handle, ctypes.byref(dcb)))
        dcb.BaudRate = baud
        dcb.flags = 1  # fBinary, no software/hardware flow control.
        dcb.ByteSize = 8
        dcb.Parity = 0
        dcb.StopBits = 0
        self._check(self.kernel32.SetCommState(self.handle, ctypes.byref(dcb)))
        timeouts = COMMTIMEOUTS(20, 0, 100, 0, 5000)
        self._check(self.kernel32.SetCommTimeouts(self.handle, ctypes.byref(timeouts)))
        self.purge()

    def purge(self) -> None:
        self._check(
            self.kernel32.PurgeComm(
                self.handle,
                self.PURGE_TXABORT
                | self.PURGE_RXABORT
                | self.PURGE_TXCLEAR
                | self.PURGE_RXCLEAR,
            )
        )

    def write(self, data: bytes) -> None:
        sent_total = 0
        while sent_total < len(data):
            view = data[sent_total:]
            buffer = ctypes.create_string_buffer(view)
            sent = wintypes.DWORD()
            self._check(
                self.kernel32.WriteFile(
                    self.handle, buffer, len(view), ctypes.byref(sent), None
                )
            )
            if sent.value == 0:
                raise TimeoutError("Serial write made no progress")
            sent_total += sent.value

    def read(self, size: int) -> bytes:
        buffer = ctypes.create_string_buffer(size)
        received = wintypes.DWORD()
        self._check(
            self.kernel32.ReadFile(
                self.handle, buffer, size, ctypes.byref(received), None
            )
        )
        return buffer.raw[: received.value]

    def close(self) -> None:
        if getattr(self, "handle", None) not in (None, self.INVALID_HANDLE_VALUE):
            self.kernel32.CloseHandle(self.handle)
            self.handle = None

    def __enter__(self) -> "WinSerial":
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.close()


class CameraLink:
    def __init__(self, serial: WinSerial):
        self.serial = serial
        self.pending = bytearray()

    def command(self, text: str, timeout: float = 3.0) -> str:
        self.serial.purge()
        self.pending.clear()
        self.serial.write((text.rstrip() + "\r\n").encode("ascii"))
        deadline = time.monotonic() + timeout
        line = bytearray()
        while time.monotonic() < deadline:
            chunk = self.serial.read(256)
            if not chunk:
                continue
            for byte in chunk:
                if byte in (10, 13):
                    if line.startswith((b"@CAM OK", b"@CAM ERR")):
                        return line.decode("ascii", errors="replace")
                    line.clear()
                elif 32 <= byte <= 126:
                    line.append(byte)
                else:
                    line.clear()
        raise TimeoutError(f"No camera reply for {text!r}")

    def set_parameter(self, name: str, value: int) -> str:
        reply = self.command(f"CAM {name} {value}")
        if not reply.startswith("@CAM OK"):
            raise RuntimeError(reply)
        print(reply)
        return reply

    def snapshot(self, size: str, timeout: float = 15.0) -> tuple[dict, bytes]:
        self.serial.purge()
        self.pending.clear()
        self.serial.write(f"CAM SNAP {size.upper()}\r\n".encode("ascii"))
        deadline = time.monotonic() + timeout
        line = bytearray()

        while time.monotonic() < deadline:
            chunk = self.serial.read(1024)
            if chunk:
                self.pending.extend(chunk)
                magic_index = self.pending.find(MAGIC)
                if magic_index >= 0:
                    del self.pending[:magic_index]
                    break
                for byte in chunk:
                    if byte in (10, 13):
                        if line.startswith(b"@CAM ERR"):
                            raise RuntimeError(line.decode("ascii", errors="replace"))
                        line.clear()
                    elif 32 <= byte <= 126:
                        line.append(byte)
                    else:
                        line.clear()
                if len(self.pending) > 4096:
                    del self.pending[:-3]
        else:
            raise TimeoutError("Snapshot header was not received")

        header = self._read_exact(HEADER_SIZE, deadline)
        metadata = parse_header(header)
        payload = self._read_exact(metadata["payload_size"], deadline)
        trailer = self._read_exact(4, deadline)
        expected_crc = struct.unpack("<I", trailer)[0]
        actual_crc = zlib.crc32(payload) & 0xFFFFFFFF
        if actual_crc != expected_crc:
            raise ValueError(
                f"Snapshot CRC mismatch: received 0x{expected_crc:08X}, "
                f"calculated 0x{actual_crc:08X}"
            )
        metadata["payload_crc32"] = actual_crc
        return metadata, payload

    def _read_exact(self, size: int, deadline: float) -> bytes:
        while len(self.pending) < size and time.monotonic() < deadline:
            chunk = self.serial.read(min(4096, size - len(self.pending)))
            if chunk:
                self.pending.extend(chunk)
        if len(self.pending) < size:
            raise TimeoutError(f"Expected {size} bytes, received {len(self.pending)}")
        result = bytes(self.pending[:size])
        del self.pending[:size]
        return result


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def parse_header(header: bytes) -> dict:
    if len(header) != HEADER_SIZE or header[:4] != MAGIC:
        raise ValueError("Invalid camera frame header")
    if header[4] != 1 or header[5] != 1 or header[7] != HEADER_SIZE:
        raise ValueError(f"Unsupported camera frame version/format: {header[4:8]!r}")
    expected_crc = struct.unpack_from("<H", header, 46)[0]
    actual_crc = crc16_ccitt(header[:46])
    if expected_crc != actual_crc:
        raise ValueError(
            f"Header CRC mismatch: received 0x{expected_crc:04X}, "
            f"calculated 0x{actual_crc:04X}"
        )
    return {
        "version": header[4],
        "format": "RGB565_BE",
        "sample_step": header[6],
        "sequence": struct.unpack_from("<I", header, 8)[0],
        "width": struct.unpack_from("<H", header, 12)[0],
        "height": struct.unpack_from("<H", header, 14)[0],
        "payload_size": struct.unpack_from("<I", header, 16)[0],
        "source_frame": struct.unpack_from("<I", header, 20)[0],
        "time_ms": struct.unpack_from("<I", header, 24)[0],
        "brightness": struct.unpack_from("<H", header, 28)[0],
        "white_balance": struct.unpack_from("<H", header, 30)[0],
        "auto_exposure": header[32],
        "target_valid": header[33],
        "confidence_pct": header[34],
        "fill_pct": header[35],
        "center_x": struct.unpack_from("<H", header, 36)[0],
        "center_y": struct.unpack_from("<H", header, 38)[0],
        "offset_x": struct.unpack_from("<h", header, 40)[0],
        "bbox_width": struct.unpack_from("<H", header, 42)[0],
        "bbox_height": struct.unpack_from("<H", header, 44)[0],
    }


def rgb565_to_array(payload: bytes, width: int, height: int) -> np.ndarray:
    pixels = np.frombuffer(payload, dtype=">u2").reshape(height, width)
    rgb = np.empty((height, width, 3), dtype=np.uint8)
    rgb[..., 0] = ((pixels >> 11) & 0x1F) * 255 // 31
    rgb[..., 1] = ((pixels >> 5) & 0x3F) * 255 // 63
    rgb[..., 2] = (pixels & 0x1F) * 255 // 31
    return rgb


def image_metrics(rgb: np.ndarray) -> dict:
    values = rgb.astype(np.float32)
    mean = values.mean(axis=(0, 1))
    luminance = (
        0.2126 * values[..., 0]
        + 0.7152 * values[..., 1]
        + 0.0722 * values[..., 2]
    )
    bright_cut = np.percentile(luminance, 75)
    bright = values[luminance >= bright_cut]
    bright_mean = bright.mean(axis=0) if len(bright) else mean
    neutral_error = float(np.abs(bright_mean - bright_mean.mean()).sum())
    exposure_error = abs(float(np.percentile(luminance, 90)) - 205.0) * 0.35
    clipped = float((luminance >= 250).mean())
    score = neutral_error + exposure_error + clipped * 100.0
    return {
        "mean_r": float(mean[0]),
        "mean_g": float(mean[1]),
        "mean_b": float(mean[2]),
        "mean_luma": float(luminance.mean()),
        "bright_r": float(bright_mean[0]),
        "bright_g": float(bright_mean[1]),
        "bright_b": float(bright_mean[2]),
        "clipped_pct": clipped * 100.0,
        "score": score,
    }


def save_snapshot(output: Path, metadata: dict, payload: bytes) -> tuple[Path, dict]:
    rgb = rgb565_to_array(payload, metadata["width"], metadata["height"])
    metrics = image_metrics(rgb)
    output.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(rgb, mode="RGB").save(output, format="BMP")
    print(
        f"saved {output} | {metadata['width']}x{metadata['height']} "
        f"BR={metadata['brightness']} WB=0x{metadata['white_balance']:02X} "
        f"RGB={metrics['mean_r']:.1f}/{metrics['mean_g']:.1f}/{metrics['mean_b']:.1f} "
        f"L={metrics['mean_luma']:.1f} target={metadata['target_valid']} "
        f"conf={metadata['confidence_pct']}% crc=0x{metadata['payload_crc32']:08X}"
    )
    return output, metrics


def default_name(metadata: dict) -> str:
    return (
        f"cam_s{metadata['sequence']:03d}_br{metadata['brightness']:03d}_"
        f"wb{metadata['white_balance']:02X}_{metadata['width']}x{metadata['height']}.bmp"
    )


def parse_int(text: str) -> int:
    return int(text, 0)


def parse_status(reply: str) -> dict[str, int | str]:
    values: dict[str, int | str] = {}
    for field in reply.split():
        if "=" not in field:
            continue
        name, raw_value = field.split("=", 1)
        try:
            values[name] = int(raw_value, 0)
        except ValueError:
            values[name] = raw_value
    return values


def run_diag(link: CameraLink, args: argparse.Namespace) -> None:
    first_reply = link.command("CAM GET")
    first = parse_status(first_reply)
    print(first_reply)
    time.sleep(args.interval)
    second_reply = link.command("CAM GET")
    second = parse_status(second_reply)
    print(second_reply)

    counter_names = ("HB", "FRAME", "VS", "DMA", "LOST")
    deltas = []
    for name in counter_names:
        if isinstance(first.get(name), int) and isinstance(second.get(name), int):
            deltas.append(f"d{name}={second[name] - first[name]}")
    if deltas:
        print(f"{args.interval:.1f}s delta: " + " ".join(deltas))

    voice = second.get("VOICE")
    heartbeat_delta = (
        second.get("HB", 0) - first.get("HB", 0)
        if isinstance(first.get("HB"), int) and isinstance(second.get("HB"), int)
        else None
    )
    vsync_delta = (
        second.get("VS", 0) - first.get("VS", 0)
        if isinstance(first.get("VS"), int) and isinstance(second.get("VS"), int)
        else None
    )
    dma_delta = (
        second.get("DMA", 0) - first.get("DMA", 0)
        if isinstance(first.get("DMA"), int) and isinstance(second.get("DMA"), int)
        else None
    )
    frame_delta = (
        second.get("FRAME", 0) - first.get("FRAME", 0)
        if isinstance(first.get("FRAME"), int)
        and isinstance(second.get("FRAME"), int)
        else None
    )

    if heartbeat_delta is None:
        print("diagnosis: firmware does not expose extended diagnostics")
    elif heartbeat_delta == 0:
        if voice == 1:
            print("diagnosis: CPU2 is occupied by voice capture")
        else:
            print("diagnosis: CPU2 camera worker is not being scheduled")
    elif vsync_delta == 0:
        print("diagnosis: CPU2 is alive, but camera VSYNC has stopped")
    elif dma_delta == 0:
        print("diagnosis: VSYNC is alive, but camera DMA completes no frames")
    elif frame_delta == 0:
        print("diagnosis: DMA is alive, but CPU2 consumes no completed frames")
    else:
        print("diagnosis: camera acquisition and CPU2 processing are running")


def make_contact_sheet(records: list[dict], output: Path) -> None:
    if not records:
        return
    columns = min(4, len(records))
    rows = math.ceil(len(records) / columns)
    cell_w = max(record["image"].width for record in records)
    cell_h = max(record["image"].height for record in records) + 30
    sheet = Image.new("RGB", (columns * cell_w, rows * cell_h), "white")
    draw = ImageDraw.Draw(sheet)
    for index, record in enumerate(records):
        x = (index % columns) * cell_w
        y = (index // columns) * cell_h
        sheet.paste(record["image"], (x, y))
        draw.text(
            (x + 2, y + record["image"].height + 2),
            f"BR {record['brightness']} WB {record['white_balance']:02X}\n"
            f"score {record['score']:.1f}",
            fill="black",
        )
    output.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(output)
    print(f"contact sheet: {output}")


def request_snapshot(
    link: CameraLink, size: str, attempts: int = 5, retry_delay: float = 0.1
) -> tuple[dict, bytes]:
    for attempt in range(attempts):
        try:
            return link.snapshot(size)
        except RuntimeError as exc:
            if "NO_FRAME" not in str(exc) or attempt + 1 >= attempts:
                raise
            time.sleep(retry_delay)
    raise RuntimeError("camera snapshot retry exhausted")


def run_snapshot(link: CameraLink, args: argparse.Namespace) -> None:
    metadata, payload = request_snapshot(link, args.size)
    output = args.output or (args.output_dir / default_name(metadata))
    save_snapshot(output, metadata, payload)


def run_set(link: CameraLink, args: argparse.Namespace) -> None:
    if args.brightness is not None:
        link.set_parameter("BR", args.brightness)
    if args.white_balance is not None:
        link.set_parameter("WB", args.white_balance)
    if args.snap:
        time.sleep(args.settle)
        metadata, payload = request_snapshot(link, args.size)
        output = args.output or (args.output_dir / default_name(metadata))
        save_snapshot(output, metadata, payload)


def run_auto(link: CameraLink, args: argparse.Namespace) -> None:
    records = []
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for brightness in args.brightness:
        link.set_parameter("BR", brightness)
        for white_balance in args.white_balance:
            link.set_parameter("WB", white_balance)
            time.sleep(args.settle)
            metadata, payload = request_snapshot(link, args.size)
            path, metrics = save_snapshot(
                args.output_dir / default_name(metadata), metadata, payload
            )
            records.append(
                {
                    **metadata,
                    **metrics,
                    "path": path,
                    "image": Image.open(path).copy(),
                }
            )

    records.sort(key=lambda record: record["score"])
    csv_path = args.output_dir / "camera_sweep.csv"
    fieldnames = [key for key in records[0] if key != "image"]
    with csv_path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for record in records:
            writer.writerow({key: record[key] for key in fieldnames})

    make_contact_sheet(records, args.output_dir / "camera_sweep.bmp")
    best = records[0]
    print(
        f"best estimate: BR={best['brightness']} WB=0x{best['white_balance']:02X} "
        f"score={best['score']:.1f} (bright neutral-area heuristic)"
    )
    if args.apply_best:
        link.set_parameter("BR", best["brightness"])
        link.set_parameter("WB", best["white_balance"])
        time.sleep(args.settle)
        metadata, payload = request_snapshot(link, "FULL")
        save_snapshot(args.output_dir / "camera_best_full.bmp", metadata, payload)
        print("best estimate applied to the camera")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM10", help="DAS JDS virtual COM port")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--output-dir", type=Path, default=Path("Debug/camera_serial")
    )
    subparsers = parser.add_subparsers(dest="action", required=True)

    subparsers.add_parser("status", help="read current camera-link status")
    subparsers.add_parser("retry", help="retry camera initialization")

    diag = subparsers.add_parser("diag", help="sample camera pipeline counters twice")
    diag.add_argument("--interval", type=float, default=2.0)

    snapshot = subparsers.add_parser("snap", help="request one frame")
    snapshot.add_argument("--size", choices=("FULL", "HALF", "QTR"), default="FULL")
    snapshot.add_argument("--output", type=Path)

    tune = subparsers.add_parser("set", help="set parameters without continuous image transfer")
    tune.add_argument("--brightness", type=parse_int)
    tune.add_argument("--white-balance", type=parse_int)
    tune.add_argument("--snap", action="store_true")
    tune.add_argument("--size", choices=("FULL", "HALF", "QTR"), default="FULL")
    tune.add_argument("--settle", type=float, default=1.0)
    tune.add_argument("--output", type=Path)

    auto = subparsers.add_parser("auto", help="scan fixed-WB/brightness candidates")
    auto.add_argument(
        "--brightness", type=parse_int, nargs="+", default=[100, 130, 160]
    )
    auto.add_argument(
        "--white-balance",
        type=parse_int,
        nargs="+",
        default=[0x68, 0x74, 0x80, 0x8C, 0x98],
    )
    auto.add_argument("--size", choices=("HALF", "QTR"), default="QTR")
    auto.add_argument("--settle", type=float, default=0.8)
    auto.add_argument("--apply-best", action="store_true")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        with WinSerial(args.port, args.baud) as serial:
            link = CameraLink(serial)
            if args.action == "status":
                print(link.command("CAM GET"))
            elif args.action == "retry":
                print(link.command("CAM RETRY", timeout=8.0))
            elif args.action == "diag":
                run_diag(link, args)
            elif args.action == "snap":
                run_snapshot(link, args)
            elif args.action == "set":
                run_set(link, args)
            elif args.action == "auto":
                run_auto(link, args)
        return 0
    except Exception as exc:
        print(f"camera debug failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

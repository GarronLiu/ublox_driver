#!/usr/bin/env python3

import argparse
import socket
import sys
import threading
import time


RTCM3_PREAMBLE = 0xD3
RTCM3_HEADER_LEN = 3
RTCM3_CRC_LEN = 3

RTCM_MESSAGE_NAMES = {
    1005: "Station ARP",
    1006: "Station ARP + height",
    1007: "Antenna descriptor",
    1008: "Antenna + serial",
    1019: "GPS ephemeris",
    1020: "GLONASS ephemeris",
    1033: "Receiver/antenna descriptor",
    1042: "BeiDou ephemeris",
    1044: "QZSS ephemeris",
    1045: "Galileo F/NAV ephemeris",
    1046: "Galileo I/NAV ephemeris",
    1230: "GLONASS code-phase bias",
}

MSM_CONSTELLATIONS = {
    107: "GPS",
    108: "GLO",
    109: "GAL",
    110: "SBAS",
    111: "QZSS",
    112: "BDS",
    113: "NAVIC",
}


class ClientHub:
    def __init__(self):
        self._clients = []
        self._lock = threading.Lock()

    def add(self, client, address):
        client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        client.settimeout(0.2)
        with self._lock:
            self._clients.append(client)
        print("RTCM client connected: {}:{}".format(address[0], address[1]), flush=True)

    def broadcast(self, data):
        dead_clients = []
        with self._lock:
            for client in self._clients:
                try:
                    client.sendall(data)
                except OSError:
                    dead_clients.append(client)
            for client in dead_clients:
                self._clients.remove(client)
                try:
                    client.close()
                except OSError:
                    pass
        return len(self._clients)

    def count(self):
        with self._lock:
            return len(self._clients)


class StatusTracker:
    def __init__(self):
        self.start_time = time.time()
        self.total_frames = 0
        self.total_bytes = 0
        self.invalid_frames = 0
        self.last_frame_time = None
        self.last_type = None
        self.last_name = "none"
        self.last_station_id = None
        self.ref_station = None
        self.msm = {}
        self.type_counts = {}

    def record_invalid_frame(self):
        self.invalid_frames += 1

    def record_frame(self, frame):
        payload = frame[RTCM3_HEADER_LEN:-RTCM3_CRC_LEN]
        msg_type = getbitu(payload, 0, 12)
        now = time.time()

        self.total_frames += 1
        self.total_bytes += len(frame)
        self.last_frame_time = now
        self.last_type = msg_type
        self.last_name = describe_rtcm_type(msg_type)
        self.type_counts[msg_type] = self.type_counts.get(msg_type, 0) + 1

        station_id = parse_station_id(payload)
        if station_id is not None:
            self.last_station_id = station_id

        ref_station = parse_reference_station(payload)
        if ref_station is not None:
            self.ref_station = ref_station

        msm = parse_msm_header(payload)
        if msm is not None:
            self.msm[msm["constellation"]] = msm

    def rates(self):
        elapsed = max(time.time() - self.start_time, 1e-3)
        return self.total_frames / elapsed, self.total_bytes * 8.0 / elapsed / 1000.0

    def recent_types(self, limit=8):
        items = sorted(self.type_counts.items(), key=lambda item: item[1], reverse=True)
        return ", ".join("{}:{}".format(msg_type, count)
                         for msg_type, count in items[:limit]) or "none"

    def render(self, args, client_count):
        now = time.time()
        uptime = format_duration(now - self.start_time)
        frame_rate, kbps = self.rates()
        last_age = "never"
        if self.last_frame_time is not None:
            last_age = "{:.1f}s ago".format(now - self.last_frame_time)

        lines = [
            "RTCM Serial Bridge",
            "==================",
            "Serial: {} @ {} baud".format(args.serial_port, args.baudrate),
            "TCP:    {}:{}  clients: {}".format(args.bind_host, args.bind_port, client_count),
            "Uptime: {}  frames: {}  invalid: {}".format(
                uptime, self.total_frames, self.invalid_frames),
            "Rate:   {:.1f} frames/s  {:.1f} kbit/s".format(frame_rate, kbps),
            "Last:   {} ({})  {}".format(self.last_type or "none", self.last_name, last_age),
            "Station ID: {}".format(self.last_station_id if self.last_station_id is not None else "unknown"),
            "",
            "Reference Station",
            "-----------------",
        ]

        if self.ref_station is None:
            lines.append("No RTCM 1005/1006 reference-station frame observed yet.")
        else:
            ref = self.ref_station
            lines.extend([
                "Type {}  Station {}  ITRF {}".format(
                    ref["type"], ref["station_id"], ref["itrf_year"]),
                "ECEF X/Y/Z: {:.4f}, {:.4f}, {:.4f} m".format(
                    ref["ecef_x"], ref["ecef_y"], ref["ecef_z"]),
                "Antenna height: {}".format(
                    "{:.4f} m".format(ref["antenna_height"])
                    if ref["antenna_height"] is not None else "not provided"),
            ])

        lines.extend([
            "",
            "MSM Observation Activity",
            "------------------------",
        ])
        if not self.msm:
            lines.append("No MSM observation frame observed yet.")
        else:
            for constellation in sorted(self.msm.keys()):
                msm = self.msm[constellation]
                age = now - msm["time"]
                lines.append("{} type {}: sats {:2d}, signals {:2d}, cells {:3d}, {:.1f}s ago".format(
                    constellation, msm["type"], msm["satellites"], msm["signals"],
                    msm["cells"], age))

        lines.extend([
            "",
            "Top RTCM types: {}".format(self.recent_types()),
            "",
            "Ctrl-C to stop. Use --no-dashboard for plain log output.",
        ])
        return "\n".join(lines)


def crc24q(data):
    crc = 0
    for byte in data:
        crc ^= byte << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF


def getbitu(data, pos, length):
    value = 0
    for bit_index in range(pos, pos + length):
        byte_index = bit_index // 8
        if byte_index >= len(data):
            return 0
        value = (value << 1) | ((data[byte_index] >> (7 - bit_index % 8)) & 1)
    return value


def getbits(data, pos, length):
    value = getbitu(data, pos, length)
    if length <= 0 or not (value & (1 << (length - 1))):
        return value
    return value - (1 << length)


def count_bits(value, width):
    count = 0
    for _ in range(width):
        count += value & 1
        value >>= 1
    return count


def describe_rtcm_type(msg_type):
    if msg_type in RTCM_MESSAGE_NAMES:
        return RTCM_MESSAGE_NAMES[msg_type]
    family = msg_type // 10
    if family in MSM_CONSTELLATIONS and 1 <= msg_type % 10 <= 7:
        return "{} MSM{}".format(MSM_CONSTELLATIONS[family], msg_type % 10)
    return "unknown"


def parse_station_id(payload):
    if len(payload) < 3:
        return None
    return getbitu(payload, 12, 12)


def parse_reference_station(payload):
    msg_type = getbitu(payload, 0, 12)
    if msg_type not in (1005, 1006):
        return None
    if len(payload) * 8 < 152:
        return None

    antenna_height = None
    if msg_type == 1006 and len(payload) * 8 >= 168:
        antenna_height = getbitu(payload, 152, 16) * 0.0001

    return {
        "type": msg_type,
        "station_id": getbitu(payload, 12, 12),
        "itrf_year": getbitu(payload, 24, 6),
        "ecef_x": getbits(payload, 34, 38) * 0.0001,
        "ecef_y": getbits(payload, 74, 38) * 0.0001,
        "ecef_z": getbits(payload, 114, 38) * 0.0001,
        "antenna_height": antenna_height,
    }


def parse_msm_header(payload):
    msg_type = getbitu(payload, 0, 12)
    family = msg_type // 10
    msm_id = msg_type % 10
    if family not in MSM_CONSTELLATIONS or not (1 <= msm_id <= 7):
        return None
    if len(payload) * 8 < 169:
        return None

    sat_mask = getbitu(payload, 73, 64)
    sig_mask = getbitu(payload, 137, 32)
    satellites = count_bits(sat_mask, 64)
    signals = count_bits(sig_mask, 32)
    cell_bits = satellites * signals
    if len(payload) * 8 < 169 + cell_bits:
        cells = 0
    else:
        cells = count_bits(getbitu(payload, 169, cell_bits), cell_bits)

    return {
        "type": msg_type,
        "constellation": MSM_CONSTELLATIONS[family],
        "station_id": getbitu(payload, 12, 12),
        "satellites": satellites,
        "signals": signals,
        "cells": cells,
        "time": time.time(),
    }


def format_duration(seconds):
    seconds = int(seconds)
    hours = seconds // 3600
    minutes = (seconds % 3600) // 60
    secs = seconds % 60
    if hours:
        return "{}h{:02d}m{:02d}s".format(hours, minutes, secs)
    if minutes:
        return "{}m{:02d}s".format(minutes, secs)
    return "{}s".format(secs)


def read_exact(port, size):
    data = port.read(size)
    if len(data) != size:
        raise TimeoutError("serial timeout while reading {} bytes".format(size))
    return data


def read_rtcm3_frame(port, validate_crc):
    while True:
        byte = read_exact(port, 1)[0]
        if byte == RTCM3_PREAMBLE:
            break

    header_tail = read_exact(port, RTCM3_HEADER_LEN - 1)
    header = bytes([RTCM3_PREAMBLE]) + header_tail
    payload_len = ((header[1] & 0x03) << 8) | header[2]
    if payload_len > 1023:
        return None

    payload_and_crc = read_exact(port, payload_len + RTCM3_CRC_LEN)
    frame = header + payload_and_crc

    if validate_crc:
        received_crc = int.from_bytes(frame[-RTCM3_CRC_LEN:], byteorder="big")
        calculated_crc = crc24q(frame[:-RTCM3_CRC_LEN])
        if received_crc != calculated_crc:
            return None

    return frame


def accept_clients(bind_host, bind_port, hub, stop_event):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((bind_host, bind_port))
    server.listen(8)
    server.settimeout(0.5)
    print("RTCM TCP server listening on {}:{}".format(bind_host, bind_port), flush=True)

    try:
        while not stop_event.is_set():
            try:
                client, address = server.accept()
            except socket.timeout:
                continue
            hub.add(client, address)
    finally:
        server.close()


def parse_args():
    parser = argparse.ArgumentParser(
        description="Read RTCM3 from a base-station serial port and forward it over TCP."
    )
    parser.add_argument("--serial-port", default="/dev/ttyACM0",
                        help="base-station receiver serial port")
    parser.add_argument("--baudrate", type=int, default=115200,
                        help="base-station receiver baud rate")
    parser.add_argument("--bind-host", default="0.0.0.0",
                        help="TCP bind address, use 0.0.0.0 for LAN access")
    parser.add_argument("--bind-port", type=int, default=3503,
                        help="TCP port exposed to rover receivers")
    parser.add_argument("--serial-timeout", type=float, default=1.0,
                        help="serial read timeout in seconds")
    parser.add_argument("--no-crc-check", action="store_true",
                        help="forward RTCM3 frames without CRC-24Q validation")
    parser.add_argument("--quiet", action="store_true",
                        help="suppress periodic transfer statistics")
    parser.add_argument("--dashboard", action="store_true",
                        help="force terminal dashboard output")
    parser.add_argument("--no-dashboard", action="store_true",
                        help="disable terminal dashboard output")
    parser.add_argument("--dashboard-mode", choices=("auto", "true", "false"),
                        default="auto",
                        help="dashboard mode for launch files")
    parser.add_argument("--status-interval", type=float, default=1.0,
                        help="status refresh interval in seconds")
    args, unknown_args = parser.parse_known_args()
    unknown_args = [arg for arg in unknown_args if not arg.startswith("__")]
    if unknown_args:
        parser.error("unrecognized arguments: {}".format(" ".join(unknown_args)))
    return args


def dashboard_is_enabled(args):
    if args.dashboard:
        return True
    if args.no_dashboard:
        return False
    if args.dashboard_mode == "true":
        return True
    if args.dashboard_mode == "false":
        return False
    return sys.stdout.isatty() and not args.quiet


def main():
    args = parse_args()
    try:
        import serial
    except ImportError:
        print("Missing dependency: pyserial. Install it with `sudo apt install python3-serial`.",
              flush=True)
        return 1

    hub = ClientHub()
    stop_event = threading.Event()
    accept_thread = threading.Thread(
        target=accept_clients,
        args=(args.bind_host, args.bind_port, hub, stop_event),
        daemon=True,
    )
    accept_thread.start()

    frame_count = 0
    byte_count = 0
    last_report_time = time.time()
    status = StatusTracker()
    last_dashboard_time = 0.0
    validate_crc = not args.no_crc_check
    dashboard_enabled = dashboard_is_enabled(args)

    try:
        with serial.Serial(args.serial_port, args.baudrate,
                           timeout=args.serial_timeout) as port:
            print("Reading RTCM3 from {} at {} baud".format(
                args.serial_port, args.baudrate), flush=True)
            while True:
                try:
                    frame = read_rtcm3_frame(port, validate_crc)
                except TimeoutError:
                    now = time.time()
                    if dashboard_enabled and now - last_dashboard_time >= args.status_interval:
                        print("\033[2J\033[H" + status.render(args, hub.count()), flush=True)
                        last_dashboard_time = now
                    continue

                if frame is None:
                    status.record_invalid_frame()
                    now = time.time()
                    if dashboard_enabled and now - last_dashboard_time >= args.status_interval:
                        print("\033[2J\033[H" + status.render(args, hub.count()), flush=True)
                        last_dashboard_time = now
                    continue

                hub.broadcast(frame)
                frame_count += 1
                byte_count += len(frame)
                status.record_frame(frame)

                now = time.time()
                if dashboard_enabled and now - last_dashboard_time >= args.status_interval:
                    print("\033[2J\033[H" + status.render(args, hub.count()), flush=True)
                    last_dashboard_time = now
                elif not args.quiet and now - last_report_time >= 5.0:
                    frame_rate, kbps = status.rates()
                    print("Forwarded {} RTCM frames, {} bytes, {} clients, "
                          "{:.1f} frames/s, {:.1f} kbit/s, last type {} ({})".format(
                              frame_count, byte_count, hub.count(), frame_rate, kbps,
                              status.last_type or "none", status.last_name),
                          flush=True)
                    last_report_time = now
    except KeyboardInterrupt:
        pass
    except serial.SerialException as exc:
        print("Failed to open/read serial port {}: {}".format(args.serial_port, exc),
              flush=True)
        return 1
    finally:
        stop_event.set()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

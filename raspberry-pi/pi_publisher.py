import argparse
import struct
import threading
import time
import subprocess
import os
import sys
import math
import fcntl

import zmq

VENV_PYTHON = "/home/ryanpage/Desktop/PIZMQPUB/.venv/bin/python"
if (
    os.path.exists(VENV_PYTHON)
    and sys.executable != VENV_PYTHON
    and not os.environ.get("PI_PUBLISHER_NO_REEXEC")
):
    os.execv(VENV_PYTHON, [VENV_PYTHON] + sys.argv)

MAGIC = 0x50493034  # "PI04"
VERSION = 0x0100

MSG_REAR_CAM = 0x0001
MSG_RADAR_L = 0x0002
MSG_RADAR_R = 0x0003
MSG_IMU = 0x0004
MSG_HEARTBEAT = 0x0010
MSG_DISCOVERY_REQ = 0x0020
MSG_DISCOVERY_RSP = 0x0021

PORT_REAR_CAM = 5555
PORT_RADAR_L = 5556
PORT_RADAR_R = 5557
PORT_IMU = 5558
PORT_CONTROL = 5559

DEVICE_TYPE_CAMERA = 1
DEVICE_TYPE_RADAR = 2
DEVICE_TYPE_IMU = 3

MOUNT_REAR_CAM = 3
MOUNT_REAR_RADAR_L = 5
MOUNT_REAR_RADAR_R = 6
MOUNT_IMU = 7

STATUS_OK = 0

HEADER_STRUCT = struct.Struct("<IHHIIQII")

# --- REFACTORED PAYLOAD STRUCTS ---
RCW_STRUCT = struct.Struct("<ffi")  # ttc, range_m, class_id
BSD_STRUCT = struct.Struct("<B")   # presence flag (0/1)
IMU_STRUCT = struct.Struct("<ff")  # pitch (theta), roll (phi)
# ----------------------------------

HEARTBEAT_STRUCT = struct.Struct("<QBBBBiI")
DISCOVERY_HEADER_STRUCT = struct.Struct("<B3x")
DEVICE_INFO_STRUCT = struct.Struct("<BBH16s")


class HealthState:
    def __init__(self):
        self._lock = threading.Lock()
        self._last_ok = {
            "cam": 0.0,
            "radar_l": 0.0,
            "radar_r": 0.0,
            "imu": 0.0,
        }

    def mark_ok(self, name):
        with self._lock:
            self._last_ok[name] = time.monotonic()

    def is_ok(self, name, timeout=2.0):
        with self._lock:
            last = self._last_ok.get(name, 0.0)
        if last <= 0.0:
            return False
        return (time.monotonic() - last) <= timeout


class StatsState:
    def __init__(self):
        self._lock = threading.Lock()
        self._counts = {
            "cam": 0,
            "radar_l": 0,
            "radar_r": 0,
            "imu": 0,
        }
        self._radar_diag = {
            "radar_l": {
                "valid_lines": 0,
                "blank_lines": 0,
                "invalid_lines": 0,
                "state_transitions": 0,
                "publish_attempts": 0,
                "publish_success": 0,
                "publish_eagain": 0,
            },
            "radar_r": {
                "valid_lines": 0,
                "blank_lines": 0,
                "invalid_lines": 0,
                "state_transitions": 0,
                "publish_attempts": 0,
                "publish_success": 0,
                "publish_eagain": 0,
            },
        }

    def increment(self, name):
        with self._lock:
            if name in self._counts:
                self._counts[name] += 1

    def snapshot(self):
        with self._lock:
            return dict(self._counts)

    def increment_radar_diag(self, key, metric, amount=1):
        with self._lock:
            if key in self._radar_diag and metric in self._radar_diag[key]:
                self._radar_diag[key][metric] += amount

    def radar_diag_snapshot(self):
        with self._lock:
            return {name: dict(metrics) for name, metrics in self._radar_diag.items()}


def build_header(msg_type, payload_size, sequence, timestamp_ns):
    return HEADER_STRUCT.pack(
        MAGIC,
        VERSION,
        msg_type,
        payload_size,
        0,
        timestamp_ns,
        sequence,
        0,
    )


def bind_addr(bind_ip, port):
    if not bind_ip or bind_ip in ("0.0.0.0", "*"):
        return f"tcp://*:{port}"
    return f"tcp://{bind_ip}:{port}"


def print_banner(bind_ip):
    print("[Pi Publisher] Bound to all ports")
    print(f"  RCW (Cam): {bind_addr(bind_ip, PORT_REAR_CAM)}")
    print(f"  Radar L:   {bind_addr(bind_ip, PORT_RADAR_L)}")
    print(f"  Radar R:   {bind_addr(bind_ip, PORT_RADAR_R)}")
    print(f"  IMU:       {bind_addr(bind_ip, PORT_IMU)}")
    print(f"  Control:   {bind_addr(bind_ip, PORT_CONTROL)}")
    print("\n" + "=" * 60)
    print("            PI SENSOR PUBLISHER - DISTRIBUTED EDGE MODE")
    print("=" * 60)
    print("  Press Ctrl+C to stop")
    print("=" * 60 + "\n")


def make_push_socket(ctx, bind_ip, port):
    sock = ctx.socket(zmq.PUSH)
    sock.setsockopt(zmq.SNDHWM, 10)
    sock.setsockopt(zmq.LINGER, 0)
    sock.bind(bind_addr(bind_ip, port))
    return sock


def parse_presence_token(raw_line):
    token = raw_line.strip()
    if token == "0":
        return 0
    if token == "1":
        return 1
    # Support DFHPD/DFD* CSV lines where field 2 is occupancy status.
    # Example: "$DFHPD,1, , , *"
    if "," in token:
        parts = [part.strip() for part in token.split(",")]
        if len(parts) >= 2 and parts[1] in ("0", "1"):
            return int(parts[1])
    return None


def update_bsd_hysteresis(state, consec_ones, consec_zeros, detection, on_count, off_count):
    if detection not in (0, 1):
        return state, consec_ones, consec_zeros, False

    transitioned = False
    if detection == 1:
        consec_ones += 1
        consec_zeros = 0
        if state == 0 and consec_ones >= on_count:
            state = 1
            transitioned = True
    else:
        consec_zeros += 1
        consec_ones = 0
        if state == 1 and consec_zeros >= off_count:
            state = 0
            transitioned = True

    return state, consec_ones, consec_zeros, transitioned


def publish_bsd_state(sock, msg_type, state, seq, stats, key):
    stats.increment_radar_diag(key, "publish_attempts")

    payload = BSD_STRUCT.pack(state)
    header = build_header(msg_type, len(payload), seq, time.time_ns())
    send_ok = True
    try:
        sock.send(header + payload, zmq.NOBLOCK)
    except zmq.Again:
        send_ok = False
        stats.increment_radar_diag(key, "publish_eagain")
    if send_ok:
        stats.increment_radar_diag(key, "publish_success")
    return seq + 1, send_ok


def rcw_worker(args, ctx, health, stats, stop_event):
    print("[RCW/Cam] Starting RCW thread (Stubbed for IPC)...")
    sock = make_push_socket(ctx, args.bind_ip, PORT_REAR_CAM)
    seq = 0
    loop_interval = 0.1 

    rcw_process = subprocess.Popen(
        ['./build/rcw'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        bufsize=1
    )
    
    fd = rcw_process.stdout.fileno()
    flags = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)

    while not stop_event.is_set():
        start_time = time.monotonic()

        try:
            raw_data = rcw_process.stdout.read()
            
            if raw_data:
                lines = raw_data.strip().split("\n")
                if lines:
                    rcw_message = lines[-1].split(",")
                    if len(rcw_message) >= 3:
                        ttc = float(rcw_message[0])
                        range = float(rcw_message[1])
                        class_id = int(rcw_message[2])
                            
                    payload = RCW_STRUCT.pack(ttc, range, class_id)
                    header = build_header(MSG_REAR_CAM, len(payload), seq, time.time_ns())
                    
                    sock.send(header + payload, zmq.NOBLOCK)
        except (IOError, TypeError, zmq.Again):
            pass
            
        seq += 1
        health.mark_ok("cam")  
        stats.increment("cam")

        elapsed = time.monotonic() - start_time
        if elapsed < loop_interval:
            time.sleep(loop_interval - elapsed)

    rcw_process.stdout.close()
    rcw_process.terminate()

    print("[RCW/Cam] Thread stopped")


def radar_worker(args, ctx, health, stats, stop_event, port, label, msg_type, key, port_override=None):
    print(f"[{label}] Starting BSD presence thread...")

    try:
        import serial
    except Exception as exc:
        print(f"[{label}] pyserial not available: {exc}")
        return

    port_num = port_override if port_override is not None else (
        PORT_RADAR_L if msg_type == MSG_RADAR_L else PORT_RADAR_R
    )
    sock = make_push_socket(ctx, args.bind_ip, port_num)

    try:
        ser = serial.Serial(port, args.radar_baud, timeout=0.1)
    except Exception as exc:
        print(f"[{label}] Failed to open {port}: {exc}")
        return

    seq = 0
    debounced_state = 0
    consec_ones = 0
    consec_zeros = 0
    on_count = max(1, int(args.radar_on_count))
    off_count = max(1, int(args.radar_off_count))
    publish_period = 1.0 / args.bsd_rate if args.bsd_rate > 0 else 0.1
    next_publish = time.monotonic()

    last_parse_debug = 0.0
    last_publish_debug = 0.0

    while not stop_event.is_set():
        raw = ser.readline().decode("ascii", errors="ignore")
        stripped = raw.strip()
        detected = parse_presence_token(stripped)
        now = time.monotonic()

        if detected is None:
            if stripped == "":
                stats.increment_radar_diag(key, "blank_lines")
            else:
                stats.increment_radar_diag(key, "invalid_lines")
                if args.radar_debug and (now - last_parse_debug) >= 1.0:
                    print(f"[{label}] invalid token: {repr(stripped)}")
                    last_parse_debug = now
        else:
            stats.increment_radar_diag(key, "valid_lines")
            health.mark_ok(key)

            prev_state = debounced_state
            debounced_state, consec_ones, consec_zeros, transitioned = update_bsd_hysteresis(
                debounced_state,
                consec_ones,
                consec_zeros,
                detected,
                on_count,
                off_count,
            )

            if args.radar_debug and (now - last_parse_debug) >= 1.0:
                print(
                    f"[{label}] parsed={detected} state={debounced_state} "
                    f"ones={consec_ones} zeros={consec_zeros}"
                )
                last_parse_debug = now

            if transitioned:
                stats.increment(key)
                stats.increment_radar_diag(key, "state_transitions")
                seq, sent_ok = publish_bsd_state(sock, msg_type, debounced_state, seq, stats, key)
                next_publish = now + publish_period
                if args.radar_debug:
                    outcome = "sent" if sent_ok else "drop(ZMQ full)"
                    print(f"[{label}] transition {prev_state}->{debounced_state} {outcome}")

        now = time.monotonic()
        if now >= next_publish:
            seq, sent_ok = publish_bsd_state(sock, msg_type, debounced_state, seq, stats, key)
            while next_publish <= now:
                next_publish += publish_period
            if args.radar_debug and (now - last_publish_debug) >= 1.0:
                outcome = "sent" if sent_ok else "drop(ZMQ full)"
                print(f"[{label}] periodic state={debounced_state} {outcome}")
                last_publish_debug = now

    try:
        ser.close()
    except Exception:
        pass
    try:
        sock.close(0)
    except Exception:
        pass
    print(f"[{label}] Thread stopped")



def imu_worker(args, ctx, health, stats, stop_event):
    # ---------------------------------------------------------------------------
    # IMU STUB — BNO08x replacement pending hardware delivery.
    # Sends pitch=0.0, roll=0.0 (perfectly level) at 50 Hz so the Jetson's
    # ground-plane fusion pipeline runs without modification or crashes.
    # Swap this function body back to the full BNO08x implementation once the
    # replacement sensor arrives.
    # ---------------------------------------------------------------------------
    print("[IMU] Starting IMU thread (STUB MODE)...")

    sock = make_push_socket(ctx, args.bind_ip, PORT_IMU)

    seq = 0
    period = 1.0 / args.imu_rate if args.imu_rate > 0 else 0.02
    next_tick = time.monotonic()

    # Safe stub values: vehicle reported as perfectly level.
    STUB_PITCH = 0.0  # radians — no forward/rearward tilt
    STUB_ROLL  = 0.0  # radians — no left/right lean

    while not stop_event.is_set():
        payload = IMU_STRUCT.pack(STUB_PITCH, STUB_ROLL)
        header = build_header(MSG_IMU, len(payload), seq, time.time_ns())

        try:
            sock.send(header + payload, zmq.NOBLOCK)
        except zmq.Again:
            pass

        seq += 1
        health.mark_ok("imu")
        stats.increment("imu")

        next_tick += period
        sleep_for = next_tick - time.monotonic()
        if sleep_for > 0:
            time.sleep(sleep_for)

    print("[IMU] Thread stopped")


def control_worker(args, ctx, health, stop_event):
    print("[Control] Starting control thread...")
    sock = ctx.socket(zmq.REP)
    sock.setsockopt(zmq.LINGER, 0)
    sock.setsockopt(zmq.RCVTIMEO, 1000)
    sock.bind(bind_addr(args.bind_ip, PORT_CONTROL))

    devices = [
        (DEVICE_TYPE_CAMERA, MOUNT_REAR_CAM, STATUS_OK, "CAM001"),
        (DEVICE_TYPE_RADAR, MOUNT_REAR_RADAR_L, STATUS_OK, "RAD001"),
        (DEVICE_TYPE_RADAR, MOUNT_REAR_RADAR_R, STATUS_OK, "RAD002"),
        (DEVICE_TYPE_IMU, MOUNT_IMU, STATUS_OK, "IMU001"),
    ]

    seq = 0
    start_time = time.monotonic()

    while not stop_event.is_set():
        try:
            msg = sock.recv(flags=0)
        except zmq.Again:
            continue
        except zmq.ZMQError:
            continue

        if len(msg) < HEADER_STRUCT.size:
            sock.send(b"")
            continue

        header = HEADER_STRUCT.unpack_from(msg, 0)
        magic, version, msg_type = header[0], header[1], header[2]
        if magic != MAGIC or version != VERSION:
            sock.send(b"")
            continue

        if msg_type == MSG_DISCOVERY_REQ:
            print("[Control] Received discovery request")
            payload = DISCOVERY_HEADER_STRUCT.pack(len(devices))
            for dev_type, mount_id, status, serial in devices:
                serial_bytes = serial.encode("ascii")[:15]
                serial_bytes = serial_bytes + b"\x00" * (16 - len(serial_bytes))
                payload += DEVICE_INFO_STRUCT.pack(dev_type, mount_id, status, serial_bytes)

            resp_header = build_header(MSG_DISCOVERY_RSP, len(payload), seq, time.time_ns())
            seq += 1
            sock.send(resp_header + payload)
            print("[Control] Sent discovery response")
            continue

        if msg_type == MSG_HEARTBEAT:
            uptime_ms = int((time.monotonic() - start_time) * 1000)
            payload = HEARTBEAT_STRUCT.pack(
                uptime_ms,
                1 if health.is_ok("cam") else 0,
                1 if health.is_ok("radar_l") else 0,
                1 if health.is_ok("radar_r") else 0,
                1 if health.is_ok("imu") else 0,
                0,  
                0,
            )
            resp_header = build_header(MSG_HEARTBEAT, len(payload), seq, time.time_ns())
            seq += 1
            sock.send(resp_header + payload)
            continue

        sock.send(b"")
    print("[Control] Thread stopped")


def stats_worker(args, stats, stop_event):
    interval = args.stats_interval
    if interval <= 0:
        return
    while not stop_event.is_set():
        time.sleep(interval)
        snapshot = stats.snapshot()
        radar_diag = stats.radar_diag_snapshot()
        print(
            f"[Stats] RCW:{snapshot['cam']} IMU:{snapshot['imu']} "
            f"RadarL:{snapshot['radar_l']} RadarR:{snapshot['radar_r']}"
        )
        print(
            "[RadarDiag] RadarL "
            f"valid={radar_diag['radar_l']['valid_lines']} "
            f"blank={radar_diag['radar_l']['blank_lines']} "
            f"invalid={radar_diag['radar_l']['invalid_lines']} "
            f"transitions={radar_diag['radar_l']['state_transitions']} "
            f"pub={radar_diag['radar_l']['publish_attempts']} "
            f"ok={radar_diag['radar_l']['publish_success']} "
            f"eagain={radar_diag['radar_l']['publish_eagain']}"
        )
        print(
            "[RadarDiag] RadarR "
            f"valid={radar_diag['radar_r']['valid_lines']} "
            f"blank={radar_diag['radar_r']['blank_lines']} "
            f"invalid={radar_diag['radar_r']['invalid_lines']} "
            f"transitions={radar_diag['radar_r']['state_transitions']} "
            f"pub={radar_diag['radar_r']['publish_attempts']} "
            f"ok={radar_diag['radar_r']['publish_success']} "
            f"eagain={radar_diag['radar_r']['publish_eagain']}"
        )


def parse_args():
    parser = argparse.ArgumentParser(description="Pi publisher for Jetson integration")
    parser.add_argument("--bind-ip", default="0.0.0.0")
    parser.add_argument("--cam-dev", type=int, default=0)
    parser.add_argument("--radar-l", default="/dev/ttyAMA3")
    parser.add_argument("--radar-r", default="/dev/ttyAMA4")
    parser.add_argument("--radar-baud", type=int, default=115200)
    parser.add_argument("--bsd-rate", type=float, default=10.0, help="BSD publish rate in Hz")
    parser.add_argument(
        "--radar-on-count",
        type=int,
        default=2,
        help="Consecutive valid '1' samples required to assert BSD",
    )
    parser.add_argument(
        "--radar-off-count",
        type=int,
        default=1,
        help="Consecutive valid '0' samples required to clear BSD",
    )
    parser.add_argument("--radar-debug", action="store_true", help="Enable radar parse/state debug logs")
    parser.add_argument("--imu-rate", type=float, default=50.0)
    parser.add_argument("--stats-interval", type=float, default=1.0)
    parser.add_argument(
        "--control-mode",
        choices=["rep", "off"],
        default="rep",
        help="rep binds REP on port 5559 for discovery + heartbeat",
    )
    args = parser.parse_args()
    if args.bsd_rate <= 0:
        parser.error("--bsd-rate must be > 0")
    if args.radar_on_count <= 0:
        parser.error("--radar-on-count must be > 0")
    if args.radar_off_count <= 0:
        parser.error("--radar-off-count must be > 0")
    return args


def main():
    args = parse_args()
    ctx = zmq.Context.instance()
    stop_event = threading.Event()
    health = HealthState()
    stats = StatsState()

    print_banner(args.bind_ip)

    threads = [
        threading.Thread(target=rcw_worker, args=(args, ctx, health, stats, stop_event)),
        threading.Thread(
            target=radar_worker,
            args=(args, ctx, health, stats, stop_event, args.radar_l, "RadarL", MSG_RADAR_L, "radar_l"),
        ),
        threading.Thread(
            target=radar_worker,
            args=(args, ctx, health, stats, stop_event, args.radar_r, "RadarR", MSG_RADAR_R, "radar_r"),
        ),
        threading.Thread(target=imu_worker, args=(args, ctx, health, stats, stop_event)),
        threading.Thread(target=stats_worker, args=(args, stats, stop_event)),
    ]

    if args.control_mode == "rep":
        threads.append(threading.Thread(target=control_worker, args=(args, ctx, health, stop_event)))

    for thread in threads:
        thread.daemon = True
        thread.start()

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        stop_event.set()

    for thread in threads:
        thread.join(timeout=1.0)


if __name__ == "__main__":
    main()

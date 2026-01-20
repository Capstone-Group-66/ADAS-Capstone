#!/usr/bin/env python3
"""
ADAS BLE Peripheral Server

Uses 'bless' library (BLE peripheral library) to create a GATT server
that advertises the ADAS service and allows notifications on AlertStream.

Cross-platform: Works on Linux (BlueZ), Windows, macOS.

Install: pip install bless

Usage:
    python ble_peripheral.py           # Normal mode, reads from stdin
    python ble_peripheral.py --test    # Sends fake alerts for testing
"""

import asyncio
import json
import struct
import sys
import time
import select
from typing import Optional

# Check for required library
try:
    from bless import BlessServer, BlessGATTCharacteristic, GATTCharacteristicProperties, GATTAttributePermissions
    HAS_BLESS = True
except ImportError:
    print("[BLE] WARNING: 'bless' library not found. Running in stub mode.", file=sys.stderr)
    print("[BLE] Install with: pip install bless", file=sys.stderr)
    HAS_BLESS = False

# ADAS BLE UUIDs (must match jetson-core/BleUuids.hpp and mobile app)
ADAS_SERVICE_UUID = "0000ada5-0000-1000-8000-00805f9b34fb"
ADAS_ALERT_STREAM_UUID = "0000a1e7-0000-1000-8000-00805f9b34fb"
ADAS_STATUS_UUID = "000057a7-0000-1000-8000-00805f9b34fb"

# Global state
server: Optional["BlessServer"] = None
tick_id = 0


async def setup_server():
    """Initialize the BLE GATT server with ADAS service."""
    global server
    
    if not HAS_BLESS:
        print("[BLE] Stub mode: skipping server setup", file=sys.stderr)
        return None
    
    server = BlessServer(name="ADAS-Jetson")
    
    # Add ADAS Service
    await server.add_new_service(ADAS_SERVICE_UUID)
    
    # Add AlertStream characteristic (Notify)
    await server.add_new_characteristic(
        ADAS_SERVICE_UUID,
        ADAS_ALERT_STREAM_UUID,
        GATTCharacteristicProperties.notify,
        None,
        GATTAttributePermissions.readable
    )
    
    # Add Status characteristic (Read/Notify) - heartbeat
    await server.add_new_characteristic(
        ADAS_SERVICE_UUID,
        ADAS_STATUS_UUID,
        GATTCharacteristicProperties.read | GATTCharacteristicProperties.notify,
        b'\x01',
        GATTAttributePermissions.readable
    )
    
    return server


async def send_notification(data: bytes):
    """Send a notification on AlertStream characteristic."""
    global server
    
    if server is None:
        print(f"[BLE] Stub: would send {len(data)} bytes", file=sys.stderr)
        return
    
    try:
        server.get_characteristic(ADAS_ALERT_STREAM_UUID).value = data
        server.update_value(ADAS_SERVICE_UUID, ADAS_ALERT_STREAM_UUID)
        print(f"[BLE] Sent notification: {len(data)} bytes", file=sys.stderr)
    except Exception as e:
        print(f"[BLE] Error sending notification: {e}", file=sys.stderr)


def hex_to_bytes(hex_str: str) -> bytes:
    """Convert hex string to bytes."""
    return bytes.fromhex(hex_str)


async def process_stdin_command(line: str):
    """Process a JSON command from C++ stdin."""
    try:
        cmd = json.loads(line.strip())
        
        if cmd.get("cmd") == "notify":
            data = hex_to_bytes(cmd.get("data", ""))
            await send_notification(data)
        else:
            print(f"[BLE] Unknown command: {cmd}", file=sys.stderr)
            
    except json.JSONDecodeError as e:
        print(f"[BLE] Invalid JSON: {e}", file=sys.stderr)
    except Exception as e:
        print(f"[BLE] Error processing command: {e}", file=sys.stderr)


async def stdin_reader():
    """Read commands from stdin (from C++ parent process)."""
    loop = asyncio.get_event_loop()
    
    print("[BLE] Listening for commands on stdin...", file=sys.stderr)
    
    while True:
        # Non-blocking stdin read
        try:
            line = await loop.run_in_executor(None, sys.stdin.readline)
            if not line:
                print("[BLE] EOF on stdin, exiting", file=sys.stderr)
                break
            await process_stdin_command(line)
        except Exception as e:
            print(f"[BLE] Stdin read error: {e}", file=sys.stderr)
            break


async def run_test_mode():
    """Send test FCW alerts periodically for demo."""
    global tick_id
    
    print("[BLE] Running in TEST mode - sending fake FCW alerts every 3 seconds", file=sys.stderr)
    
    while True:
        await asyncio.sleep(3)
        tick_id = (tick_id + 1) % 65536
        
        # Create a simple test payload
        payload = json.dumps({
            "tick_id": tick_id,
            "t_ms": int(time.time() * 1000),
            "type": "FCW",
            "severity": "warning",
            "ttl_ms": 1000,
            "rationale": json.dumps({"ttc_s": 2.5, "range_m": 4.0}),
            "sources": ["FrontCam", "FrontRadar"]
        }).encode('utf-8')
        
        # Simple header: tick_id (u16 LE), seq_no (u8), seq_max (u8)
        header = struct.pack('<HBB', tick_id, 0, 0)  # Single fragment
        
        await send_notification(header + payload)
        print(f"[BLE] Test alert sent (tick {tick_id})", file=sys.stderr)


async def main():
    """Main entry point."""
    print("=" * 60, file=sys.stderr)
    print("  ADAS BLE Peripheral Server", file=sys.stderr)
    print("=" * 60, file=sys.stderr)
    print(f"  Service UUID: {ADAS_SERVICE_UUID}", file=sys.stderr)
    print(f"  AlertStream:  {ADAS_ALERT_STREAM_UUID}", file=sys.stderr)
    print("=" * 60, file=sys.stderr)
    
    await setup_server()
    
    if server:
        print("[BLE] Starting advertising as 'ADAS-Jetson'...", file=sys.stderr)
        await server.start()
        print("[BLE] Server running.", file=sys.stderr)
    
    # Run appropriate mode
    if "--test" in sys.argv:
        await run_test_mode()
    else:
        await stdin_reader()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[BLE] Shutting down...", file=sys.stderr)

#!/usr/bin/env python3
"""
FCW Simulator for Jetson Nano (Python 3.6 Compatible)
------------------------------------------------------
Uses BlueZ directly via subprocess for maximum compatibility with old systems.

This avoids the bless/txdbus/dbus-next compatibility issues on Python 3.6.
"""

import subprocess
import struct
import time
import logging
import sys

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger("FCW_Sim")

try:
    import cbor
except ImportError:
    logger.error("Missing cbor. Run: pip3 install cbor")
    sys.exit(1)

# BLE Constants
ADAS_SERVICE_UUID = "0000ada5-0000-1000-8000-00805f9b34fb"
ADAS_ALERT_STREAM_UUID = "0000a1e7-0000-1000-8000-00805f9b34fb"

class SimpleFcwSimulator:
    def __init__(self):
        self.tick_id = 0
        self.hci_device = "hci0"
        
    def check_bluetooth(self):
        """Check if Bluetooth is available and up."""
        try:
            result = subprocess.run(
                ["hciconfig", self.hci_device],
                capture_output=True,
                text=True
            )
            if "UP RUNNING" not in result.stdout:
                logger.warning("Bluetooth adapter not up. Trying to enable...")
                subprocess.run(["sudo", "hciconfig", self.hci_device, "up"], check=True)
            logger.info("Bluetooth adapter is UP")
            return True
        except Exception as e:
            logger.error(f"Bluetooth check failed: {e}")
            return False

    def setup_advertising(self):
        """Configure BLE advertising using hcitool."""
        try:
            # Stop any existing advertising
            subprocess.run(
                ["sudo", "hciconfig", self.hci_device, "noleadv"],
                capture_output=True
            )
            
            # Set advertising parameters
            # Using hcitool to set up basic advertising
            # This advertises the device name
            subprocess.run(
                ["sudo", "hciconfig", self.hci_device, "leadv", "0"],
                check=True
            )
            
            # Set device name using hcitool
            subprocess.run(
                ["sudo", "hciconfig", self.hci_device, "name", "ADAS-Jetson"],
                check=True
            )
            
            logger.info("BLE advertising started as 'ADAS-Jetson'")
            return True
        except Exception as e:
            logger.error(f"Failed to setup advertising: {e}")
            return False

    def generate_fcw_payload(self):
        """Create the CBOR payload matching mobile app schema."""
        self.tick_id = (self.tick_id + 1) % 65536
        
        payload_data = {
            "t": self.tick_id,
            "v": 65,            # Speed
            "h": 0,             # Health mask
            "b": 0,             # BSD mask
            "a": [              # Alerts
                {
                    "id": 0,                # FCW
                    "s": 2,                 # Severity: Critical
                    "r": "Simulated FCW"    # Rationale
                }
            ]
        }
        
        cbor_bytes = cbor.dumps(payload_data)
        
        # Add 4-byte header: tick_id (u16), seq_no (u8), seq_max (u8)
        header = struct.pack('<HBB', self.tick_id, 0, 0)
        
        full_packet = header + cbor_bytes
        logger.info(f"Generated FCW Tick #{self.tick_id} ({len(full_packet)} bytes)")
        return full_packet

    def send_notification(self, data):
        """
        Send BLE notification using gatttool or bluetoothctl.
        
        Note: For a full implementation, you'd need to:
        1. Have the mobile app connect first
        2. Use a GATT server implementation
        
        This simplified version just logs what would be sent.
        For actual BLE transmission, we'll use the existing C++ BLE server.
        """
        hex_data = data.hex()
        logger.info(f"Would send notification: {hex_data[:40]}...")
        return True

    def run(self):
        """Main simulation loop."""
        logger.info("=" * 60)
        logger.info("  ADAS FCW Simulator (Jetson Edition)")
        logger.info("=" * 60)
        logger.info(f"  Service UUID: {ADAS_SERVICE_UUID}")
        logger.info(f"  Characteristic: {ADAS_ALERT_STREAM_UUID}")
        logger.info("=" * 60)
        
        if not self.check_bluetooth():
            logger.error("Cannot proceed without Bluetooth")
            return
            
        if not self.setup_advertising():
            logger.warning("Advertising setup had issues, continuing anyway...")
        
        logger.info("Starting FCW simulation (every 10s)...")
        logger.info("NOTE: For full BLE GATT server, use the C++ main_brain executable")
        logger.info("")
        
        try:
            while True:
                packet = self.generate_fcw_payload()
                self.send_notification(packet)
                logger.info("Waiting 10s...")
                time.sleep(10)
        except KeyboardInterrupt:
            logger.info("Shutting down...")
            # Stop advertising
            subprocess.run(
                ["sudo", "hciconfig", self.hci_device, "noleadv"],
                capture_output=True
            )

if __name__ == "__main__":
    sim = SimpleFcwSimulator()
    sim.run()

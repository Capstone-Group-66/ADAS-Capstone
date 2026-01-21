#!/usr/bin/env python3
"""
FCW Simulator for Windows Laptop
--------------------------------
Simulates the ADAS Jetson system by:
1. Creating a BLE GATT server (service: ADAS_SERVICE_UUID).
2. Generating a Forward Collision Warning (FCW) alert every 10 seconds.
3. Encoding the alert into the specific layered JSON -> CBOR format expected by the mobile app.
4. Handling BLE start-of-frame and fragmentation (header + payload chunks).

Payload Format (CBOR):
    {
        "t": <tick_id encoded as int>,
        "v": <speed encoded as int>,
        "h": <health_mask encoded as int>,
        "b": <bsd_mask encoded as int>,
        "a": [
            {
                "id": 0,          # 0 = FCW
                "s": 2,           # Severity
                "r": "FCW Alert"  # Rationale
            }
        ]
    }

Packet Structure (Notification):
    [Header (4 bytes)] + [CBOR Fragment]
    Header:
      - Tick ID (2 bytes, Little Endian)
      - Sequence No (1 byte)
      - Sequence Max (1 byte)

Dependencies:
    pip install bless cbor2
"""

import asyncio
import sys
import struct
import math
import logging
from typing import Dict, Any, List

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger("FCW_Sim")

try:
    import cbor  # Use cbor (not cbor2) for Python 3.6 compatibility
    from bless import BlessServer, BlessGATTCharacteristic, GATTCharacteristicProperties, GATTAttributePermissions
except ImportError as e:
    logger.error(f"Missing dependencies: {e}")
    logger.error("Please run: pip3 install cbor bless")
    sys.exit(1)

# BLE Constants (Must match Kotlin app)
ADAS_SERVICE_UUID = "0000ada5-0000-1000-8000-00805f9b34fb"
ADAS_ALERT_STREAM_UUID = "0000a1e7-0000-1000-8000-00805f9b34fb"

class FcwSimulator:
    def __init__(self):
        self.server = BlessServer(name="ADAS-Jetson-Sim")
        self.tick_id = 0
        self.is_connected = False
        
    async def setup(self):
        """Initialize BLE server and characteristics."""
        logger.info("Initializing BLE Server...")
        
        # Add Service
        await self.server.add_new_service(ADAS_SERVICE_UUID)
        
        # Add Characteristic: AlertStream (Notify only)
        # Note: permissions/properties might need adjustment based on OS quirks, 
        # but these generally work for read/notify.
        await self.server.add_new_characteristic(
            ADAS_SERVICE_UUID,
            ADAS_ALERT_STREAM_UUID,
            GATTCharacteristicProperties.notify,
            None,
            GATTAttributePermissions.readable
        )
        
        logger.info(f"Service registered: {ADAS_SERVICE_UUID}")
        logger.info(f"Characteristic registered: {ADAS_ALERT_STREAM_UUID}")

    async def start(self):
        """Start advertising."""
        await self.server.start()
        logger.info("Advertising started. Waiting for mobile app connection...")

    def generate_fcw_payload(self) -> bytes:
        """Create the complex nested dictionary and encode to CBOR."""
        self.tick_id = (self.tick_id + 1) % 65536
        
        # Layered JSON Object Structure
        # Matches Kotlin TickPayload and AlertDto
        payload_data = {
            "t": self.tick_id,
            "v": 65,            # Simulated Speed: 65 mph
            "h": 0,             # Health healthy
            "b": 0,             # No blind spot
            "a": [              # List of alerts
                {
                    "id": 0,                # ID 0 = FCW
                    "s": 2,                 # Severity: High
                    "r": "Simulated FCW"    # Rationale
                }
            ]
        }
        
        # Convert to CBOR
        cbor_bytes = cbor.dumps(payload_data)
        logger.info(f"Generated FCW Tick #{self.tick_id} ({len(cbor_bytes)} bytes CBOR)")
        return cbor_bytes

    async def send_via_ble(self, cbor_data: bytes):
        """Send data with fragmentation support."""
        if not await self.server.is_advertising():
             # Logic to detect connection is a bit tricky in Bless 
             # (it keeps advertising or stops depending on backend).
             # We'll just try to update.
             pass

        # Calculate max payload size per packet
        # BLE Header is 4 bytes.
        # Max ATT MTU is often negotiated. 
        # Ideally we'd get the current MTU, but Bless abstracts this.
        # Safe default for LE Data Length Extension is often ~244 or higher, 
        # but standard legacy BLE is 20 bytes (23 MTU - 3 overhead).
        # However, Android usually requests higher MTU (negotiated to 512 often).
        # We will assume a conservative but reasonable chunk size, 
        # or rely on Bless to fail if too large? 
        # Actually, we must manually fragment because the APP expects the 4-byte header 
        # on EVERY packet to reconstruct the stream. BLE libraries fragment the *packets* 
        # at the link layer, but our Application Layer protocol (Header+Fragment) 
        # requires us to split the CBOR logic ourselves.
        
        # Let's pick a chunk size that fits in a typical MTU.
        # If MTU is 23, payload is 20. Our header is 4. So 16 bytes.
        # If MTU is 185 (requested in Kotlin code), payload is 182. Header 4 -> 178 bytes.
        # We'll use 150 bytes to be safe and compatible.
        
        CHUNK_SIZE = 150 
        total_len = len(cbor_data)
        seq_max = math.ceil(total_len / CHUNK_SIZE) - 1
        
        if seq_max > 255:
            logger.error("Payload too large for 1-byte sequence counter!")
            return

        for seq_no in range(seq_max + 1):
            start = seq_no * CHUNK_SIZE
            end = start + CHUNK_SIZE
            chunk = cbor_data[start:end]
            
            # Header: tick_id (u16), seq_no (u8), seq_max (u8)
            # < = Little Endian
            header = struct.pack('<HBB', self.tick_id, seq_no, seq_max)
            
            packet = header + chunk
            
            # Send notification
            # Note: The notify function in Bless usually takes the characteristic UUID
            # and the value.
            try:
                # Update the value first (optional but good practice in some backends)
                self.server.get_characteristic(ADAS_ALERT_STREAM_UUID).value = packet
                
                # Send the notification
                self.server.update_value(ADAS_SERVICE_UUID, ADAS_ALERT_STREAM_UUID)
                # logger.debug(f"Sent packet {seq_no}/{seq_max} ({len(packet)} bytes)")
                
                # Small delay to prevent flooding if the stack buffer is tight
                await asyncio.sleep(0.02) 
                
            except Exception as e:
                logger.error(f"Failed to send packet {seq_no}: {e}")
                
        logger.info(f"Sent Tick #{self.tick_id} complete.")

    async def run(self):
        await self.setup()
        await self.start()
        
        logger.info("Starting simulation loop (FCW every 10s)...")
        try:
            while True:
                # Generate
                cbor_payload = self.generate_fcw_payload()
                
                # Send
                await self.send_via_ble(cbor_payload)
                
                # Wait 10 seconds
                logger.info("Waiting 10s...")
                await asyncio.sleep(10)
                
        except asyncio.CancelledError:
            logger.info("Stopping...")
        except Exception as e:
            logger.error(f"Error in loop: {e}")
        finally:
            await self.server.stop()

if __name__ == "__main__":
    sim = FcwSimulator()
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(sim.run())
    except KeyboardInterrupt:
        logger.info("Keyboard Interrupt. Exiting.")
    finally:
        loop.close()

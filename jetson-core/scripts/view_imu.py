#!/usr/bin/env python3
"""
IMU Data Viewer - Stage A Monitor
Views real-time IMU data from the Pi via ZMQ.
Requires the Pi publisher to be running.
"""
import zmq
import struct
import time
import sys
import argparse

# Protocol constants (from PiProtocol.hpp)
PI_MAGIC = 0x50493034  # "PI04"
PI_VERSION = 0x0100
MSG_IMU = 0x0004

PORT_IMU = 5558

# Struct formats (must match PiProtocol.hpp)
# Header: magic(4) + version(2) + msg_type(2) + payload_size(4) + 
#         _padding(4) + timestamp_ns(8) + sequence(4) + reserved(4) = 32 bytes
HEADER_FORMAT = '<IHHIIQII'
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)  # 32 bytes

# IMU payload: 14 floats + 1 byte + 3 reserved = 60 bytes
IMU_FORMAT = '<ffffffffffffff BBBB'
IMU_SIZE = struct.calcsize(IMU_FORMAT)  # 60 bytes


def main():
    parser = argparse.ArgumentParser(description='IMU Data Viewer')
    parser.add_argument('--pi-ip', required=True, help='Pi IP address')
    parser.add_argument('--rate', type=float, default=10.0, 
                       help='Display update rate in Hz (default 10)')
    args = parser.parse_args()
    
    ctx = zmq.Context()
    sock = ctx.socket(zmq.PULL)
    sock.setsockopt(zmq.RCVHWM, 10)
    sock.setsockopt(zmq.RCVTIMEO, 1000)  # 1 second timeout
    
    addr = f"tcp://{args.pi_ip}:{PORT_IMU}"
    print(f"Connecting to Pi IMU at {addr}...")
    sock.connect(addr)
    
    print("\n" + "="*70)
    print("                     IMU DATA VIEWER - STAGE A")
    print("="*70)
    print("  Press Ctrl+C to stop")
    print("="*70 + "\n")
    
    display_interval = 1.0 / args.rate
    last_display = 0
    sample_count = 0
    last_count_time = time.time()
    hz = 0.0
    
    try:
        while True:
            try:
                data = sock.recv()
            except zmq.Again:
                print("[IMU] Waiting for data...")
                continue
            
            if len(data) < HEADER_SIZE + IMU_SIZE:
                continue
            
            # Parse header
            header = struct.unpack(HEADER_FORMAT, data[:HEADER_SIZE])
            magic, version, msg_type, payload_size, _pad, timestamp_ns, seq, _res = header
            
            if magic != PI_MAGIC or version != PI_VERSION or msg_type != MSG_IMU:
                continue
            
            # Parse IMU payload
            imu_data = struct.unpack(IMU_FORMAT, data[HEADER_SIZE:HEADER_SIZE + IMU_SIZE])
            
            accel = imu_data[0:3]
            gyro = imu_data[3:6]
            mag = imu_data[6:9]
            quat = imu_data[9:13]
            temp = imu_data[13]
            calib = imu_data[14]
            
            sample_count += 1
            
            # Calculate rate
            now = time.time()
            if now - last_count_time >= 1.0:
                hz = sample_count / (now - last_count_time)
                sample_count = 0
                last_count_time = now
            
            # Display at specified rate
            if now - last_display >= display_interval:
                last_display = now
                
                # Clear line and print
                print(f"\r[{hz:5.1f} Hz] Seq:{seq:6d} | "
                      f"Accel: X={accel[0]:+7.2f} Y={accel[1]:+7.2f} Z={accel[2]:+7.2f} m/s² | "
                      f"Gyro: X={gyro[0]:+6.3f} Y={gyro[1]:+6.3f} Z={gyro[2]:+6.3f} rad/s | "
                      f"Calib:{calib}", end='')
                sys.stdout.flush()
                
    except KeyboardInterrupt:
        print("\n\n[IMU Viewer] Stopped")
    finally:
        sock.close()
        ctx.term()


if __name__ == "__main__":
    main()

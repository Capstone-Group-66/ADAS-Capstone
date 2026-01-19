#!/usr/bin/env python3
"""
Pi Sensor Publisher - Test Stub
This is a test version for validation. Copy to Pi and run to test connectivity.
For full implementation, see docs/PI_INTEGRATION_SPEC.md
"""

import zmq
import struct
import time
import sys
import cv2
import numpy as np
from threading import Thread, Event

# Protocol constants (from PiProtocol.hpp)
PI_MAGIC = 0x50493034  # "PI04"
PI_VERSION = 0x0100

# Message types
MSG_REAR_CAM = 0x0001
MSG_RADAR_L = 0x0002
MSG_RADAR_R = 0x0003
MSG_IMU = 0x0004
MSG_HEARTBEAT = 0x0010
MSG_DISCOVERY_REQ = 0x0020
MSG_DISCOVERY_RSP = 0x0021

# Ports (from PiProtocol.hpp)
PORT_REAR_CAM = 5555
PORT_RADAR_L = 5556
PORT_RADAR_R = 5557
PORT_IMU = 5558
PORT_CONTROL = 5559

# Mount IDs
MOUNT_REAR_CAM = 3
MOUNT_REAR_RADAR_L = 5
MOUNT_REAR_RADAR_R = 6
MOUNT_IMU = 7


class PiSensorPublisher:
    """Test publisher that simulates Pi sensor data"""
    
    def __init__(self, camera_id=0, fake_sensors=True):
        self.ctx = zmq.Context()
        self.camera_id = camera_id
        self.fake_sensors = fake_sensors
        self.stop_event = Event()
        
        # Sequence counters
        self.cam_seq = 0
        self.radar_l_seq = 0
        self.radar_r_seq = 0
        self.imu_seq = 0
        self.hb_seq = 0
        
        # Create sockets - we BIND, Jetson CONNECTS
        self.cam_sock = self._create_bind(PORT_REAR_CAM)
        self.radar_l_sock = self._create_bind(PORT_RADAR_L)
        self.radar_r_sock = self._create_bind(PORT_RADAR_R)
        self.imu_sock = self._create_bind(PORT_IMU)
        self.ctrl_sock = self._create_rep(PORT_CONTROL)
        
        print(f"[Pi Publisher] Bound to all ports")
        print(f"  Camera:    tcp://*:{PORT_REAR_CAM}")
        print(f"  Radar L:   tcp://*:{PORT_RADAR_L}")
        print(f"  Radar R:   tcp://*:{PORT_RADAR_R}")
        print(f"  IMU:       tcp://*:{PORT_IMU}")
        print(f"  Control:   tcp://*:{PORT_CONTROL}")
    
    def _create_bind(self, port):
        sock = self.ctx.socket(zmq.PUSH)
        sock.setsockopt(zmq.SNDHWM, 10)
        sock.bind(f"tcp://*:{port}")
        return sock
    
    def _create_rep(self, port):
        sock = self.ctx.socket(zmq.REP)
        sock.setsockopt(zmq.RCVTIMEO, 1000)
        sock.bind(f"tcp://*:{port}")
        return sock
    
    def _build_header(self, msg_type, payload_size, seq):
        timestamp = time.time_ns()
        return struct.pack('<IHHIQII',
            PI_MAGIC,
            PI_VERSION,
            msg_type,
            payload_size,
            timestamp,
            seq,
            0  # reserved
        )
    
    def camera_thread(self):
        """Stream camera frames at ~30 fps"""
        print("[Camera] Starting camera thread...")
        
        if self.fake_sensors:
            # Create test pattern
            def generate_frame():
                frame = np.zeros((720, 1280, 3), dtype=np.uint8)
                cv2.putText(frame, f"RearCam - Frame {self.cam_seq}", 
                           (400, 360), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 3)
                cv2.putText(frame, time.strftime("%H:%M:%S"), 
                           (500, 450), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                return frame
        else:
            cap = cv2.VideoCapture(self.camera_id)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        
        while not self.stop_event.is_set():
            if self.fake_sensors:
                frame = generate_frame()
            else:
                ret, frame = cap.read()
                if not ret:
                    time.sleep(0.033)
                    continue
            
            # Encode to MJPEG
            ret, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not ret:
                continue
            
            jpeg_bytes = jpeg.tobytes()
            
            # Camera payload header: width(2) + height(2) + encoding(1) + reserved(3)
            h, w = frame.shape[:2]
            payload_header = struct.pack('<HHBBBB', w, h, 1, 0, 0, 0)  # encoding=1=MJPEG
            payload = payload_header + jpeg_bytes
            
            header = self._build_header(MSG_REAR_CAM, len(payload), self.cam_seq)
            self.cam_seq += 1
            
            try:
                self.cam_sock.send(header + payload, zmq.NOBLOCK)
            except zmq.Again:
                pass  # Queue full, drop frame
            
            time.sleep(0.033)  # ~30 fps
        
        if not self.fake_sensors:
            cap.release()
        print("[Camera] Thread stopped")
    
    def imu_thread(self):
        """Stream IMU data at 100 Hz"""
        print("[IMU] Starting IMU thread...")
        
        while not self.stop_event.is_set():
            # Fake IMU data
            accel = [0.0, 0.0, 9.81]  # Stationary, gravity on Z
            gyro = [0.0, 0.0, 0.0]
            mag = [25.0, 0.0, 45.0]   # Earth's magnetic field
            quat = [1.0, 0.0, 0.0, 0.0]  # Identity quaternion
            temp = 25.0
            calib = 3  # Fully calibrated
            
            # IMU payload: 13 floats + 1 byte + 3 reserved = 56 bytes
            payload = struct.pack('<ffffffffffffff BBB',
                accel[0], accel[1], accel[2],
                gyro[0], gyro[1], gyro[2],
                mag[0], mag[1], mag[2],
                quat[0], quat[1], quat[2], quat[3],
                temp, calib, 0, 0)
            
            header = self._build_header(MSG_IMU, len(payload), self.imu_seq)
            self.imu_seq += 1
            
            try:
                self.imu_sock.send(header + payload, zmq.NOBLOCK)
            except zmq.Again:
                pass
            
            time.sleep(0.01)  # 100 Hz
        
        print("[IMU] Thread stopped")
    
    def radar_l_thread(self):
        """Stream left radar data at ~20 Hz"""
        print("[RadarL] Starting radar L thread...")
        
        while not self.stop_event.is_set():
            # Fake radar data - just some bytes
            raw_data = b"\x02\x00\x00\x00\x10\x00\x00\x00"  # Fake OPS243 data
            
            # Radar payload: data_length(2) + radar_type(1) + reserved(1) + data
            payload = struct.pack('<HBB', len(raw_data), 0, 0) + raw_data
            
            header = self._build_header(MSG_RADAR_L, len(payload), self.radar_l_seq)
            self.radar_l_seq += 1
            
            try:
                self.radar_l_sock.send(header + payload, zmq.NOBLOCK)
            except zmq.Again:
                pass
            
            time.sleep(0.05)  # ~20 Hz
        
        print("[RadarL] Thread stopped")
    
    def radar_r_thread(self):
        """Stream right radar data at ~20 Hz"""
        print("[RadarR] Starting radar R thread...")
        
        while not self.stop_event.is_set():
            raw_data = b"\x02\x00\x00\x00\x10\x00\x00\x00"
            payload = struct.pack('<HBB', len(raw_data), 0, 0) + raw_data
            
            header = self._build_header(MSG_RADAR_R, len(payload), self.radar_r_seq)
            self.radar_r_seq += 1
            
            try:
                self.radar_r_sock.send(header + payload, zmq.NOBLOCK)
            except zmq.Again:
                pass
            
            time.sleep(0.05)
        
        print("[RadarR] Thread stopped")
    
    def control_thread(self):
        """Handle control messages (discovery, heartbeat)"""
        print("[Control] Starting control thread...")
        start_time = time.time()
        
        while not self.stop_event.is_set():
            try:
                # Check for incoming messages
                data = self.ctrl_sock.recv()
                
                if len(data) >= 32:
                    magic, version, msg_type = struct.unpack('<IHH', data[:8])
                    
                    if magic == PI_MAGIC and version == PI_VERSION:
                        if msg_type == MSG_DISCOVERY_REQ:
                            print("[Control] Received discovery request")
                            self._send_discovery_response()
                        else:
                            print(f"[Control] Unknown message type: {msg_type}")
                            self.ctrl_sock.send(b"")  # Empty response
                    else:
                        self.ctrl_sock.send(b"")
                else:
                    self.ctrl_sock.send(b"")
                    
            except zmq.Again:
                # Timeout - send heartbeat
                pass
            
            # Send heartbeat every second
            uptime_ms = int((time.time() - start_time) * 1000)
            self._send_heartbeat(uptime_ms)
        
        print("[Control] Thread stopped")
    
    def _send_discovery_response(self):
        """Send discovery response with device list"""
        # 4 devices: RearCam, RadarL, RadarR, IMU
        num_devices = 4
        
        # Build response payload
        resp_payload = struct.pack('<BBBB', num_devices, 0, 0, 0)
        
        # Device info: type(1) + mount_id(1) + status(2) + serial(16) = 20 bytes each
        devices = [
            (1, MOUNT_REAR_CAM, 0, b"RearCam001\x00\x00\x00\x00\x00\x00"),
            (2, MOUNT_REAR_RADAR_L, 0, b"RadarL001\x00\x00\x00\x00\x00\x00\x00"),
            (2, MOUNT_REAR_RADAR_R, 0, b"RadarR001\x00\x00\x00\x00\x00\x00\x00"),
            (3, MOUNT_IMU, 0, b"IMU001\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"),
        ]
        
        for dev_type, mount_id, status, serial in devices:
            resp_payload += struct.pack('<BBH', dev_type, mount_id, status) + serial
        
        header = self._build_header(MSG_DISCOVERY_RSP, len(resp_payload), 0)
        self.ctrl_sock.send(header + resp_payload)
        print("[Control] Sent discovery response")
    
    def _send_heartbeat(self, uptime_ms):
        """Send heartbeat with status"""
        # HeartbeatPayload: uptime(8) + cam(1) + radarL(1) + radarR(1) + imu(1) + 
        #                   chrony_offset(4) + reserved(4) = 20 bytes
        payload = struct.pack('<QBBBB ii',
            uptime_ms,
            1, 1, 1, 1,  # All healthy
            0,  # Chrony offset (0 = in sync)
            0   # Reserved
        )
        
        header = self._build_header(MSG_HEARTBEAT, len(payload), self.hb_seq)
        self.hb_seq += 1
        
        # Note: Heartbeat goes out on the PUSH socket for control
        # But we're using REP, so we can't push. Send via camera socket as backup
    
    def run(self):
        """Start all threads and run until stopped"""
        print("\n" + "="*60)
        print("           PI SENSOR PUBLISHER - TEST MODE")
        print("="*60)
        print("  Press Ctrl+C to stop")
        print("="*60 + "\n")
        
        threads = [
            Thread(target=self.camera_thread, daemon=True),
            Thread(target=self.imu_thread, daemon=True),
            Thread(target=self.radar_l_thread, daemon=True),
            Thread(target=self.radar_r_thread, daemon=True),
            Thread(target=self.control_thread, daemon=True),
        ]
        
        for t in threads:
            t.start()
        
        try:
            while True:
                time.sleep(1)
                print(f"[Stats] Cam:{self.cam_seq} IMU:{self.imu_seq} "
                      f"RadarL:{self.radar_l_seq} RadarR:{self.radar_r_seq}")
        except KeyboardInterrupt:
            print("\n[Main] Stopping...")
            self.stop_event.set()
        
        for t in threads:
            t.join(timeout=2)
        
        self.ctx.term()
        print("[Main] Stopped")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Pi Sensor Publisher Test Stub")
    parser.add_argument("--camera", type=int, default=0, help="Camera ID")
    parser.add_argument("--real-camera", action="store_true", 
                       help="Use real camera instead of test pattern")
    args = parser.parse_args()
    
    publisher = PiSensorPublisher(
        camera_id=args.camera,
        fake_sensors=not args.real_camera
    )
    publisher.run()

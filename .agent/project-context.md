# ADAS-Capstone Project Context
> **Last Updated:** 2026-01-15 by Claude
> **Priority:** User-provided facts ALWAYS override proposal

---

## Project Overview
- **Name:** Customizable AI-Enabled ADAS Demonstrator
- **Course:** SYSC 4907 (Capstone)
- **Supervisor:** Mohamed Atia
- **Vehicle:** Honda Civic 2012 Coupe Si

## Team Members
| Name | Student # | Role |
|------|-----------|------|
| Ajen Srisivapalan | 101248498 | Main Brain, Mobile App |
| Damon Ricci | 101229913 | Pipeline, Main Brain, Mobile App |
| Jason Keah | 101233435 | Main Brain, Mobile App |
| John Guo | 101233817 | Mobile App, DevOps |
| Rami Mrad | 101237780 | Pipeline, Main Brain |
| Ryan Page | 101268082 | Pipeline, DevOps |

---

## Hardware Topology (CRITICAL - from user)

```
┌─────────────────────────────────────────────────────────────┐
│                     JETSON ORIN NANO                        │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐         │
│  │ FrontCam    │  │ FrontRadar  │  │ IMU         │         │
│  │ (USB)       │  │ (USB/UART)  │  │ (I2C)       │         │
│  └─────────────┘  └─────────────┘  └─────────────┘         │
│  ┌─────────────┐  ┌─────────────┐                          │
│  │ SideCam-L   │  │ SideCam-R   │                          │
│  │ (USB)       │  │ (USB)       │                          │
│  └─────────────┘  └─────────────┘                          │
│                         ▲                                   │
│                         │ Network                           │
└─────────────────────────┼───────────────────────────────────┘
                          │
┌─────────────────────────┼───────────────────────────────────┐
│                    REAR PI                                  │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐         │
│  │ RearCam     │  │ RearRadar-L │  │ RearRadar-R │         │
│  └─────────────┘  └─────────────┘  └─────────────┘         │
└─────────────────────────────────────────────────────────────┘
```

**Key Insight:** BSD and RCW require Pi↔Jetson networking to be working first!

---

## Alert Complexity Assessment (from discussion 2026-01-15)

| Alert | Sensors Needed | Connected To | Practical Complexity |
|-------|---------------|--------------|---------------------|
| **FCW** | FrontCam + FrontRadar | Jetson direct | **EASIEST** - radar-only demo possible |
| **LDW** | FrontCam + IMU | Jetson direct | Medium - needs lane CV |
| **RCW** | RearCam + RearRadars | Rear Pi | Hard - needs Pi networking |
| **BSD** | SideCams + RearRadars + RearCam | Mixed | Hard - needs Pi + fusion |

**Recommended MVP:** FCW with front radar only (degraded mode), then add camera fusion

---

## Current Implementation Status

### Mobile App (`mobile-app/MobileApplication/`)
- **Status:** Skeleton with TODOs
- **Issues Identified (2026-01-15):**
  - ❌ No coroutines - using `BluetoothGattCallback` instead of `Dispatchers.IO/Main`
  - ❌ No error handling in `TickDecoder.decode()` - will crash on malformed CBOR
  - ❌ `drive.kt` hardcoded to `Color.Green` - no TTC threshold logic
  - ⚠️ ViewModel exists but uses generic names (`status1`, `status2`)
  - ⚠️ BLE Manager not connected to ViewModel

### Jetson Core (`jetson-core/`)
- **Key Files:**
  - `src/stage_a/RadarIngest.cpp` - Front radar parsing
  - `src/stage_a/CameraIngest.cpp` - Front camera capture
  - `src/stage_a/IMUIngest.cpp` - IMU data
  - `src/stage_a/NetworkIngest.cpp` - Pi↔Jetson communication
  - `config/componentConfig.yaml` - Mount transforms, zones, fusion params

---

## Key Architecture Decisions

1. **Pipeline Architecture** - Staged with SPSC queues, 20Hz fusion tick
2. **MVVM for Mobile** - Model → ViewModel → View with StateFlow
3. **BLE Transport** - CBOR on wire, JSON in logs, GATT services
4. **Ego Frame** - Origin at rear axle midpoint, +X forward, +Y left, +Z up

---

## User-Provided Facts (Override Proposal)

<!-- Add facts here as user provides them -->

1. **2026-01-15:** Rear sensors (RearCam, RearCornerRadar-L/R) are connected to a Raspberry Pi, NOT directly to Jetson. Pi↔Jetson networking is required for BSD/RCW.

---

## Reference Documents

- **Proposal:** `docs/FINAL Project Proposal.txt` (90 pages, authoritative but may be outdated)
- **Config:** `jetson-core/config/componentConfig.yaml`
- **GitHub:** `Capstone-Group-66/ADAS-Capstone`

---

## Quick Reference: Proposal Sections

| Topic | Section |
|-------|---------|
| Alert Schema (FR70) | 4.2.5 |
| BLE Framing | 4.9 |
| Pipeline Stages | 4.5 |
| Mobile Architecture | 4.12 |
| TTC Calculation | FR31 (section 2.1.4) |
| BSD Zones | FR32-33, config lines 931-935 |
| Fusion Params | config lines 941-944 |

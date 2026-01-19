# **ADAS Capstone \- Oral Presentation Master Plan (60 Minutes)**

Date: January 22, 2026  
Total Time: 60 Minutes (Presentation) \+ 18 Minutes (Q\&A)  
Theme: "From Chaos to Order: Engineering a Modular, Distributed ADAS Pipeline"  
Core Narrative: We identified a safety gap in older vehicles, designed a distributed architecture to solve physical constraints (bandwidth/power), and implemented a vertical slice (Sensor \-\> Jetson \-\> App) to verify our critical path (latency).

## **Part 1: The Engineering Foundation (0:00 \- 20:00)**

**Goal:** Prove we followed a rigorous engineering design process, not just "plug-and-play." This section addresses the "Approach" and "Design" grading criteria.

### **1.1 Problem & Requirements Analysis (Damon)**

* **The Problem:** The Safety Gap. Modern cars have ADAS; the 1.4 billion older cars do not. Retrofits exist but are "dumb" (unconnected) and lack sensor fusion.  
* **Functional Requirements (FRs):**  
  * **Real-time Latency:** System total \< 200ms (Human reaction time buffer).  
  * **Sensor Coverage:** 360-degree awareness (Blind spots \+ Front collision).  
  * **Modularity:** Must support variable sensor mounting positions (The "Customizable" aspect).  
* **Non-Functional Requirements (NFRs):**  
  * **Reliability:** Independent failure modes (if Camera dies, Radar must still alert).  
  * **Extensibility:** Architecture must support future sensors (LiDAR placeholder).  
  * **Privacy & Compliance:** "Privacy by Design" (No Audio recording, Local Processing only). Complies with PIPEDA/Highway Traffic Act (No obstructions).  
* **User Assumptions (Critical for Grading):**  
  * **Assumption 1:** The driver is alert and responsible. This is a *Warning* system (SAE Level 0/1), not Control (Level 2+).  
  * **Assumption 2:** The vehicle provides standard 12V DC power.  
  * **Assumption 3:** Environmental conditions are within camera operating limits (Day/Night, not Heavy Snow).

### **1.2 System Architecture & Design Choices (Damon \+ All)**

* **The "Big Picture" Block Diagram:**  
  * Input: 4 Cameras, 3 Radars, **IMU (100Hz, 9-Axis)**, **GPS (5Hz)**.  
  * Compute: Jetson Orin Nano (Central Brain).  
  * Distributed Node: Raspberry Pi 4 (Rear Sensor Serializer).  
* **Design Patterns:**
  * **Producer-Consumer:** Lock-free SPSC queues decoupling ingestion from processing.
  * **Observer Pattern:** ZMQ PUB/SUB for the Rear Node bridge.  
* **Alternatives Analysis (Why did we build it this way?):**  
  * *Architecture Alt 1:* **Centralized (All USB to Jetson).**  
    * *Implication:* Rejected. 4x USB cameras \+ Radars exceed the Jetson's single USB controller bandwidth (5Gbps shared). Cable length (3m) causes signal degradation.  
  * *Architecture Alt 2:* **ROS (Robot Operating System).**  
    * *Implication:* Rejected. ROS adds significant overhead/latency and complexity for a constrained embedded system.  
  * *Architecture Alt 3:* **LiDAR Integration.**  
    * *Implication:* Rejected ("Design by Exclusion").  
    * *Engineering Reason:* The available unit (RPLidar A1) is **Single-Rotating-Point** (2D) with a 5.5Hz scan rate and 10m range.  
    * *Safety Risk:* At 60km/h, a car travels ~16m/s. By the time a 360° scan completes, the target has moved ~3m. This latency introduces unacceptable integration complexity for minimal gain over Camera+Radar.  
  * *Selected Approach:* **Distributed Custom C++ Pipeline.**  
    * *Advantage:* Low latency (SPSC Queues), control over bandwidth (MJPEG compression at the edge).  
* **Power Engineering (Ryan):**  
  * **The Challenge:** "The 5-Amp Spike." Jetson \+ Sensors draw peak current that sags standard car rails.  
  * **The Design:** Custom Power Distribution Board with Buck Converters (12V \-\> 5V stable). Fusing strategy for safety.

## **Part 2: Implementation \- The Physical Layer (20:00 \- 30:00)**

**Goal:** Explain how we solved the physical integration challenges.

### **2.1 The Rear Node & ZMQ Bridge (Ryan)**

* **The Cable Constraint:** USB 3.0 degrades over 3 meters.  
* **The Solution:** TCP/ZMQ over Ethernet.  
  * *Protocol Choice:* **ZeroMQ (ZMQ).**  
  * *Why?* It handles "Framing" (atomic messages) and "Reconnection" automatically, unlike raw TCP sockets.  
  * *Trade-off:* Adds \~20ms latency, which we correct for in software.  
* **Network Engineering:**  
  * **Static IP Configuration:** No router (DHCP) to reduce failure points.  
  * **Time Synchronization:** Using **Chrony (NTP)** to keep the Rear Pi clock aligned with the Jetson clock (Critical for Fusion).

### **2.2 Ingestion Pipeline (Stage A) (Damon)**

* **Software Design:**  
  * **SPSC Queues:** Single-Producer Single-Consumer lock-free queues.  
  * *Engineering Concept:* **"Freshness over Completeness."** If the AI lags, we drop old frames. In ADAS, processing a 500ms old frame is dangerous.  
* **Testing & Verification:**  
  * *Unit Test:* Queue overflow behavior (dropping oldest).  
  * *Integration Test:* The "Timestamp Delta" graph (verifying 20Hz stability under load).

## **Part 3: Implementation \- The Perception Engine (30:00 \- 45:00)**

**Goal:** The "Brains." Converting raw signals into safety warnings.

### **3.1 Radar Physics & Processing (John)**

* **Sensor Physics:** FMCW Radar (Doppler Effect).  
  * *Data:* Range vs. Relative Velocity.
  * *Constraint:* **No Azimuth**. The OPS243 is a simple Doppler radar; it gives distance/speed but NOT angle.
  * *Implication:* We **MUST** fuse with the camera to know *where* the object is laterally (Azimuth comes from Vision).  
* **The "Split Packet" Problem:**  
  * *Challenge:* The OPS243 sensor sends Range and Velocity as separate messages.  
  * *Solution:* The **"Hold & Fuse" Algorithm**. We buffer the Velocity packet and fuse it with the subsequent Range packet based on timestamp alignment (\<15ms).  
* **Logic:** The **Kalman Filter**.  
  * *Reasoning:* Radar data is noisy. We use a Constant Velocity model to smooth the tracks.  
* **FCW Math:** TTC \= Range / RelativeVelocity.  
  * *Trigger:* Negative Velocity (Closing) AND TTC \< 2.5s.

### **3.2 Computer Vision & Fusion (Rami)**

* **The Vision Pipeline:**  
  * Model: **YOLOv8 Nano** (ONNX).  
  * *Why Nano?* Optimized for Edge deployment (Latency vs. Accuracy trade-off). We prioritized FPS over perfect classification.  
* **Coordinate Transformation (The Math):**  
  * Mapping 1D Radar Range ($X$) to 2D Camera Pixels ($u, v$).  
  * *Concept:* **The Ego Frame**. We transform all sensor data into a common vehicle-centered coordinate system before projecting to the camera plane.
  * *Formula:* Homography Matrix / Inverse Pinhole Model.  
* **Sensor Fusion Demo:**  
  * **"The Ghost Box":** Video showing the Radar target overlaying the Camera feed.  
  * *Visual Behavior:* The bounding box **scales dynamically** based on Radar range ($Z_{radar}$) to match the perspective of the camera ($Z_{cam}$), proving the Inverse Pinhole Model works.
  * *Verification:* Visual proof that Stage A (Radar) and Stage B (Camera) are synchronized in real-time.

### **3.3 Additional Alert Logic (The Math Behind the Future)**  
  
*   **Context:** While the demo is FCW, we carefully derived the math for the full suite.  
*   **Lane Departure Warning (LDW):**  
    *   *Equation:* $|D_{center}| > (W_{lane} / 2) - D_{tolerance}$.  
    *   *Logic:* We calculate the lateral distance ($D_{center}$) from the Ego Frame origin to the polynomial lane fit. If deviation exceeds threshold, trigger Alert.  
*   **Blind Spot Detection (BSD):**  
    *   *Equation:* Point-in-Polygon Test.  
    *   *Logic:* We define static "Rear Quarter" polygons in the Ego Frame (e.g., $x \in [-5, -1], y \in [\pm 1.5, \pm 3.5]$).  
    *   *Trigger:* If any Track's Kalman State $(x, y)$ enters the specific polygon ID, trigger Alert.  
  
## **Part 4: Implementation \- The Interface & Verification (45:00 \- 55:00)**

**Goal:** Delivering the warning to the human.

### **4.1 The Connectivity Layer (Ajen)**

* **BLE Protocol:**  
  * *Design:* Custom GATT Profile vs. Serial Port Profile.  
  * *Efficiency:* The **CBOR Schema**. Why we chose binary serialization (bandwidth).
  * *Fragmentation Strategy:* Payload > MTU (240 bytes). 
    * We implemented **Manual Fragmentation**. 
    * *Header Structure:* `[ tick_id | seq_no | seq_max | payload_chunk ]`. 
    * Reassembly happens on the Android side.  
* **DevOps & CI/CD:**  
  * Automated linting and build checks on GitHub to ensure code quality across 6 developers.
  * **System Reliability:** Implemented a **Watchdog Timer** to auto-restart the pipeline if the "Tick" thread hangs for >1s.

### **4.2 The User Experience (Jason)**

* **UX Philosophy:** "Dark Cockpit." Minimizing cognitive load.  
* **Android Architecture:**  
  * **MVVM Pattern:** Decoupling the BLE Data Stream from the UI Rendering.  
  * *Safety:* Preventing ANR (App Not Responding) by moving parsing to background threads.  
* **Alert Hierarchy:** Green (Safe) \-\> Red (Critical).

### **4.3 THE VERTICAL SLICE DEMO (All)**

* **The Video:**  
  * Split screen:  
    1. Team member running at the car.  
    2. Laptop Terminal showing TTC: 1.5s.  
    3. Phone Screen flashing **RED**.  
* **Conclusion:** "This demo verifies the entire critical path latency is within safety margins."

## **Part 5: Conclusion (55:00 \- 60:00)**

* **Summary:** We followed a structured engineering process to build a modular, distributed ADAS platform.  
*   **Summary:** We followed a structured engineering process to build a modular, distributed ADAS platform.  
*   **Future Roadmap (Jan \- April):**  
    *   Phase 1: Full Camera Calibration.  
    *   Phase 2: Complex Fusion (Lane Departure).  
    *   Phase 3: Final robustness testing.  
*   **Q\&A Preparation:**  
    *   *Anticipate:* "Why not LiDAR?" -> "Cost/Complexity trade-off for aftermarket market."
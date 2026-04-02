# COMSTRUCTION - Mixed-Reality Sandbox Construction Simulator

**Bridging physical sand terrain with virtual 3D environments using depth cameras, Unity, and VR**

![Unity](https://img.shields.io/badge/Unity-2022.x-000000?logo=unity&logoColor=white)
![C#](https://img.shields.io/badge/C%23-50.4%25-239120?logo=csharp)
![Python](https://img.shields.io/badge/Python-44.3%25-3776AB?logo=python&logoColor=white)
![Award](https://img.shields.io/badge/Award-Gyeonggi%20Content%20Agency-orange)

---

## Overview

COMSTRUCTION is a **mixed-reality interactive experience** that synchronizes a real-world sandbox terrain with a Unity-based 3D virtual environment. Users sculpt physical sand, capture the depth profile with an Intel RealSense D455, and explore the generated virtual terrain through VR - complete with a remotely-controlled toy excavator whose movements are mirrored in the virtual world.

### Key Features

| Feature | Description |
|---|---|
| **Depth-to-Terrain Pipeline** | RealSense depth map to Unity heightmap conversion with ROI processing |
| **Gaussian Stamping** | Terrain deformation algorithm for realistic virtual landscape from depth data |
| **Physical-Virtual Sync** | UDP-based real-time synchronization of toy excavator with virtual counterpart |
| **VR Exploration** | Full VR walkthrough of the generated terrain |
| **One-Click Generation** | Depth capture to terrain generation to object placement in a single flow |

## Tech Stack

| Layer | Technology |
|---|---|
| Depth Capture | Intel RealSense D455, Python SDK |
| 3D Environment | Unity 2022, C# |
| Communication | UDP socket protocol |
| Terrain Algorithm | Gaussian stamping, heightmap-based deformation |
| VR | Unity XR / VR headset integration |
| Hardware Control | RC excavator with dual-signal remote |

## Awards

* **Merit Award** - COMSTRUCTION Project, Gyeonggi Content Agency, Aug. 2025

* ## Research Context

* This project explores the intersection of:
* * Physical-virtual terrain synchronization for quantitative evaluation
  * * Mixed-reality interaction with tangible interfaces
    * * Real-time sensor-driven simulation
     
      * **Full paper manuscript in preparation: "Physical-Virtual Terrain Synchronization System for Quantitative Evaluation of Excavation Simulation"**
     
      * ## Getting Started
     
      * ### Prerequisites
      * * Intel RealSense D455 depth camera
        * * Unity 2022.x with XR plugin
          * * Python 3.10+ with RealSense SDK
            * * VR headset (optional for full experience)
             
              * ```bash
                git clone https://github.com/sejun42/COMSTRUCTION.git
                cd COMSTRUCTION
                pip install -r requirements.txt
                ```

                ## Contact

                * **Sejun Yoon** - sejun1324@gmail.com
                * * GitHub: @sejun42
                  * 

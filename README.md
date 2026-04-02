# COMSTRUCTION — Mixed-Reality Sandbox Construction Simulator

> Physical sandbox terrain, Intel RealSense depth capture, Unity terrain generation, and VR exploration in one end-to-end pipeline

![Unity](https://img.shields.io/badge/Unity-2022.x-000000?logo=unity&logoColor=white)
![C#](https://img.shields.io/badge/C%23-Unity%20Scripts-239120?logo=csharp)
![Python](https://img.shields.io/badge/Python-Depth%20Pipeline-3776AB?logo=python&logoColor=white)
![Hardware](https://img.shields.io/badge/Hardware-Intel%20RealSense%20D455-orange)

---

## Overview

COMSTRUCTION is a mixed-reality interactive system that turns a real sandbox into a navigable virtual construction scene.

Users sculpt terrain with real sand, capture its depth profile with an Intel RealSense D455, convert the result into a Unity terrain heightmap, and then explore the generated world through VR. The project also experiments with synchronizing a physical toy excavator and its virtual counterpart so that off-screen physical manipulation feels connected to the in-headset simulation.

## End-to-End Pipeline

1. A user shapes terrain in a physical sandbox.
2. `depth_capture/capture_depth.py` captures a filtered depth map from RealSense.
3. The depth map is cropped with an ROI, saved as raw depth data, and exported with metadata.
4. `scripts/depth_to_raw.py` converts the latest capture into a Unity-ready `.r16` heightmap plus preview image.
5. Unity terrain scripts generate and decorate the virtual environment.
6. Sensor and controller data are forwarded into Unity for physical-to-virtual synchronization.
7. The user explores the generated terrain in VR.

## Key Capabilities

| Capability | What it does |
| --- | --- |
| Depth capture | Streams RealSense depth frames at `1280x720@30fps` and supports ROI-based capture |
| Filtering | Applies decimation, spatial, temporal, and hole-filling filters before export |
| Heightmap conversion | Transforms captured depth into Unity `.r16` terrain assets |
| One-click terrain generation | Supports a fast capture-to-terrain workflow for repeated experiments |
| Physical-virtual sync | Receives tracked or remote-control signals and mirrors them inside Unity |
| VR scene exploration | Lets users walk through the generated terrain in a headset-based environment |

## Repository Layout

```text
COMSTRUCTION/
|- depth_capture/
|  `- capture_depth.py                  # RealSense depth capture with ROI selection and export
|- scripts/
|  `- depth_to_raw.py                   # Convert saved depth maps into Unity .r16 heightmaps
|- terrain/
|  `- heightmaps/                       # Generated terrain heightmap outputs
|- unity_terrain/
|  `- cs_file/
|     |- ComstructionOneClick.cs        # Terrain generation flow in Unity
|     `- ComstructionScatter.cs         # Scene object scattering / placement
|- unity_sensor/
|  |- UdpPositionReceiver.cs            # Unity-side position receiver
|  |- Arduino/
|  |  `- excavator_sensor_arduino/...   # Arduino sketch for the excavator-side sensor setup
|  `- camera tracking pipeline/
|     `- camera_tracking_pipeline.py    # Tracking / integration support script
|- docs/
`- requirements.txt
```

## Technical Notes

### Depth capture

The RealSense capture script configures the D455 depth stream, applies optional filtering, and lets the operator define a square ROI directly in the preview window. Captures are exported as:

- `.npy` depth arrays
- `*_raw16.png` depth images
- `*_vis.png` colorized previews
- `*_meta.txt` metadata files

### Heightmap generation

The conversion script reads the latest saved depth capture, loads depth scale metadata, fills holes, clips depth ranges, resizes the terrain to a Unity-friendly resolution such as `513`, and writes a `.r16` file plus a PNG preview for inspection.

### Unity integration

The Unity-side folders suggest a split between:

- terrain generation and scene assembly
- UDP or sensor-based position intake
- external hardware experiments such as an excavator controller

This makes the repository closer to a research prototype pipeline than a single packaged application.

## Getting Started

### Prerequisites

- Intel RealSense D455
- Python 3.10+
- Unity 2022.x
- VR/XR setup if you want to test the immersive environment

### Python setup

```bash
git clone https://github.com/sejun42/COMSTRUCTION.git
cd COMSTRUCTION
pip install -r requirements.txt
```

### Capture a depth map

```bash
python depth_capture/capture_depth.py
```

Controls:

- `SPACE`: save current capture
- `R`: toggle filtering
- `X`: clear ROI
- `ESC`: quit

### Convert to Unity heightmap

```bash
python scripts/depth_to_raw.py
```

The script automatically looks for the latest saved depth capture and writes a new `.r16` terrain file into `terrain/heightmaps/`.

### Unity side

After generating a heightmap, open the Unity project side of the pipeline and connect the generated terrain asset to the terrain creation scripts under `unity_terrain/cs_file/`.

## Research Context

COMSTRUCTION explores:

- tangible interfaces for mixed reality
- real-to-virtual terrain synchronization
- rapid terrain authoring from physical materials
- simulation and evaluation for construction-style scenarios

The repository also includes saved captures and generated heightmaps, which makes it useful as both a development workspace and an experiment archive.

## Recognition

- Merit Award, Gyeonggi Content Agency, August 2025

## Contact

- Sejun Yoon — [sejun1324@gmail.com](mailto:sejun1324@gmail.com)
- GitHub — [@sejun42](https://github.com/sejun42)

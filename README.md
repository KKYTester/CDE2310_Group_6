# CDE2310 Group 6 - Autonomous Warehouse Robot

**Team Members:** Kang Kiat Yang, Guda Omkar, Grover Amitaansh, Bhatia Aksh

## Project Overview

This repository contains the software and documentation for our CDE2310 final project: an autonomous TurtleBot3-based warehouse delivery robot. The robot explores an unknown maze, detects delivery stations using ArUco markers, and autonomously delivers ping pong ball payloads.

## Mission Objectives
AKSH BHATIA

- **Phase 1:** Autonomous maze exploration using frontier-based navigation
- **Phase 2:** Marker detection and payload delivery to Station A (static target)
- **Phase 3:** Payload delivery to Station B (moving target)
- **Bonus:** Multi-floor navigation using lift API integration

## System Components

### Hardware
- TurtleBot3 Burger platform
- Raspberry Pi Camera V2 (ArUco marker detection)
- Custom flywheel launcher mechanism
- R380 DC motor + MG90S servo

### Software Stack
- **ROS 2 Humble Hawksbill**
- **SLAM:** Google Cartographer
- **Navigation:** Nav2 (A* global planning + DWA local planning)
- **Exploration:** Frontier-based autonomous mapping
- **Vision:** OpenCV ArUco marker detection

## Repository Structure
```
cde2310_e2/
├── r2auto_nav.py        # Autonomous navigation node
├── r2mover.py           # Movement control
├── r2moverotate.py      # Rotation utilities
├── r2occupancy.py       # Occupancy grid processing
├── r2scanner.py         # LiDAR data processing
├── package.xml          # ROS 2 package configuration
├── setup.py             # Python package setup
└── README.md            # This file
```

## Quick Start
```bash
# Clone repository
git clone https://github.com/KKYTester/CDE2310_Group_6.git
cd CDE2310_Group_6/cde2310_e2

# Build ROS 2 package
cd ~/ros2_ws
colcon build --packages-select cde2310_e2
source install/setup.bash

# Launch (instructions coming soon)
```

## Current Status

🚧 **Work in Progress** - Preliminary Design Review completed Week 7

- ✅ ConOps document finalized
- ✅ System architecture defined
- ✅ Component selection completed
- 🔄 ArUco detection integration in progress
- 🔄 Launcher mechanism prototyping
- ⏳ Full mission integration pending

## Documentation

- [Concept of Operations (ConOps)](docs/CDE2310_ConOps.pdf)
- [Preliminary Design Review Slides](docs/PDR_Slides.pdf)
- [Algorithm Report](docs/Algorithms_Report.docx) *(coming soon)*

## Dependencies

- ROS 2 Humble
- Python 3.10+
- OpenCV 4.7+
- NumPy
- TurtleBot3 packages
- Nav2 navigation stack
- Cartographer SLAM

## License

This project is part of the CDE2310 course at the National University of Singapore.

---

**Last Updated:** March 2026  
**Project Deadline:** 16 April 2026

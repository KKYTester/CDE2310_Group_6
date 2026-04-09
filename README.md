
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
- **SLAM:** Cartographer
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


# Autonomous Campus Delivery Robot (WIP)

> 🚧 **Active Development** - Mobile robot platform for autonomous Red Bull delivery on UWA campus using ROS2 Nav2 stack and LIDAR-based navigation.

## Project Status

**Current Phase**: Hardware design + assembly
**Target Demo**: End of March
**Latest Update**: Jetson Orin Nano configured, components tested, 

---

## Overview

Building a fully autonomous delivery robot from scratch to navigate outdoor campus environments and deliver items (starting with Red Bull cans). The system uses ROS2 Humble with the Nav2 navigation stack, RPLIDAR for 2D mapping and localization, and a custom 3D-printed chassis designed for outdoor terrain.

**Goal**: Prove autonomous last-mile delivery is viable on a university campus with pedestrians, vehicles, uneven surfaces, and dynamic obstacles.

## Current Features

✅ **ROS2 Nav2 Integration** - Navigation stack configured and tested in simulation  
🔄 **SLAM Capability** - RPLIDAR generating 2D maps of environment  
🔄 **3D-Printed Chassis** - Custom designed for payload capacity and sensor mounting  
🔄 **Jetson Orin Nano Setup** - ARM64 environment with ROS2 Humble running  
🔄 **Simulation Environment** - Gazebo testing for algorithm development  
🔄 **Motor Control** - ESC integration in progress  
🔄 **Localization** - AMCL tuning for outdoor environments  
⏳ **Obstacle Avoidance** - LIDAR-based dynamic obstacle detection (planned)  
⏳ **Waypoint Following** - Delivery mission planner (planned)

## Tech Stack

**Hardware (Current):**
- **Compute**: NVIDIA Jetson Orin Nano (8GB RAM, ARM64)
- **Perception**: RPLIDAR A1 (360° 2D laser scanner, 12m range)
- **Motors**: 2x brushless DC motors with encoders (differential drive)
- **Motor Control**: ESCs for speed control
- **IMU**: MPU6050 (orientation tracking)
- **Chassis**: Custom 3D-printed PLA/PETG frame (Fusion 360 design)
- **Power**: 7.4V LiPo battery
- **Wheels**: Rubber tires for outdoor grip

**Software:**
- **OS**: Ubuntu 22.04 (Jetson Linux)
- **Framework**: ROS2 Humble
- **Navigation**: Nav2 stack
  - Controller Server (DWB local planner)
  - Planner Server (NavFn global planner)
  - Recovery behaviors
  - Waypoint follower
- **SLAM**: SLAM Toolbox (Cartographer alternative being evaluated)
- **Localization**: AMCL (Adaptive Monte Carlo Localization)
- **Simulation**: Gazebo Fortress
- **Containerization**: Docker (WSL2 development on Windows host)
- **Languages**: Python (mission logic), C++ (sensor drivers)

**Development Tools:**
- **CAD**: Fusion 360 (chassis design)
- **3D Printing**: FDM printer (PLA for prototyping, PETG for final build)
- **Version Control**: Git/GitHub
- **Visualization**: RViz2 (debugging navigation)


## Chassis Design

### Design Philosophy
The chassis is fully 3D-printed for rapid iteration and customization. Design prioritizes:
- **Modularity**: Separate layers for electronics, battery, payload
- **Sensor Mounting**: LIDAR elevated for unobstructed 360° view
- **Weight Distribution**: Low center of gravity with heavy components (battery, Jetson) near base
- **Outdoor Capability**: Ground clearance for curbs, textured surfaces, minor obstacles
- **Maintainability**: Easy access to electronics without full disassembly


## Navigation Strategy

### Current Implementation

**LIDAR-Only Perception:**
- No camera means relying entirely on 2D laser scans for obstacle detection
- RPLIDAR provides 360° coverage at ground level (~15cm height)
- Challenge: Cannot detect low obstacles (< LIDAR height) or overhead hazards
- Advantage: Simpler sensor fusion, lower computational load, works in all lighting

**Localization Approach:**
- **SLAM Phase**: Drive robot manually to map campus areas, generate .pgm maps
- **Navigation Phase**: Use AMCL with pre-built map for real-time pose estimation
- **Sensor Fusion**: Combine LIDAR scan matching + wheel odometry + IMU via EKF

**Path Planning:**
- **Global Planner**: NavFn computes shortest path on costmap using Dijkstra's algorithm
- **Local Planner**: DWB (Dynamic Window Approach) generates collision-free trajectories in real-time
- **Recovery Behaviors**: Rotation, backing up, clearing costmap when robot gets stuck


## Current Challenges

### 🔧 Active Issues

**1. Jetson Orin Nano + ROS2 Setup**
- Status: *Mostly resolved*
- Issue: ARM64 package compatibility, GPU acceleration for Nav2
- Solution: Using Docker with NVIDIA runtime; some packages built from source

**2. LIDAR in WSL2 Development**
- Status: *Workaround implemented*
- Issue: USB passthrough for RPLIDAR unreliable in WSL2
- Solution: Using `usbipd` for USB forwarding; considering dual-boot Ubuntu for development

**3. Motor Control via ESC**
- Status: *In progress*
- Issue: ESC expects PWM signals, need smooth velocity control
- Current: Testing PWM generation from Jetson GPIO; may need Arduino bridge

**4. Outdoor Localization Accuracy**
- Status: *Needs tuning*
- Issue: AMCL struggles outdoors (fewer features, sunlight, wind moving objects)
- Plan: Increase particle count, adjust motion model noise, add IMU weighting

**5. Ground Clearance vs. LIDAR Coverage**
- Status: *Design trade-off*
- Issue: Higher ground clearance = better terrain handling, but creates LIDAR blind spot near base
- Current: 5cm clearance with LIDAR at 15cm height; testing if acceptable

**6. Power Budget**
- Status: *Testing*
- Issue: Jetson Orin Nano + motors draw significant current; battery life unknown
- Plan: Measure real-world consumption, size battery appropriately

### 🎯 Known Limitations (LIDAR-Only)

- **Cannot detect**: Overhanging branches, low curbs (< LIDAR height), thin poles (< beam width)
- **Struggles with**: Glass/reflective surfaces, moving vehicles (no classification)
- **No depth perception**: 2D LIDAR doesn't see stairs, ramps, height changes
- **Mitigation**: Conservative speed, larger costmap inflation, rely on known campus map



## Development Roadmap

### Phase 1: Hardware & Basic Navigation ✅ (Current)
- [] Design and print chassis V2
- [] Assemble robot with Jetson + LIDAR
- [x] Get ROS2 running on Jetson
- [] Integrate RPLIDAR
- [x] Create Gazebo simulation model
- [] Configure Nav2 stack
- [ ] Complete motor control integration *(in progress)*
- [ ] Tune AMCL for outdoor localization *(in progress)*

### Phase 2: Campus Mapping 🔄 (Next)
- [ ] Map common campus pathways with SLAM
- [ ] Create merged map of delivery zones
- [ ] Test localization accuracy in different areas
- [ ] Identify problematic zones (glass, reflective surfaces)

### Phase 3: Autonomous Waypoint Following ⏳
- [ ] Implement waypoint mission planner
- [ ] Add delivery state machine (pickup → dropoff → home)
- [ ] Test single-point navigation reliability
- [ ] Implement obstacle handling strategies

### Phase 4: Real-World Testing ⏳
- [ ] Conduct test deliveries on closed paths
- [ ] Gather failure data and iterate
- [ ] Tune navigation for pedestrian-heavy areas
- [ ] Optimize speed vs. safety trade-offs

### Phase 5: System Refinement ⏳
- [ ] Improve battery life / power management
- [ ] Add remote monitoring/teleoperation capability
- [ ] Implement recovery behaviors for edge cases
- [ ] Create user interface for delivery requests

### Future Enhancements (Post-MVP) 💡
- [ ] Add RGB camera for person/vehicle classification
- [ ] Implement social navigation (predict pedestrian paths)
- [ ] GPS integration for global waypoint planning
- [ ] Weather resistance improvements
- [ ] Fleet coordination (multi-robot delivery)


## Resources & References

**ROS2 Navigation:**
- [Nav2 Documentation](https://navigation.ros.org/)
- [SLAM Toolbox Guide](https://github.com/SteveMacenski/slam_toolbox)
- [Nav2 Tuning Guide](https://navigation.ros.org/tuning/index.html)

**Hardware:**
- [RPLIDAR A1 ROS2 Driver](https://github.com/Slamtec/rplidar_ros)
- [Jetson Orin Nano Setup](https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit)

**Learning Resources:**
- [Articulated Robotics ROS2 Tutorials](https://www.youtube.com/@ArticulatedRobotics) - Best ROS2 content
- Probabilistic Robotics (Thrun et al.) - Navigation theory



*Last Updated: February 2026*

# Helix Trajectory Tracking Controller for ArduPilot SITL 🚁

This package implements a trajectory tracking controller for a quadcopter using ArduPilot SITL (Software In The Loop) simulation. The controller generates and tracks a helical trajectory using attitude control commands through MAVROS.

## System Requirements

- **Ubuntu 22.04 (Jammy)**
- **ROS 2 Humble**
- **Gazebo Classic 11**
- **ArduPilot SITL**
- **MAVROS**

## Environment Setup

### 1. ROS 2 and Dependencies Installation

First, install ROS 2 Humble following the [official installation guide](https://docs.ros.org/en/humble/Installation.html).

### 2. Gazebo Classic Installation

```bash
# Install Gazebo Classic 11 & ROS-Gazebo bridge
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update
sudo apt install gazebo ros-humble-gazebo-ros-pkgs 
```

### 3. ArduPilot SITL Setup
```bash
# Clone ArduPilot repository
cd ~/ros2_ws/src
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive

# Install dependencies
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile

# Build SITL (with DDS support if needed)
./waf clean
./waf configure --board sitl --enable-DDS  # Remove --enable-DDS if not using DDS
./waf copter

# Build workspace with ROS 2 Support
cd ~/ros2_ws
colcon build --packages-up-to ardupilot_sitl
source install/setup.bash
```

### 4. ArduPilot Gazebo Plugin Setup
```bash
# Clone the Gazebo plugin
cd ~/ros2_ws/src
git clone https://github.com/ArduPilot/ardupilot_gazebo.git
cd ardupilot_gazebo

# Build and install
mkdir build && cd build
cmake ..
make -j4
sudo make install

# Configure Gazebo paths (add to ~/.bashrc)
echo 'export GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:$GAZEBO_PLUGIN_PATH' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=$HOME/ros2_ws/src/ardupilot_gazebo/models:$GAZEBO_MODEL_PATH' >> ~/.bashrc
echo 'export GAZEBO_RESOURCE_PATH=$HOME/ros2_ws/src/ardupilot_gazebo/worlds:$GAZEBO_RESOURCE_PATH' >> ~/.bashrc
source ~/.bashrc
```

### 5. MAVROS Installation
```bash
# Install MAVROS packages
sudo apt install ros-humble-mavros ros-humble-mavros-extras

# Install GeographicLib datasets
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
```

## Building the Package

1. Create a ROS 2 workspace and clone this repository:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/vybhavraghav/trajectory_tracking.git trajectory_tracking
```

2. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select trajectory_tracking
source install/setup.bash
```

## Running the Simulation

The simulation requires running multiple components in separate terminals. Follow these steps in order:

### Terminal 1: Launch Gazebo Classic
```bash
gazebo --verbose worlds/iris_arducopter_runway.world
```

### Terminal 2: Start ArduPilot SITL
```bash
cd ~/ros2_ws/src/ardupilot/Tools/autotest
./sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map
```

### Terminal 3: Launch MAVROS
```bash
ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@14555
```

### Terminal 4: Start RViz2
```bash
rviz2 -d rviz/trajectory_tracker.rviz
```

### Arm and Set Mode
In the ArduPilot SITL terminal (Terminal 2), after the node setup is done enter:
```bash
# In the MAVProxy console
mode guided
arm throttle
```

In a new terminal, run the trajectory tracking node:
```bash
ros2 run trajectory_tracking trajectory_tracking_attitude_node
```

## Available Nodes

This package contains multiple trajectory tracking and waypoint navigation implementations:

- **trajectory_tracking_velocity_node**: Implementation using velocity control commands for trajectory tracking
- **trajectory_tracking_attitude_node**: Implementation using attitude control commands for trajectory tracking
- **waypoint_navigation_node**: Navigate to waypoints using velocity control
- **waypoint_navigation_attitude_control_node**: Navigate to waypoints using attitude control

Choose the appropriate node based on your needs. For trajectory tracking, both velocity and attitude control versions are available. For waypoint navigation, you can choose between velocity-based or attitude-based control approaches.

## Controller Description

The controller implements a time-based helix trajectory tracking using a cascaded PID control structure:
- Outer loop: Position and velocity control
- Inner loop: Attitude control (handled by ArduPilot)

Key features:
- Smooth trajectory generation using time-based interpolation
- Feed-forward terms for improved tracking
- Anti-windup and maximum tilt angle protection
- Velocity and acceleration limiting
- Visualization of desired and actual trajectories in RViz2

## Parameters

The controller can be configured using ROS 2 parameters:
```bash
# Example parameter configuration
ros2 param set /spiral_attitude_controller pid.pos.p 4.0
ros2 param set /spiral_attitude_controller pid.pos.i 0.1
ros2 param set /spiral_attitude_controller pid.pos.d 3.0
ros2 param set /spiral_attitude_controller trajectory.period 120.0
```

## Topics

### Subscribed Topics
- `/mavros/local_position/pose` - Current drone position (geometry_msgs/PoseStamped)
- `/mavros/local_position/velocity_local` - Current drone velocity (geometry_msgs/TwistStamped)

### Published Topics
- `/mavros/setpoint_raw/attitude` - Attitude setpoints (mavros_msgs/AttitudeTarget)
- `/actual_path` - Actual drone trajectory (nav_msgs/Path)
- `/desired_path` - Desired helix trajectory (nav_msgs/Path)
- `/waypoint_marker` - Current target visualization (visualization_msgs/Marker)

## Important Notes

1. Before running the simulation, ensure the drone is armed and switched to GUIDED mode:
```bash
# In the ArduPilot SITL terminal
arm throttle
mode guided
```

2. The trajectory parameters (radius, height, pitch) can be adjusted in the source code.

3. Make sure all coordinate frames are properly set up in RViz2 for visualization.

4. The controller gains may need tuning based on your specific simulation setup.

## References & Documentation

### Official Documentation
- [ArduPilot SITL Documentation](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html)
- [ArduPilot Gazebo Plugin](https://ardupilot.org/dev/docs/using-gazebo-simulator-with-sitl.html)
- [ROS 2 Humble Installation](https://docs.ros.org/en/humble/Installation.html)
- [MAVROS Package](http://wiki.ros.org/mavros)
- [Gazebo Classic](https://classic.gazebosim.org/)

### Additional Resources
- [ArduPilot ROS 2 Documentation](https://ardupilot.org/dev/docs/ros2.html)
- [ArduPilot GitHub Repository](https://github.com/ArduPilot/ardupilot)
- [ArduPilot Gazebo Plugin Repository](https://github.com/ArduPilot/ardupilot_gazebo)
- [Copter Commands in GUIDED Mode](https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html#set-attitude-target)
- [MAVLink Interface](https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET)
- [MAVROS Control Wiki](https://github.com/mavlink/mavros/blob/master/mavros/README.md#mavros-control)


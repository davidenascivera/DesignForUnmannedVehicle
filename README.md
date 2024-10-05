# Design for Unmanned Vehicle: Drone Estimator Project

This project is part of the **Design for Unmanned Vehicle** course. Throughout the course, we explored all the main topics related to Unmanned Aerial Vehicles (UAVs), including dynamics, control, estimation, and practical implementation.

## Project Overview

The objective of this project is to develop an estimator for a drone. The estimator consists of two main components:

- **Attitude Estimation:** Utilizes a MARG (Magnetic, Angular Rate, and Gravity) solution, incorporating data from the magnetometer and accelerometer.
- **Position Estimation:** Employs UWB (Ultra-Wideband) ranging sensors to determine the drone's position.

<img src="Images/assignment.png" alt="Project Diagram" width="300"/>

The entire project was simulated within a PX4 environment to ensure realistic performance and integration.

## Features

- **MARG-Based Attitude Estimation:** Accurate estimation of drone orientation using sensor fusion.
- **UWB Positioning:** Reliable position tracking through UWB ranging sensors.
- **Simulation Environment:** Comprehensive simulation setup using PX4 and Docker for easy deployment.
- **Modular Codebase:** Organized using `colcon` for efficient package management.

## Getting Started

### Prerequisites

- **Docker:** Ensure Docker is installed on your system. You can download it from [here](https://www.docker.com/get-started).
- **Git:** To clone the repository. Download it from [here](https://git-scm.com/downloads).

### Installation
# Drone Estimator Project

## Installation and Setup

### Clone the Repository

```bash
git clone https://github.com/davidenascivera/DesignForUnmannedVehicle
cd DesignForUnmannedVehicle
```

### Build the Docker Image

The Docker image contains everything needed to run the project, including ROS 2 and PX4 for simulation.

```bash
docker build -t drone-estimator .
```

### Run the Docker Container

After the image is built, run the Docker container:

```bash
docker run -it drone-estimator
```

This will launch a terminal inside the Docker container where you can work with the ROS 2 environment.

### Build the ROS 2 Workspace

Once inside the Docker container, navigate to the controller_mine folder and build the ROS 2 packages using colcon:

```bash
cd controller_mine
colcon build
```

This will compile and build all the ROS 2 packages in the workspace.

### Source the Workspace

After building the workspace, source the environment to make ROS 2 packages available:

```bash
source install/setup.bash
```

### Run the ROS 2 Nodes

Once everything is built and sourced, you can run your ROS 2 nodes using ros2 run. For example, to run your prova_controller node:

```bash
ros2 run prova_controller drone_control_node
```

## Notes:

- Make sure Docker has sufficient resources allocated (e.g., CPU, RAM) to run the simulation.
- If you modify the code, you will need to rebuild the workspace using `colcon build` inside the Docker container.

## Repository Structure

```
controller_mine/
├── install/
├── log/
├── src/
│   ├── position_gazebo_plugin/
│   ├── prova_controller/
│   ├── px4-offboard/
│   ├── rosmsgs/
├── README.md
```

## Running the Simulation

The simulation environment is set up using PX4 within Docker to ensure realistic performance and integration. To run the simulation, execute the following command:

```bash
docker run -it drone-estimator
```

Once inside the container, all necessary colcon packages will be built automatically.

## License

This project is licensed under the Apache License 2.0.

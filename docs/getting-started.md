---
sidebar_position: 2
---

# Getting Started with Physical AI Humanoid Robotics

This comprehensive guide will walk you through setting up your development environment and getting the Physical AI Humanoid Robotics project up and running on your system.

## Prerequisites

Before beginning the setup process, ensure you have the following software installed on your system:

*   **Node.js (version 20.0 or above):** Required for the Docusaurus documentation site development.
    *   [Download Node.js](https://nodejs.org/en/download/)
*   **Python 3.x (3.8 or higher recommended):** For running Python-based ROS 2 nodes, AI algorithms, and scripts.
    *   Check your Python version: `python3 --version`
*   **C++ Compiler (GCC/G++ 9 or higher):** For compiling performance-critical C++ ROS 2 packages.
    *   Check your GCC version: `g++ --version`
*   **ROS 2 (Humble Hawksbill or newer):** The core robotics middleware and development framework.
    *   Follow the official [ROS 2 Installation Guide](https://docs.ros.org/en/humble/Installation.html)
*   **Git:** For version control and repository management.
    *   Install via your system's package manager
*   **Colcon:** ROS 2 build system for compiling packages.
    *   Usually installed with ROS 2: `sudo apt install python3-colcon-common-extensions`
*   **Additional Dependencies:** Gazebo simulation, OpenCV, NumPy, etc.
    *   These will be installed as part of the setup process

## System Requirements

*   **Operating System:** Ubuntu 22.04 LTS (recommended) or other ROS 2 supported distributions
*   **RAM:** 8GB minimum, 16GB recommended
*   **Storage:** 20GB free space for full development environment
*   **Processor:** Multi-core processor with good floating-point performance

## Repository Setup

1.  **Clone the Repository:**

    ```bash
    git clone https://github.com/your-organization/physical-ai-humanoid-robotics.git
    cd physical-ai-humanoid-robotics
    ```

2.  **Initialize the Workspace:**

    Create the standard ROS 2 workspace structure:

    ```bash
    mkdir -p src
    # Any submodules or additional repositories would be added here
    ```

## Development Environment Setup

1.  **Install ROS 2 Dependencies:**

    ```bash
    # Source ROS 2 installation
    source /opt/ros/humble/setup.bash

    # Install additional ROS packages
    sudo apt update
    sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
    ```

2.  **Install Python Dependencies:**

    Install project-specific Python dependencies:

    ```bash
    pip3 install -r requirements.txt
    # If requirements.txt doesn't exist, create one with common robotics libraries
    pip3 install numpy opencv-python matplotlib pyyaml transforms3d
    pip3 install torch torchvision  # For AI/ML components
    ```

3.  **Build ROS 2 Packages:**

    From the root of your ROS 2 workspace, build the packages:

    ```bash
    colcon build --packages-select humanoid_robot_core humanoid_robot_control humanoid_robot_perception
    # Or build all packages:
    colcon build
    ```

4.  **Source the ROS 2 Environment:**

    After building, source your ROS 2 workspace to make its packages available:

    ```bash
    source install/setup.bash
    # To make this permanent, add to your ~/.bashrc:
    # echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    # echo "source ~/physical-ai-humanoid-robotics/install/setup.bash" >> ~/.bashrc
    ```

## Quick Start Commands

Once your environment is set up, use these commands to work with the project:

*   **Build the project:** `colcon build`
*   **Source the workspace:** `source install/setup.bash`
*   **Launch the robot simulation:** `ros2 launch humanoid_robot_bringup simulation.launch.py`
*   **Run a specific node:** `ros2 run package_name node_name`
*   **View available topics:** `ros2 topic list`

## Running the Documentation Site

To run this documentation site locally:

```bash
cd Physical-AI-Humanoid-Robotics  # Navigate to the documentation directory
npm install  # Install documentation dependencies
npm start    # Start the local development server
```

Open your browser to `http://localhost:3000` to view the documentation.

## Next Steps

After completing the setup:

1.  Explore the [System Components](./components.md) to understand the architecture
2.  Review the [ROS 2 Integration](./ros2-integration.md) for communication protocols
3.  Check out the [Development & Maintenance](./development-maintenance.md) guide for best practices
4.  Try running the simulation environment to test the system

## Troubleshooting

*   **ROS 2 Installation Issues:** Ensure you're using a supported Ubuntu version and follow the official ROS 2 installation guide carefully.
*   **Build Errors:** Make sure all dependencies are installed and sourced correctly before building.
*   **Python Import Errors:** Verify that you're using the correct Python version and that packages are installed in the right environment.
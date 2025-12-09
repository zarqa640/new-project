---
sidebar_position: 5
---

# Development & Maintenance

This section outlines guidelines for code style, contributing to the project, testing procedures, and common troubleshooting steps for the Humanoid Robotics Project.

## Code Style

Consistency in code style is crucial for readability and maintainability. Please adhere to the following guidelines:

*   **Python:** Follow [PEP 8](https://peps.python.org/pep-0008/) for code style. Use a linter like `flake8` or `ruff` to enforce it.
*   **C++:** Adhere to the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html) or a similar established standard. Use `clang-format` for automated formatting.
*   **ROS 2:** Follow the [ROS 2 Code Style Guidelines](https://docs.ros.org/en/humble/Contributing/Code-Style-and-Language-Versions.html) specific to Python and C++ ROS packages.

## Contributing Guidelines

We welcome contributions to the Humanoid Robotics Project! To ensure a smooth collaboration process, please follow these steps:

1.  **Fork the Repository:** Start by forking the main project repository to your GitHub account.
2.  **Create a New Branch:** For each new feature or bug fix, create a new branch from `main` (or the designated development branch):
    ```bash
    git checkout -b feature/your-feature-name
    ```
3.  **Implement Changes:** Write your code, adhering to the established code style and best practices.
4.  **Write Tests:** Ensure your changes are covered by appropriate unit and integration tests.
5.  **Run Tests:** Before submitting, run all tests to ensure no existing functionality is broken.
    *   For Python tests: `cd src; pytest`
    *   For ROS 2 packages: `colcon test` from your workspace root.
6.  **Commit Changes:** Write clear and concise commit messages.
    ```bash
    git commit -m "feat: brief description of your feature"
    ```
7.  **Push to Your Fork:** Push your branch to your forked repository.
8.  **Create a Pull Request:** Open a pull request against the main project repository, providing a detailed description of your changes.

## Testing Procedures

Thorough testing is vital to maintain the quality and reliability of the robotics software.

*   **Unit Tests:** Focus on testing individual functions and modules in isolation.
*   **Integration Tests:** Verify the interaction between different components and nodes.
*   **System Tests:** Validate the overall behavior of the robot in simulated or real-world environments.

### Running Tests

*   **Python Tests:**
    ```bash
    cd src
    pytest
    ```
*   **ROS 2 Package Tests:**
    ```bash
    colcon test --packages-select <your_package_name>
    colcon test-result --all # View test results
    ```

## Troubleshooting

This section provides solutions to common issues you might encounter during development.

### 1. "Permission Denied" when running Docusaurus

*   **Issue:** You might encounter `sh: 1: docusaurus: Permission denied` when running `npm start` in the Docusaurus directory.
*   **Solution:** The `docusaurus` executable might not have proper execution permissions. Navigate to `Physical-AI-Humanoid-Robotics/node_modules/.bin/` and ensure `docusaurus` is executable:
    ```bash
    chmod +x Physical-AI-Humanoid-Robotics/node_modules/.bin/docusaurus
    ```
    Then retry `npm start`.

### 2. ROS 2 Package Not Found

*   **Issue:** `ros2 run` or `ros2 launch` commands fail with "package not found".
*   **Solution:** Ensure your ROS 2 workspace has been sourced correctly. From the root of your workspace:
    ```bash
    source install/setup.bash
    ```

### 3. Build Errors in ROS 2

*   **Issue:** `colcon build` fails with compilation errors.
*   **Solution:** Check compiler output for specific error messages. Ensure all dependencies are installed. Consider cleaning your build space:
    ```bash
    rm -rf build install log
    colcon build
    ```

This documentation will be continuously updated with more common issues and their resolutions.
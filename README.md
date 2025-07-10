# Robot Development Toolkit (RDT)

[![Build Status](https://img.shields.io/badge/build-passing-brightgreen)](https://github.com/hexakinetica/rdt-core/actions)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Contributions Welcome](https://img.shields.io/badge/contributions-welcome-brightgreen.svg?style=flat)](https://github.com/hexakinetica/rdt-core/pulls)

**A production-grade, open-source C++ architecture for industrial robot controllers.**

This repository contains the full source code for the Robot Development Toolkit (RDT), the project at the heart of the book "[The RDT Book: Building a Real-World Robot Control System](https://github.com/hexakinetica/rdt-book)".

![Screenshot](HMI_screen.jpg)


🎥 *Watch on YouTube*: [Demo Video](https://youtu.be/JDtNz1AyA9I)


## ⚠️ Project Status: Public Beta Version

**This is a beta-level demonstration project.** The primary goal of this codebase is to serve as a **learning tool** and a **living architectural blueprint**. It is designed to be read, understood, and experimented with.

**What this means for you:**

*   **There are bugs and known issues.** The system is functional for its intended "happy path" scenarios but has not been hardened for production use. You will find rough edges, TODOs, and plenty of "косяков" (as we say).
*   **The architecture is the main feature.** The core value here is not in the flawless execution of every edge case, but in the clear, modular, and layered design that demonstrates professional engineering principles.
*   **This is a perfect place to contribute!** Finding and fixing these bugs, improving the build system, or adding features discussed in the book are fantastic ways to dive into industrial robotics and make a real impact on the project.

Think of it as a car with a beautifully designed engine and chassis, but the radio might crackle and the glove box doesn't quite close. The fundamentals are solid, and we invite you to help us finish the interior.

##  filosofía: A Field Manual for Robot Control

This is not just another academic simulator. RDT is a "field manual" in C++ form, designed to bridge the huge gap between university-level robotics (like basic ROS scripts) and the robust, deterministic, and maintainable systems required on a factory floor.

The project is architected from the ground up to demonstrate key principles of industrial automation software:

*   **Strict RT/NRT Separation**: A deterministic real-time (RT) core for motion execution, decoupled from a flexible non-real-time (NRT) domain for planning and UI.
*   **Layered & Decoupled Architecture**: Using interfaces and dependency injection to create modular, testable, and extensible components.
*   **Hardware Abstraction Layer (HAL)**: A pluggable HAL that allows the system to run with a `FakeMotionInterface` (for simulation) or a `UDPMotionInterface` (for real hardware) without changing the core logic.
*   **Modern C++**: Leveraging C++17/20 features like `std::jthread`, `std::optional`, and smart pointers for robust, safe, and readable code.

## Architectural Overview

The project is structured as a series of modular libraries, each representing a key architectural component of the control system. The main application in `1_RobotControl_main` assembles these components into a cohesive whole.

| Module Directory | Architectural Role | Description |
| :--- | :--- | :--- |
| `1_RobotControl_main` | **HMI / Application** | The main executable, Qt-based GUI panels, and the "composition root" where all modules are assembled. |
| `data_types_rdt` | **Core Data Types** | Defines fundamental, strongly-typed units (`Meters`, `Radians`) and data structures (`Pose`, `AxisSet`). |
| `state_data_nrt` | **State Bus (SSOT)** | A thread-safe "blackboard" (`StateData`) for sharing state across NRT modules. |
| `kinematic_solver_nrt` | **Kinematics Engine** | Solves Forward/Inverse Kinematics. Wraps the KDL library. |
| `interpolator_nrt` | **Trajectory Interpolator** | Implements the Strategy pattern for generating different motion profiles (LIN, PTP). |
| `robot_controller_nrt`| **NRT Orchestrator** | The "brain" of the NRT domain. Manages system state and orchestrates the planner. |
| `trajectory_queue_lf`| **RT/NRT Bridge** | A lock-free, single-producer, single-consumer (SPSC) queue. |
| `motion_manager_rt` | **Real-Time Core** | The deterministic, high-priority thread (`MotionManager`) that executes trajectories. |
| `motion_interface_hal`| **Hardware Abstraction** | Defines the `IMotionInterface` contract and its concrete implementations (simulated/UDP). |
| `logger` | **Logging Service** | A centralized, thread-safe logging utility. |

---

## Getting Started

### 1. Dependencies

This project relies on several third-party libraries. You will need:
*   **Build System**: CMake (version 3.16+)
*   **Compiler**: A C++20 compliant compiler (GCC)
*   **Core Libraries**:
    *   **Qt6**: For the Human-Machine Interface (HMI).
    *   **Eigen 3.4.0**: For linear algebra.
    *   **Orocos KDL**: For kinematics and dynamics.
    *   **TBB (oneapi-tbb)**: For high-performance parallel operations.
    *   **OCCT (occt/occt-install)**: For 3D visualization and STEP file parsing.
    *   **TinyXML2**: For configuration or data serialization.
    *   **FreeType**: A dependency for OCCT or text rendering.

### 2. A Note on Library Paths (Your Contribution Opportunity!)

**Heads up:** Currently, the project uses a hardcoded path in the root `CMakeLists.txt` to find these dependencies:
```cmake
set(LIBS_DIR "${CMAKE_CURRENT_LIST_DIR}/../../libs")
```
This means you must place all the required libraries into a `libs` folder located two directories above your build folder.

**This is not ideal!** but had a reason.

### 3. Building the Project

1.  **Clone the repository:**
    ```bash
    git clone https://github.com/YourUsername/rdt-project.git
    cd rdt-project
    ```
2.  **Set up dependencies:**
    Make sure all libraries listed above are available in the path specified in the CMake file.

3.  **Build the controller:**
    ```bash
    mkdir build
    cd build
    cmake ..
    make -j$(nproc) 
    ```
4.  **Run the application:**
    The main executable will be in the `1_RobotControl_main` subdirectory of your build folder.
    ```bash
    ./1_RobotControl_main/RobotControl_main
    ```
5.  **Run Unit Tests:**
    Each module has its own test executable (e.g., `interpolator_main`). You can run them individually.
    ```bash
    ./interpolator_nrt/interpolator_main
    ```

---

## How to Contribute

We are thrilled you're interested in contributing! This project is driven by the community, and we welcome all contributions.

1.  **Code & Features**: Found a bug? Have an idea for a new feature discussed in the book (like Spline motion)?
    *   Fork the **[Controller Repository](https://github.com/hexakinetica/rdt-project)**.
    *   Create a new branch for your feature or fix.
    *   Submit a Pull Request! Please open an issue first to discuss larger changes.

2.  **Book & Documentation**: Found a typo, a logical error, or have a suggestion to improve the explanations in the book?
    *   Fork the **[Book Repository](https://github.com/hexakinetica/rdt-book)**.
    *   Submit a Pull Request with your proposed changes.






### 🧩 Related Projects

- More robotics content on our [YouTube channel](https://www.youtube.com/@hexakinetica)




### Contributing

We welcome contributions from the community! If you'd like to improve the models or add new functionality, please submit a pull request.

### Contact

Email: contact@hexakinetica.com
Website: https://www.hexakinetica.com



### Disclaimer

These models are not official KUKA models and are not affiliated with, endorsed by, or approved by KUKA Robotics. All trademarks, product names, and company names mentioned are the property of their respective owners.
The models are provided for visualization and educational purposes only and are not intended for manufacturing, engineering, or commercial use. The authors and distributors provide these models "as is" without any guarantee of accuracy, completeness, or fitness for any particular purpose.
If you are the copyright holder or believe any material posted violates your rights, please contact us to request removal.

### License
This project is licensed under the MIT License.
You are free to:
Use, copy, modify, merge, publish, distribute, sublicense, and sell copies of the software
Under the following terms:
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
The software is provided "as is", without warranty of any kind, express or implied.
For more information, see the full license text: https://opensource.org/licenses/MIT

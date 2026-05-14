# Intro to Robotics: Robotic Manipulator Kinematics & Perception

This repository contains the implementation of kinematics, perception, and path-planning algorithms for a robotic manipulator (Pincher). The core implementation and all related tasks are located in the **`PoE Approach`** directory, which uses the Product of Exponentials formula for modeling the robot's kinematics. 

## 🎥 Robot Demonstration

*The following video is a demonstration of Task 4 which was unstacking a complex pyramid. The task covers the basic functionality of other tasks as well.*


https://github.com/user-attachments/assets/fac1602e-1be8-4883-a8cb-f2a384e5335b



## 🌍 World & Camera Frames

The image below identifies the robot's base (world) frame alongside the camera frames used for the perception pipeline.

<img width="509" height="585" alt="PHOTO-2026-05-07-17-36-25" src="https://github.com/user-attachments/assets/0aa71fc7-47d2-49ae-97e5-20b1df4ad4e7" />


## 📂 Project Structure & Implementation

The active working directory for this project is the **`PoE Approach`** folder. It contains MATLAB scripts (`.m`) and Live Scripts (`.mlx`) divided into several key robotics modules:

### 1. Kinematics (Product of Exponentials)
* **`ForwardKinematics.mlx` / `pincherFK.m`**: Computes the end-effector pose given the joint angles using the PoE formula.
* **`Inverse_Kinematics.mlx` / `findSolution.m`**: Calculates the required joint angles to reach a desired target pose.
* **`VecTose3.m`**: Utility for converting spatial velocity vectors to $se(3)$ matrix representations.

### 2. Perception Pipeline
* **`perception.mlx`**: Main live script for the computer vision and object detection workflow.
* **`perception_pipeline_3d_regionprops.m`**: Processes 3D region properties to detect and localize objects in the workspace.
* **`camera_transform.mat`**: Contains the calibration and transformation matrices linking the camera frame to the world frame.

### 3. Collision & Safety
* **`checkSelfCollision.m` & `check_collision.mlx`**: Ensures the manipulator does not intersect with its own links during motion.
* **`checkJointLimits.m`**: Validates that calculated inverse kinematics solutions do not exceed the physical limits of the servos.

### 4. Tasks & Planning
* **Tasks 1 through 4** (`task1_updated.mlx`, `task_2Ipdated.mlx`, `task3_double_updated.mlx`, `task_4_updated.mlx`): Sequential milestone scripts combining kinematics and perception to achieve specific goals, such as picking and placing objects.
* **`Pick_Place.mlx`**: A dedicated routine integrating path planning, jaw positioning (`positionJaw.m`), and kinematics to execute a complete pick-and-place sequence.
* **`buildPincherTwin.mlx`**: Sets up a digital twin / simulation environment for the Pincher arm.

## 🚀 Getting Started

### Prerequisites
* **MATLAB**: The project heavily relies on MATLAB Live Scripts (`.mlx`) and standard functions (`.m`).
* **Robotics System Toolbox**
* **Image Processing / Computer Vision Toolbox** (For the perception pipeline).
* Intel Real Sense Depth Camera
* Pypose has to be loaded on the Arbotix Phantom Pincher X

### Running the Code
1. Clone the repository.
2. Navigate to the `PoE Approach` directory in MATLAB.
3. Open `buildPincherTwin.mlx` to initialize the robot model.
4. Run the individual task files (e.g., `task1.mlx`) or explore the `PoE_approach.mlx` script to see the mathematical foundation in action.

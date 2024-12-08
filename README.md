# Robotics and Embedded Systems Track - Robot Arm Project

Welcome to the **Google Developer Groups on Campus Robotics and Embedded Systems Track at Covenant University**. This repository showcases the ongoing development of a **robotic arm** capable of learning to **pick and place objects**.

---

## Project Overview

Our goal is to build a robotic system that can:
- Detect and manipulate objects in a 3D space.
- Perform pick-and-place tasks autonomously using reinforcement learning and inverse kinematics.

We use **PyBullet**, **Python**, and **matplotlib** to simulate and visualize the robot arm's movements.

---

## Installation
To run the simulation, ensure the following dependencies are installed:

- **Python 3.12+**
- **PyBullet:** `pip install pybullet`
- **NumPy:** `pip install numpy`
- **matplotlib:** `pip install matplotlib`

---

## How to Run

1. Clone this repository:
   ```bash
   git clone https://github.com/iitimii/robot_arm.git
   cd robot-arm
   ```

2. Run the simulation:
    ```bash
    python arm_pick_nd_place.py
    ```

## Features

### Simulation Environment
- **Physics Engine:** PyBullet provides a realistic environment for simulating the robot's movements and interactions.

### Visualization
- **Joint angles** plotted over time.
- **3D trajectory** of the end effector.

### 3D Models
- **Robot Arm Model** (To be added).
- **Objects Model** objects to be picked and placed by the robotic arm. (To be added).

### Hardware Code
- **Microcontroller Interface** (To be added).
- **Motor Control** (To be added).

### Reinforcement Learning
- **Task Learning** using RL algorithms to optimize pick-and-place actions. (To be added).

---

## Roadmap

### Current Phase
- Implementing proof-of-concept pick-and-place tasks using the Kuka Robot arm model.

### Next Steps
- Develop and integrate our custom arm model.
- Integrate camera for object detection.
- Implement Gymnasium and StableBaselines for reinforcement learning.
- Use reinforcement learning for dynamic task adaptation.
- Build a physical version of the robot.
- Implement a Vision Language Action model for improved task understanding and interaction.

---

## Contributing

We welcome contributions! To get started:

1. Fork the repository.
2. Create a new branch for your feature: `git checkout -b feature-name`.
3. Commit your changes: `git commit -m 'Add feature'`.
4. Push to your branch: `git push origin feature-name`.
5. Create a pull request.

---

## License

This project is licensed under the MIT License. See the [LICENSE](./LICENSE) file for more details.

---

## Contact

For questions or collaboration, feel free to reach out to the Robotics and Embedded Systems Track team:

- **Lead:** Timilehin Owolabi
- **Email:** [timilehin.owolabi@stu.cu.edu.ng](mailto:timilehin.owolabi@stu.cu.edu.ng)

---

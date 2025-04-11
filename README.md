# CI-and-QA-for-Robotics-Software

[![Codacy Badge](https://app.codacy.com/project/badge/Grade/456677987e644642b1ceac03558b15ee)](https://app.codacy.com)
[![CI](https://github.com/zaid3727/CI-and-QA-for-Robotics-Software/actions/workflows/ci.yml/badge.svg)](https://github.com/zaid3727/CI-and-QA-for-Robotics-Software/actions/workflows/ci.yml)

---

## 🚦 Overview

This project demonstrates how to apply **Continuous Integration (CI)** and **Quality Assurance (QA)** to a robotics software system using:

- ✅ A **State Machine** (`SMACH`) and a **Behaviour Tree** (`py_trees_ros`) to handle robot safety logic
- ✅ **Unit & integration tests** with `pytest` and `unittest`
- ✅ **CI** via GitHub Actions to run tests on every push and PR
- ✅ **Code quality checks** using Codacy, PEP8, flake8, and docstring linting

All testing is done on a robot simulated using the **Robile platform**, where:
- 🪫 Low battery triggers rotation
- 🟥 Obstacles trigger a stop

---

## 📁 Repository Structure

```
.
├── .github/workflows/       # CI and Codacy workflows
├── safety_robile/           # ROS 2 package with BT and SMACH logic
│   ├── safety_robile/       # Python code
│   └── test/                # Linting tests (flake8, pep257)
├── tests/                   # Unit/integration tests for BT & SM
├── Robile/                  # Simulation package (custom robot)
├── py_trees_ros/            # py_trees ROS2 wrapper
├── executive_smach/         # SMACH ROS2 version
├── robile_*                 # Launch, description, and nav packages
├── requirements.txt         # Python deps
├── .codacy.yml              # Codacy config
└── README.md                # This file
```

---

## 🛠️ Setup Instructions

> 💡 Requires: Ubuntu 22.04, ROS 2 Humble, and Python 3.10+

### 1. Install ROS 2 Humble

```bash
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2-latest.list
sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions python3-rosdep
source /opt/ros/humble/setup.bash
sudo rosdep init || true
rosdep update
```

### 2. Clone & Build the Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/zaid3727/CI-and-QA-for-Robotics-Software.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### 3. Install Runtime Dependencies

```bash
sudo apt install -y ros-humble-py-trees-ros-interfaces
pip install -r src/CI-and-QA-for-Robotics-Software/requirements.txt
pip install git+https://github.com/splintered-reality/py_trees_ros.git
```

---

## 🚀 Running the System

### Run Behaviour Tree:

```bash
ros2 run safety_robile safety_robile_BT
```

### Run State Machine:

```bash
ros2 run safety_robile safety_robile_SMACH
```

### Simulate Inputs:

```bash
# Simulate low battery
ros2 topic pub /battery_voltage std_msgs/msg/Float32 "data: 15.0"

# Simulate obstacle
ros2 topic pub /scan sensor_msgs/msg/LaserScan "{ranges: [0.2, 0.3, 0.5]}"
```

---

## ✅ Continuous Integration (CI)

Every push and pull request triggers:

- 🧪 Tests via `pytest` in `tests/`
- ✅ Lint checks (`flake8`, `pep257`)
- 📦 ROS environment setup (via GitHub Actions)

Live CI badge:  
https://github.com/zaid3727/CI-and-QA-for-Robotics-Software/actions

---

## 🧪 Quality Assurance (QA)

This repo includes:

- Unit tests for all behaviours and states: `tests/`
- Linting via:
  - `flake8`
  - `pep257`
  - `pylint` (enabled in `.codacy.yml`)
- Codacy integration to report:
  - Code smells
  - Complexity
  - Style violations

Codacy runs on every PR automatically.

---

## 👥 Team Members

- [Mohammed Zaid Nidgundi](https://github.com/zaid3727)
- [Jay Parikh](https://github.com/jp-droid)
- [Talha Riyaz Shaikh](https://github.com/Talha-Riyaz-Shaikh)
---

## 📸 Screenshots (Required for Submission)

Add screenshots showing:
- ✅ Passing GitHub Actions CI
- ✅ Codacy PR analysis
- 🤖 Robot reacting to simulated inputs

---

Let me know if you want me to commit this directly, create the Codacy token setup guide, or prepare a Codacy config that enforces custom PEP8 rules. You're in top shape for submission 🏁

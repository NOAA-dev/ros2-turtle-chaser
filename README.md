# ROS 2 Turtle Chaser 🐢🤖

A ROS 2 multi-node project where a controller turtle autonomously **chases and eliminates the nearest target turtles** in a dynamic environment using proportional control, custom services, and messages.

This project demonstrates **real ROS 2 system design**, asynchronous service handling, and robust state management beyond basic tutorials.

---

## 📌 Project Overview

The system consists of:
- A **Controller node** that continuously selects the nearest target and pursues it smoothly
- A **Manager (God) node** that spawns turtles, tracks alive turtles, and processes kill requests
- **Custom ROS 2 messages and services** for structured inter-node communication
- A **bringup package** to launch the entire system together

The controller turtle rotates toward the target, moves smoothly, and sends a kill request when it reaches close proximity.

---

## 🧩 Architecture

### Packages

src/
├── trial_interfaces/
│ ├── msg/AliveStatus.msg
│ ├── srv/Killer.srv
│
├── turtle_chaser/
│ ├── Turtle_controller.py
│ ├── turtle_God.py
│
└── program_bringup/
└── launch/turtle_chaser.launch.xml


### Node Responsibilities

- **TurtleControllerNode**
  - Subscribes to alive turtle states
  - Selects the nearest target
  - Uses proportional control for smooth pursuit
  - Sends async kill requests safely

- **TurtleGodNode**
  - Periodically spawns turtles
  - Maintains alive turtle state
  - Handles kill requests and removes turtles
  - Publishes alive turtle data

---

## 🎮 Control Logic

- Nearest-target selection using Euclidean distance
- Angular error normalization using: atan2(sin(θ_error), cos(θ_error))

- Rotate-first strategy for stable motion
- Velocity saturation and dead-zones for smooth movement
- Idempotent kill handling to avoid duplicate requests

---

## 🚀 How to Run

### Prerequisites
- ROS 2 Jazzy
- turtlesim

### Build
```bash
colcon build
source install/setup.bash

ros2 launch program_bringup turtle_chaser.launch.xml

```
This launches:

turtlesim_node
Turtle controller
Turtle manager (God node)

📷 Demo

The screenshots below show the system running, with turtles being dynamically spawned and chased:

Turtlesim window showing pursuit and elimination

Launch logs confirming controller, manager, and turtlesim nodes running together

(You can add screenshots or a GIF in a /media folder for better visualization.)

🧠 Key Learnings

Designing multi-node ROS 2 systems

Using custom messages and services correctly

Handling asynchronous service calls safely

Debugging race conditions and state desynchronization

Writing smoother control logic for dynamic targets

Building launch files for clean system startup

📈 Skill Level

This project goes beyond basic ROS tutorials and reflects:

System-level thinking

Runtime debugging ability

Practical robotics control concepts

🔮 Future Improvements

Support for multiple controller turtles

Dynamic parameter tuning

Visualization of target selection

Porting logic to real robots or Gazebo simulation

📎 Tech Stack

ROS 2 Jazzy

Python

turtlesim

👤 Author

Second-year engineering undergraduate at COEP PUNE INDIA exploring robotics, ROS 2, and autonomous systems through hands-on projects.

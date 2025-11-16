# MRTP
This repository contains material related to the textbook "Mobile Robotics: Theory and Practice" (aka MRTP -- https://robotics.ucmerced.edu/MRTP).
If you cite the book, please use the following:

    @misc{MRTP,
    author = {Stefano Carpin},
    title = {Mobile Robotics: Theory and Practice},
    url = {http://robotics.ucmerced.edu/MRTP},
    year={2025}
    } 

See https://github.com/stefanocarpin/MRTP/wiki/Installation-Intructions for instructions on what to install to run the examples presented in the textbook.

Once the installation is complete, to build the code examples open a shell, clone the repository, 

    git clone https://github.com/stefanocarpin/MRTP

move to the folder MRTP/MRTP, and run

     colcon build
     
Important: as pointed out in the documentation, to use ROS you must first source the setup. See also https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html#setup-environment

## Docker
If for some reason you can't install ROS2 on your machine (working with Windows or Mac) you can use a container to provide the necessary environment through Docker.

[See Docker README](docker/README.md).


````markdown
# Final Project Instructions

## Team Rules
- The final project may be completed individually or in teams of up to three students (no exceptions).
- Team registration is available under the “Announcements” section.
- If working individually, no registration is required. Any student not listed on a team will automatically be considered to be working alone.
- All teams must register by **December 5**.

## Presentations
- Project presentations take place during the last week of classes.
- **Individual projects:** Present during your assigned lab section.
- **Team projects:** Present in any of the lab sections assigned to your team (no exceptions).

---

# Simulation Environment Setup

First, ensure you have the **latest version of the MRTP repository**, as the final project files were just added.  
After rebuilding the MRTP workspace, launch the simulation environment with:

```bash
ros2 launch gazeboenvs tb4_warehouse.launch.py
````

To run both Gazebo and RViz (useful for debugging):

```bash
ros2 launch gazeboenvs tb4_warehouse.launch.py use_rviz:=true
```

### Initial Robot Pose

The robot starts in the environment with:

* **x = 2.12**
* **y = -21.3**
* **yaw = 1.57 radians**

The localization algorithm is initialized with these values.

---

# Environment Description

The warehouse environment contains **two stationary human characters**.
Their footprints are included in the map loaded into Gazebo and available to the robot via the `/map` topic.

Your controller must determine:

1. Whether the humans have moved to a different location.
2. If they have moved, identify their new (permanently relocated) position.

   * Humans, if moved, remain at the new location and do not move further.

---

# Project Requirements

To solve this problem, you must:

### 1. Navigation

Use the navigation library provided in class to move the robot through the environment using the **Nav2 stack**.

### 2. Map Retrieval

Access the environment map through the `/map` topic.

### 3. Environment Change Detection

Use the robot’s **range finder** to detect changes (e.g., if humans have moved).

### 4. Robot Localization

Use the `/amcl_pose` topic to obtain the robot’s current location in the environment.

---

# Deliverables

1. **Zipped workspace**

   * Include all code in a self-contained ROS2 workspace.
   * Upload to CatCourses.

2. **One-page PDF description**

   * Describe your solution approach.
   * Upload to CatCourses.

3. **Presentation and demo**

   * Present during lab hours in the final week.

---

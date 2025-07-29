# doozy_ws
Cartesian Path Planning and Perception-Based Manipulation

# Static Path Planning

## Overview

This task involves path planning and execution of a trajectory over a workobject using a CSV-based set of waypoints. The robot used was a 6-DOF manipulator integrated with MoveIt and hardware interfacing.

## Steps Followed

### 1. Robot Setup (same as Task 2 & 3)

* Design in SolidWorks → S2URDF conversion → Add virtual joint.
* MoveIt setup with MoveIt Setup Assistant.
* Tested joint simulation using `demo_gazebo.launch`.

### 2. Controller Integration

* Developed a hardware interface node that:

  * Converts joint angles from radians to degrees.
  * Publishes the converted values to `/arduino/arm_actuate` topic.
* Components involved:

  * `angles_converter.py`: Python node converting and publishing joint states.
  * `AnglesConverter.srv`: Custom service to handle conversion requests.
  * `arduinobot_interface.{h,cpp}`: C++ nodes for service-client communication.
  * `automatoncontroller.launch`: Launch file to bring up all controller nodes.

### 3. Workobject Design

* Designed a **workobject** in **CATIA**.
* Extracted waypoint **CSV data** directly from the CATIA sketch using **VBA macros**.

### 4. Workobject Import

* Imported the 3D workobject mesh into RViz using "**Mesh from File**" option.
* Positioned the mesh **0.4 meters in front** of the robot base.

### 5. Path Planning Node

* Created a node `path_planning.py` that:

  * Reads the CSV file.
  * Transforms the coordinates.
  * Sends pose goals to MoveIt via `go_to_pose()`.
  * Optionally visualizes trajectory using **Matplotlib (3D interactive)** comparing CATIA points vs robot executed path.

### 6. Execution

* Final trajectory was executed using `move_group.execute()` for continuous and smooth execution.

---

# Perception-Based Pick and Place

## Overview

This task involves building a custom 6-DOF robotic manipulator with an integrated vision system to perform a perception-driven pick-and-place operation. The complete process includes designing, modeling, simulation, and integrating perception.

## Steps Followed

### 1. Robot Design and URDF Creation

* Designed a custom **6-DOF robot** using **SolidWorks** (from part modeling to full assembly).
* Used the **S2URDF** plugin to generate a ROS-compatible **URDF**.

### 2. URDF Enhancement

* Manually edited the URDF to:

  * Add a **virtual joint** connecting the robot base to the world.
  * Add a **camera link and joint**, and include the corresponding **Gazebo plugins** for RGB-D simulation.

### 3. MoveIt Setup

* Installed MoveIt using a **custom HTML configuration**.
* Generated a **MoveIt package** using the **MoveIt Setup Assistant**.
* Verified the **joint simulation** using `demo_gazebo.launch`.

### 4. Perception Integration

* Launched the camera plugin with topics like `/depth_camera/image_raw` and `/depth_camera/depth/image_raw`.
* Integrated the camera topics in **RViz** using Image and Camera displays.
* Used `rqt_image_view` for raw image visualization.

### 5. Object Detection Node (Python)

* Developed a perception node to detect red objects using **OpenCV color thresholding**.
* The node publishes the 3D position of the object to `/object_pose`.
* A separate node publishes the RViz marker to `/visualization_marker` for debugging.

### 6. Challenges Faced

* Simulated time issues (`/clock` not published).
* `/object_pose` not being published due to missing camera input.
* Camera plugin errors and missing markers in RViz.
* Ensured object visibility from camera by proper camera placement and object distance.

---

**Note**: Both tasks were developed using **ROS Noetic**, simulated in **Gazebo**, visualized in **RViz**, and driven by **MoveIt**.


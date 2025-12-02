# Project Overview

This project utilizes image processing and potential-field methods to detect obstacles, boundaries, and goals in an environment, and then generate and visualize navigation paths.

---

## Data Structure

### `txt_files/`
Stores the **basic coordinates** required by the system.

### `images/`
Contains additional **output images** obtained during detection and visualization.

---

## Detection Modules (OpenCV)

The following files include OpenCV-based code used to identify key components of the environment:

- **`red_detections.py`**  
- **`blue_and_red_detection.py`**  
- **`detect_green_cube.py`**  

These scripts determine:
- map boundaries  
- obstacles  
- the goal point  

---

##  Visualization Modules

These modules generate visual outputs for the system:

- **`visualization_fields.py`** – visualizes the **potential fields**  
- **`visualization_path.py`** – visualizes the **computed navigation path** on the generated map  

---

##  Potential Field Generation

- **`mereology_generating.py`**  
  Contains the algorithm responsible for generating the **potential fields** based on detected obstacles, boundaries, and target positions.




# ar_drone_wrapper

## Overview

This repository contains a ROS package for controlling the Parrot AR Drone 1.0. The package is intended to be used with ROS Melodic and is based on the [python-ardrone project](https://github.com/venthur/python-ardrone).

## Installation

To install the package, navigate to your ROS `src` folder and clone this repository:

```bash
cd ~/catkin_ws/src
git clone <repository-url>
```

### Setting Up Virtual Environment

Create a virtual environment and activate it:

```bash
virtualenv env
source env/bin/activate
```

### Installing Dependencies

Install the required Python dependencies:

```bash
pip install -r requirements.txt
```

### Compiling the Project

Compile the ROS workspace:

```bash
cd ~/catkin_ws
catkin_make
```

### Additional Dependencies

The `ffmpeg` library needs to be installed to receive the drone's camera image:

```bash
sudo apt install ffmpeg
```

## Usage

### Running the ROS Node

To run the ROS node, execute the following command:

```bash
rosrun ar_drone_wrapper drone_driver.py
```

### Topics

The package publishes and subscribes to various ROS topics needed for the drone to work:

- **Published Topics**:
  - `/ardrone/cmd_vel`
  - `/ardrone/nav_data`
  - `/ardrone/takeoff`
  - `/ardrone/land`
  - `/ardrone/reset`
  - `/ardrone/odom`
  - `/ardrone/front_camera/raw_image`
  - `/ardrone/front_camera/raw_image_compressed`
  - `/ardrone/altitude`
  - `/ardrone/battery`
  - `/ardrone/tf`
  - `/ardrone/mag`

- **Subscribed Topics**:
  - `/ardrone/cmd_vel`: Command velocity for controlling the drone.
  - `/ardrone/takeoff`: Command to make the drone take off.
  - `/ardrone/land`: Command to make the drone land.
  - `/ardrone/reset`: Command to reset the drone.

## Code Explanation

### Importing Libraries

The code imports necessary libraries including ROS for communication, OpenCV for image processing, and the AR Drone library for drone control.

### Publisher and Subscriber Setup

The code sets up publishers for navdata and camera images, and subscribers for drone control commands (takeoff, land, reset, and movement).

### Helper Functions

- `images(drone)`: Captures and publishes the front camera image of the drone.
- `nav_data(drone)`: Captures and publishes the navigation data of the drone.
- `cmd_vel(move_data)`: Controls the drone's movement based on command velocity messages.
- `takeoff(_data)`: Commands the drone to take off.
- `land(_data)`: Commands the drone to land.
- `reset(_data)`: Commands the drone to reset.

### Main Function

The `main` function initializes the ROS node, connects to the drone, and sets up the necessary subscribers. It then starts the ROS spin loop to keep the node running.

## Navdata Structure

The navdata structure contains various telemetry data from the drone, including battery level, altitude, velocity, and orientation.

```json
{
  "vy": 0.0,
  "phi": -3,
  "psi": -115,
  "num_frames": 0,
  "battery": 79,
  "altitude": 0,
  "ctrl_state": 1,
  "vx": 0.0,
  "theta": -1,
  "vz": 0.0,
  "drone_state": {
    "acq_thread_on": 1,
    "angles_out_of_range": 0,
    "ctrl_watchdog_mask": 0,
    "video_mask": 0,
    "com_watchdog_mask": 0,
    "fw_ver_mask": 0,
    "video_thread_on": 1,
    "adc_watchdog_mask": 0,
    "com_lost_mask": 0,
    "control_mask": 0,
    "user_el": 0,
    "atcodec_thread_on": 1,
    "command_mask": 1,
    "user_feedback_start": 0,
    "altitude_mask": 1,
    "fw_file_mask": 1,
    "navdata_thread_on": 1,
    "vbat_low": 0,
    "fly_mask": 0,
    "vision_mask": 0,
    "ultrasound_mask": 0,
    "emergency_mask": 0,
    "fw_upd_mask": 0,
    "pic_version_mask": 1,
    "cutout_mask": 0,
    "navdata_bootstrap": 0,
    "motors_mask": 0,
    "navdata_demo_mask": 1,
    "timer_elapsed": 0
  },
  "vision_flag": 0,
  "seq_nr": 23636
}
```

## TODO

- Debug and fix the navdata publishing issue.
- Add tag detection functionality.

## License

This project is licensed under the MIT License.

## Contributions

Contributions are welcome! Please fork this repository, make your changes, and submit a pull request.

## Contact

For any questions or support, please open an issue in this repository.

---

# Gesture Mouse

A webcam controlled virtual mouse that lets you move the OS cursor, click, drag,
and right click using hand gestures that was built on ROS 2 Jazzy. Detects 21 hand
landmarks per frame with MediaPipe, classifies a three gesture vocabulary with
a stability filtered state machine, and drives the system cursor through
PyAutoGUI.

This is a course robotics project. The webcam is the sensor, hand tracking is
the computer vision layer, and ROS 2 is the middleware tying it together.

<!---  video link  --->

## Features

  Real time index fingertip cursor control with adaptive smoothing (One Euro Filter)
  Three gesture vocabulary that all share the same pointing posture, so the cursor never jumps when switching gestures
  Left click, right click, and **click and drag** via held peace sign
  Drag threshold that disambiguates a quick tap (click) from a sustained hold while moving (drag)
  Decoupled ROS 2 architecture: one publisher node, multiple independent subscriber nodes
  Optional stretch goal: a 2 DOF planar arm tracking the hand in RViz via closed form inverse kinematics

## Gesture reference

**Point**: Thumb + index extended, others curled: Move cursor only
**Peace sign**: Index + middle extended, ring + pinky curled: Left click (quick tap) / click and drag (hold while moving)
**Open hand**: All four fingers extended: Right click

A small state machine requires each gesture to hold steady for a few frames
before it counts, which suppresses misfires such as the brief "peace" shape
that flashes while the hand opens for a right click.

## Architecture

The publisher knows nothing about its consumers. Two independent subscribers
attach to the same `/hand_position` topic where one maps it to screen pixels and the
other to joint angles. These demonstrate ROS 2's pub/sub decoupling.

## Tech stack

  **OS:** Ubuntu 24.04 LTS (developed in VMware Workstation 17 on Windows 11)
  **Middleware:** ROS 2 Jazzy Jalisco with `rclpy`
  **Hand tracking:** MediaPipe Hands 0.10.x — 21 landmarks per frame, CPU only
  **Camera I/O:** OpenCV 4.8.x
  **Cursor control:** PyAutoGUI 0.9.54
  **Smoothing:** One Euro Filter (Casiez et al., 2012)
  **Message types:** `geometry_msgs/Point`, `std_msgs/Bool`, `sensor_msgs/JointState`

## Installation

Tested on Ubuntu 24.04 with ROS 2 Jazzy installed at `/opt/ros/jazzy`.

```bash
# System packages
sudo apt update
sudo apt install ros jazzy desktop python3 colcon common extensions \
                 ros jazzy robot state publisher ros jazzy rviz2 git

# Python packages
pip install mediapipe opencv python==4.8.1.78 pyautogui  break system packages
```

Then clone the package into a colcon workspace and build:

```bash
mkdir  p ~/gesture_ws
cd ~/gesture_ws
git clone https://github.com/<username>/gesture_mouse.git
source /opt/ros/jazzy/setup.bash
colcon build  packages select gesture_mouse
source install/setup.bash
```

## Usage

PyAutoGUI needs X11 access to move the cursor:

```bash
export DISPLAY=:0
xhost +local:
```

### Primary: gesture controlled OS mouse

```bash
ros2 launch gesture_mouse gesture_mouse.launch.py
```

A preview window appears showing the camera feed, the hand skeleton overlay,
and the current gesture label. Point your index finger to move the cursor,
peace sign to click or drag, open hand to right click.

### Stretch: 2 DOF arm visualization in RViz

```bash
ros2 launch gesture_mouse arm_demo.launch.py
```

When RViz opens: set Fixed Frame to `world`, add a RobotModel display, set its
Description Topic to `/robot_description`. The arm will then track your hand
through closed form 2 link inverse kinematics.

### Inspecting topics

```bash
ros2 topic list
ros2 topic echo /hand_position
ros2 topic echo /hand_left_button
ros2 topic echo /hand_right_click
ros2 topic hz   /hand_position
```

## Tuning

All parameters live as constants near the top of the relevant source files.

**`gesture_mouse/gesture_publisher.py`**


`MIN_CUTOFF`: Lower = smoother (less jitter) at rest, slightly more lag
`BETA`: Higher = snappier (less lag) during fast motion
`SENSITIVITY`: Fraction of camera FOV used as active region; smaller = finger sweeps map to a larger screen range
`EXTENSION_FACTOR`: How extended a finger must be to register as "up"
`GESTURE_STABILITY_FRAMES`: Frames a gesture must hold before it confirms; raise to reduce misfires, lower for snappier response
`SHOW_PREVIEW`: Toggle the camera preview window

**`gesture_mouse/mouse_controller.py`**


`DRAG_THRESHOLD`: Pixels the hand must move while the button is held before a press becomes a drag (lower = drags start sooner; higher = more click friendly)

## Repository layout

```
gesture_mouse/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/gesture_mouse
├── gesture_mouse/
│   ├── __init__.py
    # Camera  > hand tracking  > topics

│   ├── gesture_publisher.py     
│   ├── mouse_controller.py      
    # Topics  > OS cursor + click/drag

│   └── arm_controller.py        
    # Topics  > 2 DOF inverse kinematics
├── launch/
│   ├── gesture_mouse.launch.py  # Primary deliverable
│   └── arm_demo.launch.py       # Stretch goal
└── urdf/
    └── two_dof_arm.urdf         # Planar 2 link arm for RViz
```

## Known limitations

  **VMware frame rate cap.** Hand tracking runs at ~11 FPS inside VMware Workstation due to virtualization overhead. On bare hardware MediaPipe comfortably exceeds 30 FPS.
  **Gazebo Classic on 24.04.** The proposal originally listed Gazebo Classic 11, but Classic is EOL and doesn't install cleanly on Ubuntu 24.04 / ROS 2 Jazzy. The stretch goal uses RViz instead, which satisfies the "visualize gesture data commanding a virtual robot arm" requirement.
  **Exclusive webcam access.** While the VM is using the webcam, the Windows host can't open it. Recording a demo requires capturing the VM's display from the host (e.g. OBS Display Capture).
  **CANNOT SCROLL** Must use PgDn PgUp buttons on keyboard or alternative to scrolling.

## Design evolution

The click model went through three iterations:

1. **Pinch (thumb + index):** dragged the cursor sideways as the thumb came in to meet the index tip, so clicks landed off target.
2. **Peace sign (edge triggered):** kept the index finger stationary during the click; fixed the off target problem, but supported only discrete clicks.
3. **Peace sign (held state) + drag threshold:** the left button is now driven as a held state rather than a one frame pulse, so a quick tap reads as a click and a sustained hold while moving reads as a drag. A small dead zone around the press point absorbs residual jitter so taps don't accidentally register as micro drags.

Smoothing was upgraded from a fixed rate exponential moving average to a One
Euro Filter, which adapts: heavy smoothing when the hand is still (kills
jitter at rest) and light smoothing during fast motion (no lag while
sweeping).
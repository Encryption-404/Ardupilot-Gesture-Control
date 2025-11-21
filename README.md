# Ardupilot-Gesture-Control
This code integrates DroneKit flight control with a real-time Mediapipe hand-tracking system, allowing an ArduPilot drones to be flown using gesture-based RC overrides or keyboard fallback.

1. Drone and communication libraries

The code uses DroneKit, which provides the Python API to connect to ArduPilot (SITL or a real drone), change flight modes, send RC override commands, arm the vehicle, and perform takeoff/landing. DroneKit depends on pymavlink, which handles MAVLink protocol communication in the background. These two libraries are essential for all drone-related operations in the script.

2. Computer vision and webcam handling

For video capture and image frames, the code uses OpenCV (cv2). This library accesses the webcam, resizes frames, draws UI overlays (such as dead-bands), and displays the hand-tracking video window. Without OpenCV, no camera input or real-time visualization would be possible.

3. Hand-tracking AI models

Hand detection and landmark tracking are powered by MediaPipe, specifically the mediapipe.solutions.hands model. This library performs palm detection, wrist-point extraction, and landmark estimation in real time. MediaPipe runs lightweight machine-learning models optimized for fast inference on consumer hardware, enabling gesture control without needing a GPU.

4. System, math, and threading utilities

Standard Python libraries like threading, time, queue, and math support the program’s internal operations. Threading enables the hand-tracking loop to run in parallel with the drone-control loop, ensuring minimal latency. The msvcrt library is required on Windows for non-blocking keyboard input handling, allowing simultaneous keyboard + gesture control.

5. Installation summary

To run this code successfully, the required install commands are:

pip install dronekit pymavlink opencv-python mediapipe


DroneKit and pymavlink enable MAVLink communication with ArduPilot, OpenCV handles the camera and UI overlay, and MediaPipe provides advanced hand-tracking. These libraries work together to create a full gesture-controlled drone interface.

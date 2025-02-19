# Gesture-Based Control in Assistive Technology

🚀 **Control DexHand in ROS2 Simulation & TurtleBot using Gestures** 🚀  

This project enables **gesture-based control** for assistive technology using **Google's Mediapipe Library** to process hand gestures from a webcam. The recognized gestures are mapped to control **DexHand in ROS2 simulation** and **a TurtleBot** for intuitive robotic interactions.

---

## 📌 Features
- 🖐️ **Hand Gesture Recognition**: Uses **Mediapipe** for real-time detection.
- 🤖 **DexHand Control in ROS2**: Simulated robotic hand manipulation.
- 🏎️ **TurtleBot Navigation**: Move a TurtleBot based on hand gestures.
- 🎥 **Real-Time Webcam Processing**: Uses a webcam feed for dynamic control.

---

## Prerequsites
- Ubuntu 22.04.5 (Jammy Jellyfish) LTS 
- ROS2 Humble - https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
- Dexhand Rviz Package:
  You need the Dexhand Package for the Rviz Visualiztion. Follow the steps from the below repository to install the package in your system.
  https://github.com/iotdesignshop/dexhand_ros2_meta
- Google Mediapipe Library. Which can be Installed by running the following command:
```bash
pip install mediapipe
```
- If you dont have ```pip``` installed in your system , run the following command below:
```bash
sudo apt install python3-pip
```

## 🛠 Installation for Hand Gesture Control

### Create a Workspace wiht a ```src``` folder in it
```bash
mkdir -p ~/ handgestureWS/src
```
### Place the ```handgestures``` package in ```src``` folder

###  Set Up ROS2 Workspace
bash
```
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```
🎮 Usage
In one terminal, Run Gesture Recognition
```bash
ros2 run handgestures handcontroller
```
In other terminal, Run Control DexHand
```bash
ros2 launch dexhand_gesture_controller simulation.launch.py
```

#🧠 How It Works
- Mediapipe Hand Tracking: Captures landmarks of the user's hand.
- Gesture Classification: Recognizes specific gestures (e.g., fist, open palm).
- Command Mapping: Converts gestures into ROS2 messages.
- ROS2 Communication: Sends the commands to DexHand and TurtleBot.
📸 Demo


#🛠 Future Improvements
- ✅ Support for custom gestures
- ✅ Add more intuitive hand signals for control
- ✅ Improve gesture recognition accuracy

# Acknowledgement

This project utilizes the Rviz package from the following GitHub repository:
https://github.com/iotdesignshop/dexhand_ros2_meta

We acknowledge and appreciate the contributions of the developers and maintainers of this repository, which have been instrumental in the development of our project

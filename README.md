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

## 🛠 Installation

### 1️⃣ Install Dependencies
```bash
sudo apt update && sudo apt install -y ros-humble-desktop python3-opencv
pip install mediapipe opencv-python numpy
```

###2️⃣ Clone the Repository
```bash
git clone https://github.com/your-username/Gesture-Based-Control-AT.git
cd Gesture-Based-Control-AT
```
###3️⃣ Set Up ROS2 Workspace
bash
```
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```
🎮 Usage
Run Gesture Recognition
```bash
python3 scripts/gesture_recognition.py
```
Control DexHand
```bash
ros2 launch dexhand_simulation dexhand_control.launch.py
```
Control TurtleBot
```bash
ros2 launch turtlebot3_gesture_control turtlebot_control.launch.py
```

##🧠 How It Works
- Mediapipe Hand Tracking: Captures landmarks of the user's hand.
- Gesture Classification: Recognizes specific gestures (e.g., fist, open palm).
- Command Mapping: Converts gestures into ROS2 messages.
- ROS2 Communication: Sends the commands to DexHand and TurtleBot.
📸 Demo


##🛠 Future Improvements
- ✅ Support for custom gestures
- ✅ Add more intuitive hand signals for control
- ✅ Improve gesture recognition accuracy

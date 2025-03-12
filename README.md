# QArm Control Interface

This repository contains a **Graphical User Interface (GUI) for controlling the Quanser QArm** using Python and Tkinter. It also integrates **RealSense camera feeds** for real-time visualization.

## 📌 Features

- **Control QArm joints** via sliders or predefined positions.
- **Real-time camera feed** from a RealSense camera (RGB & Depth).
- **Forward Kinematics Visualization** for QArm movement.
- **Gripper Control** for picking and placing objects.
- **Emergency Stop** to quickly halt QArm movement.

## 🛠️ Setup & Installation

### **1. Clone the Repository**
```sh
git clone https://github.com/your-username/qarm-control.git
cd qarm-control
```

### **2. Install Required Libraries**
Ensure you have Python 3.7+ installed. Install the required dependencies using:
```sh
pip install numpy opencv-python pillow
```
### **3. Install the Quanser Python SDK
The Quanser Python API (Quanser SDK) is required for communication with the QArm.
Installation:
```sh
pip install quanser
```
Official Documentation: 
- [Quanser QLabs SDK - Installation Guide](https://qlabs.quanserdocs.com/en/latest/Get%20Started.html)

### 4. Run the Application
```sh
python qarm_gui.py
```
## 🎮 Usage
- Adjust joint angles using the sliders.
- Use predefined positions like "Home", "Pick", and "Place".
- View live camera feed (RGB and Depth).
- Open/Close the gripper using the provided buttons.
- Use Emergency Stop in case of unintended movement.

## 🖼️ GUI Preview
![image](https://github.com/user-attachments/assets/10bcbc06-62e2-43cc-872a-b8543f0c8379)

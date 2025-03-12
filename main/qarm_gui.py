import tkinter as tk
from tkinter import ttk
from pal.products.qarm import QArm
import numpy as np
from PIL import Image, ImageTk  # For loading and displaying the logo
import threading
import cv2
from pal.utilities.vision import Camera3D

# Initialize RealSense camera
hardware = 1  # Set to 1 for real hardware, 0 for simulation
id = '0' if hardware else '0@tcpip://localhost:18901'
myCam1 = Camera3D(mode='RGB&DEPTH', frameWidthRGB=640, frameHeightRGB=480,
                   frameWidthDepth=640, frameHeightDepth=480, deviceId=id, readMode=0)



# Function to update camera feed in Tkinter
def update_camera_feed():
    while True:
        try:
            # Read RGB image
            myCam1.read_RGB()
            rgb_frame = cv2.cvtColor(myCam1.imageBufferRGB, cv2.COLOR_BGR2RGB)
            rgb_image = Image.fromarray(rgb_frame)
            rgb_photo = ImageTk.PhotoImage(rgb_image)

            # Read Depth image
            if hardware:
                myCam1.read_depth(dataMode='M')
            else:
                myCam1.read_depth(dataMode='PX')

            depth_frame = myCam1.imageBufferDepthM if hardware else myCam1.imageBufferDepthPX
            depth_colored = cv2.applyColorMap(cv2.convertScaleAbs(depth_frame, alpha=255.0/np.max(depth_frame)), cv2.COLORMAP_JET)
            depth_image = Image.fromarray(depth_colored)
            #depth_photo = ImageTk.PhotoImage(depth_image)

            # Update canvas images
            canvas_rgb.create_image(0, 0, anchor=tk.NW, image=rgb_photo)
            #canvas_depth.create_image(0, 0, anchor=tk.NW, image=depth_photo)

            # Keep reference to avoid garbage collection
            canvas_rgb.image = rgb_photo
           # canvas_depth.image = depth_photo

        except Exception as e:
            print(f"Error updating camera feed: {e}")


# Initialize QArm
myArm = QArm(hardware=1)

# Joint limits (in radians) for QArm
JOINT_LIMITS = {
    0: (-1.57, 1.57),  # Joint 1: -90° to +90°
    1: (-0.52, 0.79),  # Joint 2: -30° to +45°
    2: (-1.05, 1.05),  # Joint 3: -60° to +60°
    3: (-1.57, 1.57),  # Joint 4: -90° to +90°
}

# Predefined positions (joint angles in radians and gripper position)
PREDEFINED_POSITIONS = {
    "home": [0.0, 0.0, 0.0, 0.0, 0.0],  # All joints at zero, gripper open
    "pick": [0.4, 0.64, 0.35, 0.0, 1.0],  # Example pick position
    "place": [-0.5, 0.6, -0.15, -0.2, 0.0],  # Example place position
}

# Forward Kinematics Class
class QArmKinematics:
    """Forward kinematics for the QArm."""

    # QArm parameters (in meters)
    L1 = 0.1400  # Base to shoulder
    L2 = 0.3500  # Shoulder to elbow
    L3 = 0.0500  # Elbow to wrist offset
    L4 = 0.2500  # Wrist to end-effector
    L5 = 0.1500  # Gripper length

    def __init__(self):
        self.BETA = np.arctan2(self.L3, self.L2)  # Angle for the elbow offset

    def forward_kinematics(self, phi):
        """Calculate the position of each joint and the end-effector."""
        theta = np.zeros(4)
        theta[0] = phi[0]  # Base joint
        theta[1] = phi[1] + self.BETA - np.pi / 2  # Shoulder joint
        theta[2] = phi[2] - self.BETA  # Elbow joint
        theta[3] = phi[3]  # Wrist joint

        # Transformation matrices
        T01 = self.dh_matrix(0, -np.pi / 2, self.L1, theta[0])
        T12 = self.dh_matrix(self.L2, 0, 0, theta[1])
        T23 = self.dh_matrix(0, -np.pi / 2, 0, theta[2])
        T34 = self.dh_matrix(0, 0, self.L4 + self.L5, theta[3])

        # Calculate joint positions
        p0 = np.array([0, 0, 0, 1])  # Base position
        p1 = T01 @ np.array([0, 0, 0, 1])
        p2 = T01 @ T12 @ np.array([0, 0, 0, 1])
        p3 = T01 @ T12 @ T23 @ np.array([0, 0, 0, 1])
        p4 = T01 @ T12 @ T23 @ T34 @ np.array([0, 0, 0, 1])  # End-effector position

        return np.array([p0, p1, p2, p3, p4])

    def dh_matrix(self, a, alpha, d, theta):
        """Create a Denavit-Hartenberg transformation matrix."""
        return np.array([
            [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
            [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])

# Function to check if joint angles are within limits
def check_joint_limits(angles):
    for i, angle in enumerate(angles):
        lower, upper = JOINT_LIMITS[i]
        if not (lower <= angle <= upper):
            return f"Joint {i+1} angle {angle:.2f} is out of limits! Must be between {lower:.2f} and {upper:.2f}."
    return None

# Function to update the QArm's position
def update_qarm(angles, gripper):
    error = check_joint_limits(angles)
    if error:
        error_label.config(text=error, foreground="red")
    else:
        error_label.config(text="", foreground="black")
        myArm.read_write_std(phiCMD=angles, grpCMD=gripper, baseLED=np.array([0, 1, 1], dtype=np.float64))
        update_visualization(angles, gripper)
        update_joint_labels(angles)

# Function to update the visualization
def update_visualization(angles, gripper):
    canvas.delete("all")
    
    # Calculate joint positions using forward kinematics
    kinematics = QArmKinematics()
    positions = kinematics.forward_kinematics(angles)
    
    # Scale positions for visualization (adjust as needed)
    scale = 200  # Pixels per meter
    offset_x, offset_y = 200, 400  # Canvas offset

    # Draw the base
    canvas.create_rectangle(offset_x - 20, offset_y, offset_x + 20, offset_y + 20, fill="gray")

    # Draw each joint and segment
    for i in range(1, len(positions)):
        x1 = positions[i - 1][0] * scale + offset_x
        y1 = -positions[i - 1][2] * scale + offset_y  # Flip Y-axis for visualization
        x2 = positions[i][0] * scale + offset_x
        y2 = -positions[i][2] * scale + offset_y

        # Draw the segment
        canvas.create_line(x1, y1, x2, y2, width=5, fill="blue")

        # Draw the joint
        canvas.create_oval(x2 - 5, y2 - 5, x2 + 5, y2 + 5, fill="black")

    # Draw the gripper
    gripper_color = "red" if gripper > 0.5 else "green"
    canvas.create_oval(x2 - 10, y2 - 10, x2 + 10, y2 + 10, fill=gripper_color)

# Function to update joint angle labels
def update_joint_labels(angles):
    joint1_label.config(text=f"Joint 1: {angles[0]:.2f} rad")
    joint2_label.config(text=f"Joint 2: {angles[1]:.2f} rad")
    joint3_label.config(text=f"Joint 3: {angles[2]:.2f} rad")
    joint4_label.config(text=f"Joint 4: {angles[3]:.2f} rad")

# Function to handle slider changes (real-time updates)
def on_slider_change(event=None):
    angles = [joint1_slider.get(), joint2_slider.get(), joint3_slider.get(), joint4_slider.get()]
    gripper = gripper_var.get()
    update_qarm(angles, gripper)

# Function to handle predefined positions
def move_to_predefined(position):
    angles = PREDEFINED_POSITIONS[position][0:4]
    gripper = PREDEFINED_POSITIONS[position][4]
    joint1_slider.set(angles[0])
    joint2_slider.set(angles[1])
    joint3_slider.set(angles[2])
    joint4_slider.set(angles[3])
    gripper_var.set(gripper)
    update_qarm(angles, gripper)

# Function to reset to home position
def reset_to_home():
    move_to_predefined("home")

# Function for emergency stop
def emergency_stop():
    """Stop the QArm by setting all joint commands to zero and closing the gripper."""
    angles = [0.0, 0.0, 0.0, 0.0]  # Reset all joints to zero
    gripper = 0.0  # Open the gripper
    update_qarm(angles, gripper)
    error_label.config(text="Emergency Stop Activated!", foreground="red")

# Create the main window
root = tk.Tk()
root.title("QArm Control Interface")
root.geometry("800x600")
root.configure(bg="#f0f0f0")

# Load the logo
try:
    logo_image = Image.open("kingston_logo.png")
 # Replace with the path to your logo
    logo_image = logo_image.resize((120, 120), Image.Resampling.LANCZOS)  # Resize the logo
    logo_photo = ImageTk.PhotoImage(logo_image)
    logo_label = tk.Label(root, image=logo_photo, bg="#f0f0f0")
    logo_label.grid(row=3, column=1, padx=10, pady=10, sticky="ne")
except Exception as e:
    print(f"Error loading logo: {e}")

# Joint Control Section
joint_frame = ttk.LabelFrame(root, text="Joint Control", padding=10)
joint_frame.grid(row=3, column=0, padx=10, pady=10, sticky="nsew")

joint1_slider = ttk.Scale(joint_frame, from_=-1.57, to=1.57, orient="horizontal", length=200)
joint1_slider.set(0.0)
joint1_slider.bind("<B1-Motion>", on_slider_change)  # Update while dragging
joint1_slider.bind("<ButtonRelease-1>", on_slider_change)  # Update on release
joint1_slider.grid(row=0, column=0, padx=5, pady=5)
joint1_label = ttk.Label(joint_frame, text="Joint 1: 0.00 rad")
joint1_label.grid(row=1, column=0)

joint2_slider = ttk.Scale(joint_frame, from_=-0.52, to=0.79, orient="horizontal", length=200)
joint2_slider.set(0.0)
joint2_slider.bind("<B1-Motion>", on_slider_change)  # Update while dragging
joint2_slider.bind("<ButtonRelease-1>", on_slider_change)  # Update on release
joint2_slider.grid(row=0, column=1, padx=5, pady=5)
joint2_label = ttk.Label(joint_frame, text="Joint 2: 0.00 rad")
joint2_label.grid(row=1, column=1)

joint3_slider = ttk.Scale(joint_frame, from_=-1.05, to=1.05, orient="horizontal", length=200)
joint3_slider.set(0.0)
joint3_slider.bind("<B1-Motion>", on_slider_change)  # Update while dragging
joint3_slider.bind("<ButtonRelease-1>", on_slider_change)  # Update on release
joint3_slider.grid(row=0, column=2, padx=5, pady=5)
joint3_label = ttk.Label(joint_frame, text="Joint 3: 0.00 rad")
joint3_label.grid(row=1, column=2)

joint4_slider = ttk.Scale(joint_frame, from_=-1.57, to=1.57, orient="horizontal", length=200)
joint4_slider.set(0.0)
joint4_slider.bind("<B1-Motion>", on_slider_change)  # Update while dragging
joint4_slider.bind("<ButtonRelease-1>", on_slider_change)  # Update on release
joint4_slider.grid(row=0, column=3, padx=5, pady=5)
joint4_label = ttk.Label(joint_frame, text="Joint 4: 0.00 rad")
joint4_label.grid(row=1, column=3)

# Gripper Control Section
gripper_frame = ttk.LabelFrame(root, text="Gripper Control", padding=10)
gripper_frame.grid(row=4, column=0, padx=10, pady=10, sticky="nsew")

gripper_var = tk.DoubleVar(value=0.0)

def open_gripper():
    gripper_var.set(0.0)
    angles = [joint1_slider.get(), joint2_slider.get(), joint3_slider.get(), joint4_slider.get()]
    update_qarm(angles, gripper_var.get())

def close_gripper():
    gripper_var.set(1.0)
    angles = [joint1_slider.get(), joint2_slider.get(), joint3_slider.get(), joint4_slider.get()]
    update_qarm(angles, gripper_var.get())

ttk.Button(gripper_frame, text="Open Gripper", command=open_gripper).grid(row=1, column=0, padx=5, pady=5)
ttk.Button(gripper_frame, text="Close Gripper", command=close_gripper).grid(row=1, column=1, padx=5, pady=5)

# Create a frame for camera views
camera_frame = ttk.LabelFrame(root, text="Camera Views", padding=6)
camera_frame.grid(row=0, column=0, rowspan=2, padx=6, pady=6, sticky="nsew")

# Create canvases for RGB and Depth images
canvas_rgb = tk.Canvas(camera_frame, width=600, height=400, bg="black")
canvas_rgb.grid(row=1, column=1, padx=5, pady=5)
#canvas_depth = tk.Canvas(camera_frame, width=200, height=500, bg="black")
#canvas_depth.grid(row=0, column=5, padx=5, pady=5)

# Predefined Positions Section
positions_frame = ttk.LabelFrame(root, text="Predefined Positions", padding=10)
positions_frame.grid(row=2, column=0, padx=10, pady=10, sticky="nsew")

ttk.Button(positions_frame, text="Home", command=reset_to_home).grid(row=0, column=0, padx=5, pady=5)
ttk.Button(positions_frame, text="Pick", command=lambda: move_to_predefined("pick")).grid(row=0, column=1, padx=5, pady=5)
ttk.Button(positions_frame, text="Place", command=lambda: move_to_predefined("place")).grid(row=0, column=2, padx=5, pady=5)

# Emergency Stop Button
#emergency_stop_button = ttk.Button(root, text="Emergency_Stop", command=emergency_stop)
#emergency_stop_button.grid(row=3, column=1, columnspan=2, padx=5, pady=5)

# Visualization Section
visualization_frame = ttk.LabelFrame(root, text="Visualization", padding=7)
visualization_frame.grid(row=0, column=1, rowspan=3, padx=5, pady=5, sticky="nsew")

canvas = tk.Canvas(visualization_frame, width=400, height=400, bg="white")
canvas.pack()

# Error Message Section
error_label = tk.Label(root, text="", fg="red", bg="#f0f0f0")
error_label.grid(row=3, column=0, columnspan=2, padx=10, pady=10)

# Run camera feed in a separate thread
camera_thread = threading.Thread(target=update_camera_feed, daemon=True)
camera_thread.start()

# Start the GUI
root.mainloop()

# Terminate the QArm when the GUI is closed
myArm.terminate()
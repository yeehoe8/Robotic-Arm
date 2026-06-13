# 6-DOF Robotic Arm Master Controller

## 🗂️ Project Structure

* `server.py`: The Python FastAPI backend that handles the heavy kinematics math and Bluetooth serial communication.
* `index.html`: The 3D interactive frontend dashboard (runs entirely in your browser).
* `arduino_firmware.ino`: The C++ script to be uploaded to the Arduino Nano driving the PCA9685 servo driver.

---

## ⚙️ Prerequisites & Installation

To run this system, you must set up your Python environment correctly.

**1. Install Python**
Ensure you have **Python 3.11.9** installed on your system. You can download it from the official Python website. During installation, make sure to check the box that says **"Add Python to PATH"**.

**2. Visual Studio Code Setup**
This project relies on several external Python libraries to handle the web server, complex math, and serial communication. If you try to run the code without these, you will get a `ModuleNotFoundError`.

1. Open this project folder in **Visual Studio Code (VSC)**.
2. Open a new terminal in VSC (`Terminal` > `New Terminal`).
3. Copy and paste the following command to install all required dependencies:

```bash
pip install fastapi uvicorn numpy scipy pyserial

```

*(Note: It is crucial to install `pyserial`, not just `serial`)*

---

## 🚀 How to Run the System

### Step 1: Start the Robot Brain (Backend)

The Python server must be running in the background to handle the math and talk to the Arduino.

1. In your VSC terminal, run the server script:

```bash
python server.py

```

2. Wait until you see the message: `Uvicorn running on http://127.0.0.1:8000`. **Leave this terminal open and running.**

### Step 2: Open the Dashboard (Frontend)

Because the Python backend handles the networking, the frontend does not require a web server to run.

1. Open your computer's **File Explorer** (or Finder on Mac).
2. Navigate to the folder where you saved this project.
3. Simply **double-click `index.html**` to open it in your preferred web browser (Chrome, Edge, or Firefox).

### Step 3: Connect the Hardware

1. Ensure your Arduino is powered and the HC-05 Bluetooth module is paired to your computer.
2. Look at the **Control Panel** on the left side of the web dashboard.
3. Enter your specific Bluetooth COM Port (e.g., `COM4` or `COM10`).
4. Click **Connect**. The indicator dot will turn green, and you are ready to command the arm!

---

## ⚠️ Troubleshooting

* **Arm moves up instead of forward?** The Inverse Kinematics solver prioritizes a strict vertical suction cup orientation. If you command a coordinate that is physically out of reach, the "Best Effort" cascade will drop the vertical lock to maximize distance. Ensure your targets are within the physical boundary of the arm.
* **"Port exists, but connection failed"**: Ensure no other programs (like the Arduino IDE Serial Monitor) are currently using the COM port.
* **Joint 2 stalls or hums**: The Arduino firmware uses staggered PWM phase-shifting to prevent current surges, but ensure your bench power supply is providing adequate amperage for the shoulder joint under load.

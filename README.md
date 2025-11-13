⚙️ Hardware Setup
Microcontroller: Arduino Uno

Actuators: 2x Servos (e.g., SG90, MG90S)

Servo B (Base): Connected to Arduino Pin 9

Servo A (Elbow): Connected to Arduino Pin 10

Power: External 5V supply for servos (with a shared GND to the Arduino)

Vision: USB Webcam (connected to the computer)

Marker: A bright red dot on the arm's end-effector.

Wiring
Servo A (Elbow) Signal -> Pin 10

Servo B (Base) Signal -> Pin 9

Servo A + Servo B (Power/VCC) -> External 5V+

Servo A + Servo B (Ground) -> External 5V-

Arduino (GND) -> External 5V- (This is crucial to share the ground!)

💻 Software & Dependencies
This project requires both the Arduino IDE and Python.

Arduino:

[suspicious link removed]

Servo.h library (built-in)

Python 3:

Install the required libraries using the requirements.txt file:

pip install -r requirements.txt

🚀 How to Run
Clone this Repository: git clone https://github.com/2-DOF-Visual-Servoing-Arm.git

Upload Arduino Code:

Open arduino_controller/arduino_controller.ino in the Arduino IDE.

Select your Board (Arduino Uno) and Port.

Upload the sketch to your Arduino.

Run the Python Brain:

IMPORTANT: Edit the visual_servo.py script and update these two lines at the top:

ARDUINO_PORT = '/dev/ttyACM0' (Change this to your Arduino's port. On Windows, it's COM3 or similar. On Ubuntu, it's often /dev/ttyACM0 or /dev/ttyUSB0.)

CAM_INDEX = 2 (Change this to your camera's index.)

Run the main script from your terminal:

cd python_brain

python visual_servo.py

Interact:

The camera window will open.

The arm will move to its "home" position (90°, 90°).

Press 's' (for 'start') to begin the visual servoing. The arm will try to move the red dot to the green circle.

Press 'r' to force the arm to re-calculate its motion (re-estimate the Jacobian). This is useful if it gets stuck.

Press 'q' to quit.

🧠 How It Works (The Concept)
This system uses an Image-Based Visual Servoing (IBVS) control loop.

Find Dot: OpenCV finds the (x, y) pixel coordinates of the red dot.

Calculate Error: It calculates the distance (error_x, error_y) between the dot and the center of the screen (the target).

Estimate Jacobian (The "Wiggle"): The arm doesn't know how its joint angles (θ1, θ2) relate to the camera's pixels (x, y). To figure this out, it performs a quick "test" (this happens when you press 'r'):

It wiggles the Base servo by +1° and measures how many pixels the dot moves (dx_dth1, dy_dth1).

It wiggles the Elbow servo by +1° and measures how many pixels the dot moves (dx_dth2, dy_dth2).

It builds a 2x2 "Jacobian" matrix, J, which is the "magic translator" between (angles) and (pixels).

Solve & Act: The script calculates the inverse of this matrix (J_inv). It then multiplies the J_inv by the pixel error to find the exact change in angles (d_th1, d_th2) needed to correct the error.

Repeat: The arm moves, the dot gets closer, and the loop repeats, making smaller and smaller corrections until the error is zero.
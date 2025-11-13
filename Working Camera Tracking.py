import cv2
import numpy as np
import serial
import time
import sys

# --- 1. CONFIGURATION ---

# CRITICAL: Change this to your Arduino's port!
ARDUINO_PORT = '/dev/ttyACM0' 

CAM_INDEX = 2            # Your camera index
BAUD_RATE = 9600
WINDOW_NAME = "Visual Servoing (Press 'q' to quit)" # Window name

# Control Parameters
GAIN = 0.1               # How fast/aggressively the arm moves. Start small.

# === CHANGED THIS ===
# Increased wiggle to get a clear, non-zero pixel change
WIGGLE_AMOUNT = 8.0      # (Degrees) How much to "wiggle" servos to learn

WIGGLE_DELAY = 1.0       # (Seconds) How long to wait for servo to move
ERROR_THRESHOLD = 5      # (Pixels) How close to the center is "good enough"

# HSV Color range for your RED dot
RED_LOWER = np.array([0, 120, 70])
RED_UPPER = np.array([10, 255, 255])
RED_LOWER_2 = np.array([170, 120, 70])
RED_UPPER_2 = np.array([180, 255, 255])

# --- 2. HELPER FUNCTIONS ---

def connect_to_arduino(port, baud):
    """Tries to connect to the Arduino."""
    try:
        arduino = serial.Serial(port=port, baudrate=baud, timeout=0.1)
        time.sleep(2) # Wait for the connection to establish
        print(f"✅ Connected to Arduino on {port}")
        return arduino
    except Exception as e:
        print(f"❌ ERROR: Could not connect to Arduino on {port}.")
        print(f"   Check that the port is correct and the device is plugged in.")
        print(f"   Error details: {e}")
        sys.exit(1) # Exit the script

def connect_to_camera(index):
    """Tries to connect to the camera."""
    cam = cv2.VideoCapture(index)
    if not cam.isOpened():
        print(f"❌ ERROR: Cannot open camera at index {index}.")
        print("   Check that it is plugged in and not in use by another program.")
        sys.exit(1)
        
    ret, frame = cam.read()
    if not ret:
        print(f"❌ ERROR: Cannot read frame from camera at index {index}.")
        cam.release()
        sys.exit(1)
    
    h, w = frame.shape[:2]
    center_x, center_y = w // 2, h // 2
    print(f"✅ Camera feed opened: {w}x{h} (Center: {center_x}, {center_y})")
    return cam, h, w, center_x, center_y

def send_servo_command(arduino, servo_id, angle):
    """Sends a formatted command to the Arduino."""
    angle = int(np.clip(angle, 0, 180)) # Clamp angle between 0 and 180
    command = f"{servo_id}{angle}\n"
    arduino.write(command.encode('utf-8'))
    return angle # Return the clamped angle

def find_dot(frame):
    """Finds the largest red contour in the frame."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    mask1 = cv2.inRange(hsv, RED_LOWER, RED_UPPER)
    mask2 = cv2.inRange(hsv, RED_LOWER_2, RED_UPPER_2)
    mask = mask1 + mask2
    
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        c = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(c)
        
        if area > 100: 
            M = cv2.moments(c)
            if M["m00"] != 0:
                # === CHANGED THIS ===
                # Return FLOAT position for more accuracy
                dot_x = M["m10"] / M["m00"]
                dot_y = M["m01"] / M["m00"]
                return (dot_x, dot_y), area
                
    return None, 0 # No dot found

def read_stable_dot_position(cam, text_to_show=""):
    """
    Reads the dot position and displays it on the window.
    This function is NOW used to show the camera feed
    during calibration.
    """
    positions = []
    
    # Take 5 snapshots
    for _ in range(5):
        ret, frame = cam.read()
        if not ret:
            continue
            
        pos, _ = find_dot(frame)
        if pos:
            positions.append(pos)
        
        # === NEW ===
        # Show the window even if dot not found
        cv2.putText(frame, text_to_show, (20, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        if pos:
            cv2.circle(frame, (int(pos[0]), int(pos[1])), 10, (0, 0, 255), 2)
            
        cv2.imshow(WINDOW_NAME, frame)
        cv2.waitKey(1)
        # === END NEW ===
        
        time.sleep(0.02) # Small delay
        
    if not positions:
        return None
    
    # Calculate median of the FLOAT positions
    median_x = np.median([p[0] for p in positions])
    median_y = np.median([p[1] for p in positions])
    return (median_x, median_y) # Return float

def calibrate_jacobian(arduino, cam, home_base, home_elbow):
    """
    "Wiggles" the servos to learn the Image Jacobian.
    """
    print("\n--- 🤖 Starting 1-Time Calibration ---")
    print("Please do not disturb the robot...")
    
    # 1. Go to home position and get baseline
    send_servo_command(arduino, 'B', home_base)
    send_servo_command(arduino, 'E', home_elbow)
    print("Moving to home position...")
    time.sleep(WIGGLE_DELAY * 2) # Wait 2x to settle
    
    pos_home = read_stable_dot_position(cam, "Reading Home Position...")
    if pos_home is None:
        print("❌ CALIBRATION FAILED: Cannot find red dot at home position.")
        return None, None
    print(f"Home dot position found at: ({pos_home[0]:.2f}, {pos_home[1]:.2f})")

    # 2. Wiggle Base Servo
    print(f"Wiggling Base servo by +{WIGGLE_AMOUNT} degrees...")
    send_servo_command(arduino, 'B', home_base + WIGGLE_AMOUNT)
    time.sleep(WIGGLE_DELAY)
    pos_base_wiggle = read_stable_dot_position(cam, "Wiggling Base...")
    
    if pos_base_wiggle is None:
        print("❌ CALIBRATION FAILED: Lost dot during Base wiggle.")
        return None, None
    
    # Calculate pixel change per degree for Base
    dx_db = (pos_base_wiggle[0] - pos_home[0]) / WIGGLE_AMOUNT
    dy_db = (pos_base_wiggle[1] - pos_home[1]) / WIGGLE_AMOUNT
    
    # Reset base
    send_servo_command(arduino, 'B', home_base)
    time.sleep(WIGGLE_DELAY)

    # 3. Wiggle Elbow Servo
    print(f"Wiggling Elbow servo by +{WIGGLE_AMOUNT} degrees...")
    send_servo_command(arduino, 'E', home_elbow + WIGGLE_AMOUNT)
    time.sleep(WIGGLE_DELAY)
    pos_elbow_wiggle = read_stable_dot_position(cam, "Wiggling Elbow...")
    
    if pos_elbow_wiggle is None:
        print("❌ CALIBRATION FAILED: Lost dot during Elbow wiggle.")
        return None, None

    # Calculate pixel change per degree for Elbow
    dx_de = (pos_elbow_wiggle[0] - pos_home[0]) / WIGGLE_AMOUNT
    dy_de = (pos_elbow_wiggle[1] - pos_home[1]) / WIGGLE_AMOUNT
    
    # Reset elbow
    send_servo_command(arduino, 'E', home_elbow)
    time.sleep(WIGGLE_DELAY)

    # 4. Build the Jacobian Matrix
    jacobian = np.array([
        [dx_db, dx_de],
        [dy_db, dy_de]
    ])
    
    # 5. Invert the Jacobian
    try:
        jacobian_inv = np.linalg.inv(jacobian)
        print("\n--- ✅ Calibration Complete ---")
        print(f"Jacobian (J):\n {jacobian}")
        print(f"Inverse Jacobian (J^-1):\n {jacobian_inv}")
        return jacobian_inv, jacobian
    except np.linalg.LinAlgError:
        print("❌ CALIBRATION FAILED: Jacobian is non-invertible (singularity).")
        print("   This means the wiggle moves were parallel or zero.")
        print("   Try a different home position or larger WIGGLE_AMOUNT.")
        return None, None

# --- 3. MAIN SCRIPT ---

# Connect to hardware
arduino = connect_to_arduino(ARDUINO_PORT, BAUD_RATE)
cam, IMG_H, IMG_W, CENTER_X, CENTER_Y = connect_to_camera(CAM_INDEX)

# === NEW ===
# Create the window at the start
cv2.namedWindow(WINDOW_NAME)

# Set initial servo positions
current_base_angle = 90.0
current_elbow_angle = 90.0
current_base_angle = send_servo_command(arduino, 'B', current_base_angle)
current_elbow_angle = send_servo_command(arduino, 'E', current_elbow_angle)

# Run the 1-time calibration
jacobian_inv, _ = calibrate_jacobian(arduino, cam, current_base_angle, current_elbow_angle)

if jacobian_inv is None:
    print("Exiting due to calibration failure.")
    cam.release()
    cv2.destroyAllWindows()
    arduino.close()
    sys.exit(1)

print("\n--- 🚀 Starting Visual Servoing Loop ---")
print("Press 'q' to quit.")
print("The robot will now try to center the red dot.")

while True:
    # --- A. SEE (Get Frame) ---
    ret, frame = cam.read()
    if not ret:
        print("No frame, skipping loop.")
        continue

    # --- B. FIND (Detect Dot) ---
    dot_pos, dot_area = find_dot(frame) # Returns float pos

    if dot_pos:
        dot_x, dot_y = dot_pos
        
        # --- C. COMPARE (Calculate Error) ---
        error_x = CENTER_X - dot_x
        error_y = CENTER_Y - dot_y
        error_vector = np.array([error_x, error_y])
        
        if np.linalg.norm(error_vector) > ERROR_THRESHOLD:
            # --- D. SOLVE & DECIDE ---
            delta_angles = jacobian_inv @ error_vector * GAIN
            
            delta_base = delta_angles[0]
            delta_elbow = delta_angles[1]

            # --- E. ACT (Update Angles) ---
            current_base_angle += delta_base
            current_elbow_angle += delta_elbow
            
            current_base_angle = send_servo_command(arduino, 'B', current_base_angle)
            current_elbow_angle = send_servo_command(arduino, 'E', current_elbow_angle)

        # --- F. VISUALIZE ---
        # Draw on the frame (use int() for drawing)
        cv2.circle(frame, (int(dot_x), int(dot_y)), 10, (0, 255, 255), 2)
        cv2.circle(frame, (int(dot_x), int(dot_y)), 3, (0, 0, 255), -1)
        cv2.circle(frame, (CENTER_X, CENTER_Y), 10, (0, 255, 0), 2)
        cv2.line(frame, (int(dot_x), int(dot_y)), (CENTER_X, CENTER_Y), (255, 0, 0), 2)
        
    else:
        # No dot found
        cv2.putText(frame, "Dot not found", (20, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    # Show the final frame
    cv2.imshow(WINDOW_NAME, frame) # Use the same window name

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break # Exit loop

# --- 7. CLEANUP ---
print("Shutting down...")
cam.release()
cv2.destroyAllWindows()
send_servo_command(arduino, 'B', 90)
send_servo_command(arduino, 'E', 90)
arduino.close()
print("Done.")
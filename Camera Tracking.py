import cv2
import numpy as np
import serial
import time
import math

# ---------------- CONFIGURATION ----------------
SERIAL_PORT = 'COM3'      # Change to your Arduino port
BAUD = 57600
FRAME_W, FRAME_H = 640, 480

SCALE = 0.000147          # meters per pixel (tune as needed)
l1, l2 = 0.09, 0.08       # link lengths (in meters)

CENTER_TOLERANCE = 25     # pixels around center = deadzone
CX_ALPHA = 0.35           # smoothing for centroid (0..1). Lower -> smoother/slower
SEND_DELAY = 0.10         # seconds between servo updates
COLOR_MODE = 'red'        # 'red' or 'black'

# Movement smoothing / limits (NEW)
MAX_ANGLE_STEP = 2.0      # max degrees change per send (slow motion)
ANGLE_DEADBAND = 1.0      # don't react to changes smaller than this (degrees)
MIN_AREA = 200            # minimal contour area to accept detection
# ------------------------------------------------

# Initialize serial
try:
    ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1)
    time.sleep(2)
    print("Connected to Arduino on", SERIAL_PORT)
except Exception as e:
    print("Serial not connected:", e)
    ser = None

# Open camera (prefer DirectShow on Windows)
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
if not cap.isOpened():
    raise RuntimeError("Cannot open camera")

# Image center
icx, icy = FRAME_W // 2, FRAME_H // 2

# Smoothed centroid (start at center)
sm_cx = float(icx)
sm_cy = float(icy)

# Current servo angles (start at neutral 90)
cur_th1 = 90.0
cur_th2 = 90.0

# last send time
last_send = 0.0

# ---------------- helper functions ----------------
def detect_colored_mask(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    if COLOR_MODE == 'red':
        # two ranges for red hue wrap-around
        lower1 = np.array([0, 100, 100])
        upper1 = np.array([10, 255, 255])
        lower2 = np.array([160, 100, 100])
        upper2 = np.array([179, 255, 255])
        mask = cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)
    elif COLOR_MODE == 'black':
        lower = np.array([0, 0, 0])
        upper = np.array([179, 255, 60])
        mask = cv2.inRange(hsv, lower, upper)
    else:
        mask = np.zeros((frame.shape[0], frame.shape[1]), dtype=np.uint8)
    # morphological cleanup
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask

def compute_ik_angles(xd, yd):
    # compute th1, th2 in degrees clamped 0..180
    xd2 = xd*xd
    yd2 = yd*yd
    denom = 2 * l1 * math.sqrt(xd2 + yd2) if (xd2 + yd2) > 1e-12 else 1e-12
    term1 = (xd2 + yd2 + l1*l1 - l2*l2) / denom
    term1 = max(-1.0, min(1.0, term1))
    th1_rad = -1.0 * math.acos(term1) + math.atan2(yd, xd)
    th2_rad = math.atan2(yd - l1*math.sin(th1_rad), xd - l1*math.cos(th1_rad))
    th2_rad = th2_rad - th1_rad
    th1_deg = th1_rad * 180.0 / math.pi
    th2_deg = th2_rad * 180.0 / math.pi
    if th1_deg < 0: th1_deg += 360.0
    if th2_deg < 0: th2_deg += 360.0
    th1_deg = max(0.0, min(180.0, th1_deg))
    th2_deg = max(0.0, min(180.0, th2_deg))
    return th1_deg, th2_deg

def send_servo_angles(a_deg, b_deg):
    global ser
    if ser is None or not ser.is_open:
        return
    try:
        ser.write(f"A{a_deg:.2f}\n".encode('utf-8'))
        time.sleep(0.01)
        ser.write(f"B{b_deg:.2f}\n".encode('utf-8'))
    except Exception as e:
        print("Serial write error:", e)

# ---------------- main loop ----------------
print("Starting main loop. Press ESC to quit.")
while True:
    ret, frame = cap.read()
    if not ret:
        print("Frame capture failed")
        break

    mask = detect_colored_mask(frame)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cx = None
    cy = None
    if contours:
        c = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(c)
        if area > MIN_AREA:
            M = cv2.moments(c)
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                # draw detection
                cv2.circle(frame, (cx, cy), 4, (0,255,0), -1)
                cv2.drawContours(frame, [c], -1, (0,255,0), 2)
                cv2.putText(frame, f"area:{int(area)}", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0),1)

    # Smooth centroid to remove jitter
    if cx is not None:
        sm_cx = CX_ALPHA * cx + (1.0 - CX_ALPHA) * sm_cx
        sm_cy = CX_ALPHA * cy + (1.0 - CX_ALPHA) * sm_cy
    else:
        # If not detected, slowly drift smoothed coordinates towards center so IK won't jump
        sm_cx = 0.98 * sm_cx + 0.02 * icx
        sm_cy = 0.98 * sm_cy + 0.02 * icy

    # compute pixel offsets from center using smoothed centroid
    dx = sm_cx - icx
    dy = sm_cy - icy

    # show center crosshair and info
    cv2.drawMarker(frame, (icx, icy), (255,255,255), cv2.MARKER_CROSS, 20, 1)
    cv2.putText(frame, f"dx:{dx:.1f} dy:{dy:.1f}", (10,frame.shape[0]-20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255),1)

    # Move only if we have a valid detection AND outside the pixel deadzone
    if cx is not None and (abs(dx) > CENTER_TOLERANCE or abs(dy) > CENTER_TOLERANCE):
        # Convert pixels -> meters (same convention as your original code)
        measured_x = -SCALE * dx
        measured_y = SCALE * dy

        # compute desired target angles from IK
        target_th1, target_th2 = compute_ik_angles(measured_x, measured_y)

        # compute deltas between target and current servo angles
        delta1 = target_th1 - cur_th1
        delta2 = target_th2 - cur_th2

        # apply deadband: if small change, ignore
        if abs(delta1) < ANGLE_DEADBAND:
            delta1 = 0.0
        if abs(delta2) < ANGLE_DEADBAND:
            delta2 = 0.0

        # limit the step to MAX_ANGLE_STEP per send (this enforces slow movement)
        step1 = max(-MAX_ANGLE_STEP, min(MAX_ANGLE_STEP, delta1))
        step2 = max(-MAX_ANGLE_STEP, min(MAX_ANGLE_STEP, delta2))

        # prepare new current angles
        new_th1 = cur_th1 + step1
        new_th2 = cur_th2 + step2

        # send only at rate-limited interval
        now = time.time()
        if now - last_send >= SEND_DELAY and (abs(step1) > 1e-6 or abs(step2) > 1e-6):
            send_servo_angles(new_th1, new_th2)
            last_send = now
            cur_th1 = new_th1
            cur_th2 = new_th2
            cv2.putText(frame, f"Sent A:{cur_th1:.1f} B:{cur_th2:.1f}", (10,60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0),1)
        else:
            cv2.putText(frame, "Waiting or step too small", (10,60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,200,200),1)
    else:
        # No valid detection or within deadzone -> do not move
        cv2.putText(frame, "Centered or not detected - no move", (10,60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255),1)

    # show windows
    cv2.imshow("Camera", frame)
    cv2.imshow("Mask", mask)

    key = cv2.waitKey(1) & 0xFF
    if key == 27:
        break

# cleanup
cap.release()
if ser:
    ser.close()
cv2.destroyAllWindows()

############################################################
from dronekit import connect, VehicleMode
import time, msvcrt

MID = 1500
STEP = 200
TAKEOFF_ALT = 5

def get_key():
    if msvcrt.kbhit():
        return msvcrt.getch().decode("utf-8").lower()
    return None

print("Connecting to SITL...")
vehicle = connect("tcp:127.0.0.1:5762", wait_ready=True)
print("Connected!")

def send_rc(roll, pitch, throttle, yaw):
    overrides = {'1': roll, '2': pitch, '4': yaw}
    if throttle is not None:
        overrides['3'] = throttle
    vehicle.channels.overrides = overrides

def hover():
    vehicle.channels.overrides = {'1': MID, '2': MID, '4': MID}

def clear_all_overrides():
    vehicle.channels.overrides = {}
    time.sleep(0.2)

def takeoff_to(alt):
    print("\n=== TAKEOFF REQUESTED ===")
    clear_all_overrides()
    vehicle.mode = VehicleMode("GUIDED")

    print("Arming motors...")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Performing simple_takeoff...")
    vehicle.simple_takeoff(alt)

    while True:
        current_alt = vehicle.location.global_relative_frame.alt
        print(" Altitude:", round(current_alt, 2))
        if current_alt >= alt * 0.95:
            break
        time.sleep(0.4)

    print("Switching to LOITER...")
    vehicle.mode = VehicleMode("LOITER")
    hover()

def land():
    print("Landing...")
    clear_all_overrides()
    vehicle.mode = VehicleMode("LAND")


############################################################
# HAND TRACKING THREAD (DJI NEO 2 STYLE)
############################################################
import cv2
import mediapipe as mp
import threading
import queue
import math

# Shared control dictionary
hand_control = {
    "active": False,    # True when hands detected
    "roll": 0,
    "pitch": 0,
    "throttle": 0,
    "yaw": 0
}

FRAME_W = 640
FRAME_H = 480
CENTER_X = FRAME_W // 2
CENTER_Y = FRAME_H // 2

NEUTRAL_BAND = 60
SLOW_BAND = 50

cap = cv2.VideoCapture(0)

mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=2,
    model_complexity=0,
    min_detection_confidence=0.65,
    min_tracking_confidence=0.5
)

def normalize(val, maxv):
    return max(min((val / maxv) * 100, 100), -100)

############################################################
# HAND TRACKING LOOP
############################################################
def hand_tracking_loop():
    global hand_control

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        frame = cv2.resize(frame, (FRAME_W, FRAME_H))
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(rgb)

        centers = []

        # detect hands
        if results.multi_hand_landmarks:
            for hl in results.multi_hand_landmarks:
                wrist = hl.landmark[9]
                px = int(wrist.x * FRAME_W)
                py = int(wrist.y * FRAME_H)
                centers.append((px, py))

        # --------------------------------------
        # NO HANDS → disable override
        # --------------------------------------
        if len(centers) == 0:
            hand_control["active"] = False
            cv2.imshow("Gesture Control", frame)
            cv2.waitKey(1)
            continue

        # --------------------------------------
        # 1 HAND MODE  → roll + throttle (unchanged)
        # --------------------------------------
        if len(centers) == 1:
            hx, hy = centers[0]
            dx = hx - CENTER_X
            dy = hy - CENTER_Y

            # GREEN SHADED DEAD BAND SQUARE
            overlay = frame.copy()
            cv2.rectangle(
                overlay,
                (CENTER_X - NEUTRAL_BAND, CENTER_Y - NEUTRAL_BAND),
                (CENTER_X + NEUTRAL_BAND, CENTER_Y + NEUTRAL_BAND),
                (0, 255, 0),
                -1
            )
            frame = cv2.addWeighted(overlay, 0.35, frame, 0.65, 0)

            # controls
            if abs(dx) > NEUTRAL_BAND:
                roll = -normalize(dx, FRAME_W//2)
            else:
                roll = 0

            if abs(dy) > NEUTRAL_BAND:
                throttle = -normalize(dy, FRAME_H//2)
            else:
                throttle = 0

            hand_control = {
                "active": True,
                "roll": roll,
                "pitch": 0,
                "throttle": throttle,
                "yaw": 0
            }

        # --------------------------------------
        # 2 HAND MODE → pitch only (DJI Neo 2)  <-- SIGN FIX APPLIED HERE
        # --------------------------------------
        else:
            (x1, y1), (x2, y2) = centers

            # draw hand-to-hand line
            cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)

            # midpoint
            cx = (x1 + x2) // 2

            # GREEN SHADED VERTICAL DEAD-BAND CORRIDOR
            overlay = frame.copy()
            cv2.rectangle(
                overlay,
                (CENTER_X - NEUTRAL_BAND, 0),
                (CENTER_X + NEUTRAL_BAND, FRAME_H),
                (0, 255, 0),
                -1
            )
            frame = cv2.addWeighted(overlay, 0.35, frame, 0.65, 0)

            # deadband lines (edges)
            cv2.line(frame, (CENTER_X-NEUTRAL_BAND, 0),
                     (CENTER_X-NEUTRAL_BAND, FRAME_H), (0,255,0), 2)
            cv2.line(frame, (CENTER_X+NEUTRAL_BAND, 0),
                     (CENTER_X+NEUTRAL_BAND, FRAME_H), (0,255,0), 2)

            # slow band lines
            cv2.line(frame, (CENTER_X-SLOW_BAND, 0),
                     (CENTER_X-SLOW_BAND, FRAME_H), (0,255,0), 1)
            cv2.line(frame, (CENTER_X+SLOW_BAND, 0),
                     (CENTER_X+SLOW_BAND, FRAME_H), (0,255,0), 1)

            dx = cx - CENTER_X

            # pitch logic (compute as before) then INVERT sign
            if abs(dx) <= NEUTRAL_BAND:
                pitch_val = 0
            elif abs(dx) <= SLOW_BAND:
                pitch_val = normalize(dx/2, SLOW_BAND)
            else:
                pitch_val = normalize(dx, FRAME_W//2)

            # -invert sign so inward (near center) -> forward (pitch < MID) ---
            pitch = -pitch_val

            hand_control = {
                "active": True,
                "roll": 0,
                "pitch": pitch,
                "throttle": 0,
                "yaw": 0
            }

        cv2.imshow("Gesture Control", frame)
        cv2.waitKey(1)

threading.Thread(target=hand_tracking_loop, daemon=True).start()


############################################################
# MAIN DRONE CONTROL LOOP 
############################################################
print("\nLOITER MODE CONTROLS:")
print(" W/S = forward / back")
print(" A/D = left / right")
print(" X/Z = up / down (throttle)")
print(" Q/E = yaw left / right")
print(" K   = TAKEOFF")
print(" L   = LAND")
print(" SPACE = Hover\n")

hover()

try:
    while True:
        # ----------------------------------------
        # HAND CONTROL OVERRIDES KEYBOARD
        # ----------------------------------------
        if hand_control["active"]:
            send_rc(
                MID + int(hand_control["roll"] * 5),
                MID + int(hand_control["pitch"] * 5),
                MID + int(hand_control["throttle"] * 5),
                MID
            )
            time.sleep(0.05)
            continue

        # ----------------------------------------
        # NO HANDS → normal keyboard mode
        # ----------------------------------------
        key = get_key()

        if not key:
            hover()
            clear_all_overrides()
            time.sleep(0.05)
            continue

        roll = MID
        pitch = MID
        yaw = MID
        throttle = None

        if key == "w": pitch = MID - STEP
        elif key == "s": pitch = MID + STEP
        elif key == "a": roll = MID - STEP
        elif key == "d": roll = MID + STEP
        elif key == "q": yaw = MID - STEP
        elif key == "e": yaw = MID + STEP
        elif key == "x": throttle = MID + STEP
        elif key == "z": throttle = MID - STEP

        elif key == "k":
            takeoff_to(TAKEOFF_ALT)
            continue
        elif key == "l":
            land()
            continue
        elif key == " ":
            clear_all_overrides()
            hover()
            continue

        send_rc(roll, pitch, throttle, yaw)
        time.sleep(0.05)

except KeyboardInterrupt:
    print("Stopping...")
    clear_all_overrides()
    vehicle.close()

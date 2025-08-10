"""little_bicycle_P_V2.1 controller."""

import cv2
import numpy as np
import struct
from controller import Supervisor

robot = Supervisor()
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

robotNode = robot.getSelf()

# Link Supervisor-mounted camera to bicycle pose
bicycle_node = robot.getFromDef('BICYCLE')
camera_mount_node = robot.getFromDef('VISION_CAMERA_TRANSFORM')
camera_mount_translation = camera_mount_node.getField('translation') if camera_mount_node else None
camera_mount_rotation = camera_mount_node.getField('rotation') if camera_mount_node else None

def sync_camera_to_bicycle():
    if bicycle_node is None or camera_mount_translation is None or camera_mount_rotation is None:
        return
    try:
        bike_pos = bicycle_node.getPosition()
        bike_rot_field = bicycle_node.getField('rotation')
        bike_rot = bike_rot_field.getSFRotation() if bike_rot_field is not None else [0, 1, 0, 0]
        camera_mount_translation.setSFVec3f(bike_pos)
        camera_mount_rotation.setSFRotation(bike_rot)
    except Exception:
        # Fail-safe: don't crash the control loop if fields are temporarily unavailable
        pass

preview = 1 # Mask Preview 1

Kp = 0.01
Ki = 0.02
Kd = 0.0001

P = 0
I = 0
D = 0
oldP = 0
PID = 0

# Bicycle speed
maxS = 6 # max speed
minS = 3 # min speed

# Handlebar angle
hMax = 0.1920 # rads (11°) Max
hndB = 0 # center
maxV = 0          # Max Velocity

# Initialize camera
camera = robot.getDevice('camera')
camera.enable(timestep*4)

# Initialize display
display = robot.getDevice('display')
display.attachCamera(camera)
display.setColor(0x00FF00)
display.setFont('Verdana', 16, True)

# IPC: Emitter für Vision-Commands (an Balance-Controller)
command_emitter = robot.getDevice('command_tx')

if preview == 1:
    cv2.startWindowThread()

def getError(act_error):
    error_P = act_error

    # get image form camera bicycle
    img = np.frombuffer(camera.getImage(), dtype=np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
    
    # height and width of image
    h = img.shape[0]
    w = img.shape[1]

    # set point in x and y
    xSet = int(w / 2) - 10
    ySet = 205

    mask = np.zeros((h, w), dtype=np.uint8)
    pts = np.array([[[90, 200], [390, 200], [410, 210], [70, 210]]]) # Bicycle
    cv2.fillPoly(mask, pts, 255)
    zone = cv2.bitwise_and(img, img, mask=mask)
    
    hsv = cv2.cvtColor(zone, cv2.COLOR_BGR2HSV)
    
    dark_color = np.array([75, 0, 0])
    bght_color = np.array([179, 255, 255])
    Kernel = np.ones((5, 5), np.uint8)

    mask0 = cv2.inRange(hsv, dark_color, bght_color)
    mask0 = cv2.morphologyEx(mask0, cv2.MORPH_CLOSE, Kernel)
    mask0 = cv2.morphologyEx(mask0, cv2.MORPH_OPEN, Kernel)

    if preview == 1:
        cv2.imshow("preview", mask0)
        cv2.waitKey(1)

    try:
        cnts0, _ = cv2.findContours(mask0, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    except:
        return error_P
    else:
        try:
            largest_contour = max(cnts0, key=cv2.contourArea)
        except:
            return error_P
        else:
            try:
                x,y,w,h = cv2.boundingRect(largest_contour)
                center_x = int(x + w / 2)
                display.setAlpha(0.0)
                display.fillRectangle(0, 0, display.width, display.height)
                display.setAlpha(1.0)
                display.drawLine(center_x - 20, ySet, center_x + 20, ySet)
                display.drawLine(center_x, ySet - 20, center_x, ySet + 20)
                display.fillOval(center_x, ySet, 3, 3)
            except:
                return error_P
            else:
                return xSet - center_x 



# hour:minutes:seconds
def hms(sec):
    h = sec // 3600
    m = sec % 3600 // 60
    s = sec % 3600 % 60
    tm = f'{h:02d}:{m:02d}:{s:02d}'
    return tm


def printStatus():
    global maxV
    vpos = 0.93
    # Get Velocity
    velo = robotNode.getVelocity()
    
    # Velocity calulation:  Speed Module (x, y, z)
    velocity = (velo[0]**2 + velo[1]**2 + velo[2]**2)**0.5
    velocity = velocity * 3.6 # km/h
    
    if velocity > maxV:
        maxV = (velocity + maxV) / 2

    timer = int(robot.getTime())
    strP = hms(timer)
    
    if robot.getName() == 'Little Bicycle 1':
        vpos = 0.93
        strP = f'Time: {strP:s}'
        robot.setLabel(0, strP, 0, 0.97, 0.06, 0x000000, 0, 'Lucida Console')
    elif robot.getName() == 'Little Bicycle 2':
        vpos = 0.89
    strP = f'Robot: {robot.getName():s}   Speed: {velocity:5.2f} km/h   Max {maxV:5.2f} km/h'
    robot.setLabel(1, strP, 0, vpos, 0.06, 0x000000, 0, 'Lucida Console')
        
def send_vision_command(steer_cmd_rad, speed_cmd_units, vision_error_norm=0.0, p_term=0.0, i_term=0.0, d_term=0.0, mask_coverage=0.0):
    """Sendet Vision-Command an Balance-Controller (C) im kompatiblen Struct-Format.
    - steer_cmd_rad: Lenkwinkel in rad ([-hMax, hMax]) → wird auf [-1, 1] normiert
    - speed_cmd_units: Geschwindigkeit in internen Einheiten [minS, maxS] → wird auf [0, 1] normiert
    - vision_error_norm, p_term, i_term, d_term, mask_coverage: optionale Debug-Werte
    """
    try:
        if not command_emitter:
            return False
        # Normalisieren
        steer_cmd = np.clip(steer_cmd_rad / hMax, -1.0, 1.0)
        speed_cmd = 0.0 if maxS <= minS else (speed_cmd_units - minS) / (maxS - minS)
        speed_cmd = float(np.clip(speed_cmd, 0.0, 1.0))

        # Struct: 2 floats, 1 int, 5 floats → 32 Bytes
        command_data = struct.pack('@ffifffff',
                                   float(steer_cmd), float(speed_cmd), 1,
                                   float(vision_error_norm), float(p_term), float(i_term), float(d_term),
                                   float(mask_coverage))

        command_emitter.send(command_data)
        return True
    except Exception as e:
        print(f"Fehler beim Senden des Vision-Commands: {e}")
        return False


# Main loop:
while robot.step(timestep) != -1:

    # Keep camera mounted to the bicycle in world coordinates
    sync_camera_to_bicycle()

    P = getError(P)
    I = I * 2 / 3 + P * timestep / 1000
    D = D * 0.5 + (P - oldP) / timestep * 1000
    
    PID = Kp * P + Ki * I + Kd * D
    oldP = P

    hndB = hMax - abs(PID) # handlebar angle control
    hndB = hndB + PID
    if hndB > hMax: hndB = hMax # max handlebar angle
    elif hndB < -hMax: hndB = -hMax # min handlebar angle
    
    bcyS = maxS # max speed
    bcyS = bcyS - abs(PID * 4) # speed control
    if bcyS < minS: bcyS = minS # min speed

    # Command senden (Vision-Fehler normiert mit Bildbreite falls verfügbar)
    vision_error_norm = float(P) / float(camera.getWidth()) if camera else 0.0
    send_vision_command(hndB, bcyS, vision_error_norm, Kp * P, Ki * I, Kd * D, 0.0)
    printStatus()
    
    pass

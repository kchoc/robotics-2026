from sr.robot3 import Robot
import json
import math

robot = Robot()
target_id = 1  # target marker to navigate toward

def get_marker_data():
    markers = robot.camera.see()
    for m in markers:
        if m.id == target_id:
            # Return distance and rotation to the marker
            return (m.distance, m.rot_y)
    return (math.inf, 0.0)

while robot.tick():
    # Receive actions from supervisor (wheel speeds)
    msg = robot.receive()
    if msg:
        try:
            left, right = json.loads(msg)
        except Exception:
            left, right = (0, 0)
        robot.motors[0].power = left
        robot.motors[1].power = right

    # Send marker observations back
    data = get_marker_data()
    robot.send(json.dumps(data))

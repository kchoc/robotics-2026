import numpy as np


def rotation_matrix_from_euler(roll, pitch, yaw):
    """
    Aircraft principal axes
    roll  = φ (X axis)
    pitch = θ (Y axis)
    yaw   = ψ (Z axis)

    Returns R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    """

    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])

    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])

    R = Rx @ Ry @ Rz

    return R


def true_floor_yaw(roll, pitch, yaw, tilt=None):
    """
    Computes true yaw on the ground (mod pi/2).

    If tilt is given, assumes camera is tilted downward
    around X axis by that many radians and compensates for it.
    """
    
    # Create rotation matrix from measured orientation
    R = rotation_matrix_from_euler(roll, pitch, yaw)
    
    # Apply inverse tilt if provided
    if tilt is not None:
        R_tilt_inverse = rotation_matrix_from_euler(0, -tilt, 0)
        R = R_tilt_inverse @ R
    
    # Extract the forward direction (first column of rotation matrix)
    forward = R[:, 0]
    
    # Project forward direction onto horizontal plane (xy-plane)
    forward_horizontal = np.array([forward[0], forward[1], 0])
    
    # Compute yaw from projected forward vector
    floor_yaw = np.arctan2(forward_horizontal[1], forward_horizontal[0])    
    
    # Check if upright (z-axis pointing up)
    z_axis = R[:, 2]
    is_upright = int(z_axis[2] > 0.5)
    
    return floor_yaw, is_upright

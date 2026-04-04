import numpy as np

def true_floor_yaw(R, tilt=None):
  """
  Computes horizontal yaw (rotation around world Z axis).

  If tilt is given, assumes camera is tilted downward around the
  camera X axis by that many radians and compensates to a level frame.
  """

  # Bring the observed rotation into a level frame before extracting yaw.
  if tilt is not None:
    R_tilt_comp = np.array([
      [1, 0, 0],
      [0, np.cos(tilt), -np.sin(tilt)],
      [0, np.sin(tilt), np.cos(tilt)]
    ])
    R = R_tilt_comp @ R

  # Yaw from the world-horizontal heading of the rotated X axis.
  floor_yaw = np.arctan2(R[1, 0], R[0, 0])

  # Check if upright (local z-axis pointing up in level/world frame)
  z_axis = R[:, 2]
  is_upright = int(z_axis[2] > 0.5)

  return floor_yaw, is_upright

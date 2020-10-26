# -*- coding: utf-8 -*-
# ------------------------------------------------------------------------------
#
#  HEBI Core python API - Copyright 2018 HEBI Robotics
#  See https://hebi.us/softwarelicense for license details
#
# ------------------------------------------------------------------------------


import ctypes as _ctypes
import numpy as _np
from ._internal import math_utils as _math_utils
from ._internal import raw as _raw
from ._internal import trajectory as _trajectory

_c_double_p = _ctypes.POINTER(_ctypes.c_double)

def create_trajectory(time, position, velocity=None, acceleration=None):
  """
  Creates a smooth trajectory through a set of waypoints (position
  velocity and accelerations defined at particular times). This trajectory
  wrapper object can create multi-dimensional trajectories (i.e., multiple
  joints moving together using the same time reference).

  :param time: A vector of desired times at which to reach each
               waypoint; this must be defined
               (and not ``None`` or ``nan`` for any element).
  :type time:  list, numpy.ndarray

  :param position: A matrix of waypoint joint positions (in SI units). The
                   number of rows should be equal to the number of joints,
                   and the number of columns equal to the number of waypoints. 
                   Any elements that are ``None`` or ``nan`` will be considered
                   free parameters when solving for a trajectory.
                   Values of ``+/-inf`` are not allowed.
  :type position:  str, list, numpy.ndarray, ctypes.Array
  
  :param velocity: An optional matrix of velocity constraints at the
                   corresponding waypoints; should either be ``None``
                   or matching the size of the positions matrix.
                   Any elements that are ``None`` or ``nan`` will be considered
                   free parameters when solving for a trajectory.
                   Values of ``+/-inf`` are not allowed.
  :type velocity:  NoneType, str, list, numpy.ndarray, ctypes.Array
  
  :param acceleration: An optional matrix of acceleration constraints at
                       the corresponding waypoints; should either be ``None``
                       or matching the size of the positions matrix.
                       Any elements that are ``None`` or ``nan`` will be considered
                       free parameters when solving for a trajectory.
                       Values of ``+/-inf`` are not allowed.
  :type acceleration:  NoneType, str, list, numpy.ndarray, ctypes.Array
  
  :return: The trajectory. This will never be ``None``.
  :rtype: Trajectory

  :raises ValueError: If dimensionality or size of any
                      input parameters are invalid.
  :raises RuntimeError: If trajectory could not be created.
  """
  time = _np.asarray(time, _np.float64)
  position = _np.asmatrix(position, _np.float64)

  joints = position.shape[0]
  waypoints = position.shape[1]

  if time.size != waypoints:
    raise ValueError('length of time vector must be equal to number of waypoints')

  if not _math_utils.is_finite(time):
    raise ValueError('time vector must have all finite values')

  if velocity is not None:
    velocity = _np.asmatrix(velocity, _np.float64)
    if velocity.shape[0] != joints or velocity.shape[1] != waypoints:
      raise ValueError('Invalid dimensionality of velocities matrix')
    velocity_c = velocity.getA1().ctypes.data_as(_c_double_p)
    get_vel_offset = lambda i: _ctypes.cast(_ctypes.byref(velocity_c.contents, i * waypoints * 8), _c_double_p)
  else:
    velocity_c = _ctypes.cast(_ctypes.c_void_p(0), _c_double_p)
    get_vel_offset = lambda i: velocity_c

  if acceleration is not None:
    acceleration = _np.asmatrix(acceleration, _np.float64)
    if acceleration.shape[0] != joints or acceleration.shape[1] != waypoints:
      raise ValueError('Invalid dimensionality of velocities matrix')
    acceleration_c = acceleration.getA1().ctypes.data_as(_c_double_p)
    get_acc_offset = lambda i: _ctypes.cast(_ctypes.byref(acceleration_c.contents, i * waypoints * 8), _c_double_p)
  else:
    acceleration_c = _ctypes.cast(_ctypes.c_void_p(0), _c_double_p)
    get_acc_offset = lambda i: acceleration_c

  time_c = time.ctypes.data_as(_c_double_p)
  position_c = position.getA1().ctypes.data_as(_c_double_p)
  trajectories = [None] * joints

  for i in range(0, joints):

    pos_offset = _ctypes.cast(_ctypes.byref(position_c.contents, i * waypoints * 8), _c_double_p)
    vel_offset = get_vel_offset(i)
    acc_offset = get_acc_offset(i)

    trajectory = _raw.hebiTrajectoryCreateUnconstrainedQp(
      waypoints, pos_offset, vel_offset, acc_offset, time_c)

    if not trajectory:
      raise RuntimeError('Could not create trajectory')
    trajectories[i] = trajectory

  return _trajectory.Trajectory(trajectories, time.copy(), waypoints)

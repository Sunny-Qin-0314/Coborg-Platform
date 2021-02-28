# -*- coding: utf-8 -*-
# ------------------------------------------------------------------------------
#
#  HEBI Core python API - Copyright 2018 HEBI Robotics
#  See https://hebi.us/softwarelicense for license details
#
# ------------------------------------------------------------------------------


from .raw import *
from ctypes import c_double, byref
import numpy as np


class Trajectory(object):
  """
  Represents a smooth trajectory through a set of waypoints.
  """

  __slots__ = ['_trajectories', '_waypoint_times', '_number_of_waypoints', '_number_of_joints', '_start_time', '_end_time']

  def __init__(self, trajectories, waypoint_times, num_waypoints):
    self._trajectories = trajectories
    self._waypoint_times = waypoint_times
    self._number_of_waypoints = num_waypoints
    self._number_of_joints = len(trajectories)
    self._start_time = waypoint_times[0]
    self._end_time = waypoint_times[-1]

  @property
  def number_of_waypoints(self):
    """
    The number of waypoints in this trajectory.

    :return: number of waypoints
    :rtype:  int
    """
    return self._number_of_waypoints

  @property
  def number_of_joints(self):
    """
    The number of joints in this trajectory.

    :return: the number of joints
    :rtype:  int
    """
    return len(self._trajectories)

  @property
  def start_time(self):
    """
    The time (in seconds) at which the trajectory starts.

    :return: the start time
    :rtype:  float
    """
    return self._start_time

  @property
  def end_time(self):
    """
    The time (in seconds) at which the trajectory ends.

    :return: the end time
    :rtype:  float
    """
    return self._end_time

  @property
  def duration(self):
    """
    The time (in seconds) between the start and end of this trajectory.

    :return: the duration
    :rtype:  float
    """
    return self._end_time - self._start_time

  @property
  def waypoint_times(self):
    """

    :return: The input time (in seconds) for each waypoint
    :rtype:  numpy.ndarray
    """
    return self._waypoint_times

  def get_state(self, time):
    """
    Returns the position, velocity, and acceleration for a given point
    in time along the trajectory.

    :param time: the time, in seconds
    :type time:  int, float

    :return: a triplet containing the position, velocity, and acceleration
             at the given point in time.
    :rtype:  numpy.ndarray, numpy.ndarray, numpy.ndarray
    """
    position_val = c_double(0.0)
    velocity_val = c_double(0.0)
    acceleration_val = c_double(0.0)

    position = np.zeros(self.number_of_joints, np.float64)
    velocity = np.zeros(self.number_of_joints, np.float64)
    acceleration = np.zeros(self.number_of_joints, np.float64)

    for i, trajectory in enumerate(self._trajectories):
      passed_run =\
        hebiTrajectoryGetState(trajectory, time,
                               byref(position_val),
                               byref(velocity_val),
                               byref(acceleration_val)) == StatusSuccess

      position[i] = position_val.value
      velocity[i] = velocity_val.value
      acceleration[i] = acceleration_val.value

    return position, velocity, acceleration

# -*- coding: utf-8 -*-
# ------------------------------------------------------------------------------
#
#  HEBI Core python API - Copyright 2018 HEBI Robotics
#  See https://hebi.us/softwarelicense for license details
#
# ------------------------------------------------------------------------------

from ._internal import log_file as _log_file
from ._internal import group as _group
from ._internal import raw as _raw

from os.path import isfile as _isfile

try:
  import imp
  imp.find_module('matplotlib')
  _found_matplotlib = True


  def _field_is_command(field_name):
    return field_name.snake_case in ['position', 'velocity', 'effort']


except ImportError:
  _found_matplotlib = False

if not _found_matplotlib:
  print('matplotlib not found - hebi.util.plot_logs and hebi.util.plot_trajectory will not work.')


def create_imitation_group(size):
  """
  Create an imitation group of the provided size.
  The imitation group returned from this function provides the exact same
  interface as a group created from the :class:`Lookup` class.

  However, there are a few subtle differences between the imitation group and
  group returned from a lookup operation. See :ref:`imitation-group-contrast` section
  for more information.

  :param size: The number of modules in the imitation group
  :type size:  int
  
  :return: The imitation group. This will never be ``None``
  :rtype:  Group

  :raises ValueError: If size is less than 1
  """
  return _group.create_imitation_group(size)


def load_log(file):
  """
  Opens an existing log file.

  :param file: the path to an existing log file
  :type file:  str, unicode

  :return: The log file. This function will never return ``None``
  :rtype:  LogFile
  
  :raises TypeError: If file is an invalid type
  :raises IOError: If the file does not exist or is not a valid log file
  """
  try:
    f_exists = _isfile(file)
  except TypeError as t:
    raise TypeError('Invalid type for file. '
                    'Caught TypeError with message: {0}'.format(t.args))

  if not f_exists:
    raise IOError('file {0} does not exist'.format(file))

  log_file = _raw.hebiLogFileOpen(file.encode('utf-8'))
  if log_file is None:
    raise IOError('file {0} is not a valid log file'.format(file))

  return _log_file.LogFile(log_file)


def plot_logs(logs, fbk_field, figure_spec=None, modules=None):
  """
  Nicely formatted plotting of HEBI logs.

  :param logs: A single hebiLog object or a cell array of log objects.
  :type logs: list, LogFile

  :param fbk_field: Feedback field to plot
  :type fbk_field:  str

  :param figure_spec: The figure number you would like to use for plots. If multiple log files
                      are plotted, then the subsequent figure numbers increment by 1 starting
                      at `figure_spec`. If unspecified, a new figure will be created to avoid
                      overwriting previous figures.
  :type figure_spec:  int

  :param modules: Optionally select which modules to plot
  :type modules: NoneType, list
  """
  from matplotlib import pyplot as plt
  import numpy as np
  if isinstance(logs, _log_file.LogFile):
    logs = [logs]
  elif not isinstance(logs, list):
    raise TypeError('Parameter logs was of unexpected type {0}'.format(type(logs).__name__))

  from ._internal.raw import get_field_info
  feedback_info = get_field_info(fbk_field)
  command_plot = _field_is_command(feedback_info)
  num_logs = len(logs)

  for i in range(num_logs):
    log = logs[i]
    if modules is None:
      plot_mask = [int(j) for j in range(log.number_of_modules)]
    else:
      plot_mask = modules

    if figure_spec is None:
      fig = plt.figure()
    else:
      curr_fig = figure_spec + i
      fig = plt.figure(curr_fig, clear=True)

    if command_plot:
      ax = plt.subplot(2, 1, 1)
      ax2 = plt.subplot(2, 1, 2)
      plt.sca(ax)
    else:
      ax = plt.axes()

    num_plots = len(plot_mask)

    x_module_series = list()
    y_module_series = list()
    cmd_module_series = list()
    diff_module_series = list()

    for _ in range(num_plots):
      x_module_series.append(list())
      y_module_series.append(list())
      cmd_module_series.append(list())
      diff_module_series.append(list())

    get_y = lambda entry: feedback_info.get_field(entry)[plot_mask]
    x_lim_max = 0
    x_lim_min = None

    if command_plot:
      cmd_info = get_field_info(feedback_info.snake_case + "_command")
      get_cmd = lambda entry: cmd_info.get_field(entry)[plot_mask]

      last_entry = None

      for entry in log.feedback_iterate:
        last_entry = entry
        entry_time = entry.time[plot_mask]
        entry_y    = get_y(entry)
        entry_cmd  = get_cmd(entry)

        if x_lim_min is None:
          x_lim_min = entry_time.min()

        for j in range(num_plots):
          x = entry_time[j]
          y = entry_y[j]
          cmd = entry_cmd[j]
          diff = y - cmd

          x_module_series[j].append(x)
          y_module_series[j].append(y)
          cmd_module_series[j].append(cmd)
          diff_module_series[j].append(diff)
      x_lim_max = last_entry.time[plot_mask].max()
    else:
      last_entry = None
      for entry in log.feedback_iterate:
        last_entry = entry
        entry_time = entry.time[plot_mask]
        entry_y = get_y(entry)

        if x_lim_min is None:
          x_lim_min = entry_time.min()

        for j in range(num_plots):
          x = entry_time[j]
          y = entry_y[j]
          x_module_series[j].append(x)
          y_module_series[j].append(y)
      x_lim_max = last_entry.time[plot_mask].max()

    y_label_unit_str = ' ({0})'.format(feedback_info.units)

    for j in range(num_plots):
      plt.plot(x_module_series[j], y_module_series[j])

    plt.xlabel('time (sec)')
    plt.ylabel(feedback_info.pascal_case + y_label_unit_str)
    plt.title('{0} - Log {1} of {2}'.format(feedback_info.pascal_case, i + 1, num_logs))
    plt.xlim(x_lim_min, x_lim_max)
    plt.grid(True)

    if command_plot:
      plt.sca(ax2)
      plt.xlabel('time (sec)')
      plt.ylabel('error' + y_label_unit_str)
      plt.title('{0} error'.format(feedback_info.pascal_case))
      plt.xlim(x_lim_min, x_lim_max)
      plt.grid(True)

      for j in range(num_plots):
        plt.sca(ax)
        ax.ColorOrderIndex = 1
        plt.plot(x_module_series[j], cmd_module_series[j], '--')

        plt.sca(ax2)
        plt.plot(x_module_series[j], diff_module_series[j])

    plt.legend([str(mask) for mask in plot_mask])

  plt.show()


def plot_trajectory(trajectory, dt=0.01, figure_spec=None, legend=None):
  """
  Visualizes position, velocity, and acceleration of a trajectory.

  :param trajectory:
  :type trajectory:  hebi.trajectory.Trajectory

  :param dt: Delta between points in trajectory to plot
  :type dt:  int, float

  :param figure_spec: The figure number or figure handle that should be used for plotting.
                      If a figure with the specified number exists it will be overwritten.
                      If left unspecified, a new figure will automatically be generated.
  :type figure_spec:  int, str

  :param legend: String of the text that gets displayed as the legend.
                 By default it shows the joint number.
  :type legend:  str
  """
  from matplotlib import pyplot as plt
  import numpy as np

  if figure_spec is None:
    fig = plt.figure()
  elif isinstance(figure_spec, (int, str)):
    fig = plt.figure(num=figure_spec)
  else:
    raise TypeError('Parameter figure_spec was of unexpected type {0}.'.format(type(figure_spec).__name__))

  if legend is None:
    # Joint number::: TODO
    legend = ''

  duration = trajectory.duration
  linear_times = np.arange(0.0, duration, dt)
  actual_times = trajectory.waypoint_times

  linear_pos = [0.0] * len(linear_times)
  linear_vel = [0.0] * len(linear_times)
  linear_acc = [0.0] * len(linear_times)
  actual_pos = [0.0] * len(actual_times)
  actual_vel = [0.0] * len(actual_times)
  actual_acc = [0.0] * len(actual_times)

  for i, t in enumerate(linear_times):
    p_t, v_t, a_t = trajectory.get_state(t)
    linear_pos[i] = p_t
    linear_vel[i] = v_t
    linear_acc[i] = a_t

  for i, t in enumerate(actual_times):
    p_t, v_t, a_t = trajectory.get_state(t)
    actual_pos[i] = p_t
    actual_vel[i] = v_t
    actual_acc[i] = a_t

  # NOTE:
  #   time          == linear_times
  #   waypointTime  == actual_times

  ax = plt.subplot(3, 1, 1)
  plt.plot(linear_times, linear_pos)
  ax.ColorOrderIndex = 1

  plt.plot(actual_times, actual_pos, marker='o')
  plt.title('Trajectory Profile')
  plt.ylabel('position (rad)')
  plt.xlabel('time (sec)')
  plt.grid(True)

  plt.legend(legend)

  ax = plt.subplot(3, 1, 2)
  plt.plot(linear_times, linear_vel)
  ax.ColorOrderIndex = 1
  plt.plot(actual_times, actual_vel, marker='o')
  plt.ylabel('velocity (rad/sec)')
  plt.xlabel('time (sec)')
  plt.grid(True)

  ax = plt.subplot(3, 1, 3)
  plt.plot(linear_times, linear_acc)
  ax.ColorOrderIndex = 1
  plt.plot(actual_times, actual_acc, marker='o')
  plt.ylabel('acceleration (rad/sec^2)')
  plt.xlabel('time (sec)')
  plt.grid(True)

  plt.show()


def clear_all_groups():
  """
  Clear all groups currently allocated by the API.
  This is useful to clear up resources when running in an environment such as IPython.
  """
  from ._internal.group import GroupDelegate
  GroupDelegate.destroy_all_instances()

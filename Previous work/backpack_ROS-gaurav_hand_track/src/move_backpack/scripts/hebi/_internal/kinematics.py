# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
#
#  HEBI Core python API - Copyright 2018 HEBI Robotics
#  See https://hebi.us/softwarelicense for license details
#
# -----------------------------------------------------------------------------


import numpy as np
from math import pi

from .utils import CaseInvariantString, intern_string

neg_half_pi = pi*-0.5


# -----------------------------------------------------------------------------
# Transform Functions
# -----------------------------------------------------------------------------


def set_translate(matrix, x, y, z):
  matrix[0, 3] = x
  matrix[1, 3] = y
  matrix[2, 3] = z


def set_rotate_x(matrix, radians):
  from math import cos, sin
  c_r = cos(radians)
  s_r = sin(radians)
  matrix[0, 0] = 1.0
  matrix[0, 1] = 0.0
  matrix[0, 2] = 0.0
  matrix[1, 0] = 0.0
  matrix[1, 1] = c_r
  matrix[1, 2] = -s_r
  matrix[2, 0] = 0.0
  matrix[2, 1] = s_r
  matrix[2, 2] = c_r


def set_sphere_inertia(inertia, mass, radius):
  inertia[0:3] = 0.4 * mass * radius * radius
  inertia[3:6] = 0.0


def set_rod_x_axis_inertia(inertia, mass, length):
  inertia[1:3] = mass * length * length * 0.083333333333333333
  inertia[3:6] = inertia[0] = 0.0


# ------------------------------------------------------------------------------
# Descriptor classes returned from parser functions
# ------------------------------------------------------------------------------


class ActuatorDesc(object):

  __slots__ = ['_com', '_com_xyz', '_inertia', '_input_to_axis', '_input_to_axis_xyz', '_mass', '_name']

  def __init__(self, name, mass, inertia, com_xyz, input_to_axis_xyz):
    self._name = name
    self._mass = mass
    self._inertia = np.array(inertia, dtype=np.float64)
    self._com_xyz = com_xyz
    self._input_to_axis_xyz = input_to_axis_xyz
    self._com = np.identity(4, np.float64)
    self._input_to_axis = np.identity(4, np.float64)

    v = com_xyz
    set_translate(self._com, v[0], v[1], v[2])
    v = input_to_axis_xyz
    set_translate(self._input_to_axis, v[0], v[1], v[2])

  @property
  def name(self):
    return self._name

  @property
  def mass(self):
    return self._mass

  @property
  def moments_of_inertia(self):
    return self._inertia

  @property
  def com(self):
    return self._com

  @property
  def input_to_axis(self):
    return self._input_to_axis


class BracketDesc(object):

  __slots__ = ['_mass', '_mount', '_name']

  def __init__(self, name, mount, mass):
    self._name = name
    self._mount = mount
    self._mass = mass

  @property
  def name(self):
    return self._name

  @property
  def mount(self):
    return self._mount

  @property
  def mass(self):
      return self._mass


# ------------------------------------------------------------------------------
# Descriptor Classes for parsing
# ------------------------------------------------------------------------------

# TODO: Make this a bit more future proof...


class BracketParserMatch(object):

  __slots__ = ['_mount', '_parser']

  def __init__(self, parser, mount):
    self._parser = parser
    self._mount = mount

  @property
  def bracket_name(self):
    return self._parser.name

  @property
  def light(self):
    return 'Light' in self._parser.name

  @property
  def heavy(self):
    return 'Heavy' in self._parser.name

  @property
  def left(self):
    return 'left' in self._mount

  @property
  def right(self):
    return 'right' in self._mount

  @property
  def inside(self):
    return 'inside' in self._mount

  @property
  def outside(self):
    return 'outside' in self._mount

  @property
  def mass(self):
    return self._parser.mass


class BracketParser(object):

  __slots__ = ['_mass', '_mounts', '_name']

  def __init__(self, name, mounts, mass):
    self._name = intern_string(name)
    self._mounts = [intern_string(mount) for mount in mounts]
    self._mass = mass

  def match(self, name, mount):
    if name == self._name:
      mount = intern_string(mount)
      for entry in self._mounts:
        if mount == entry:
          return BracketParserMatch(self, mount)
    return None

  @property
  def mass(self):
    return self._mass

  @property
  def name(self):
    return self._name


# ------------------------------------------------------------------------------
# Maps and Lists for HEBI Products
# ------------------------------------------------------------------------------


from threading import local


class KinematicsParserTLS(local):
  """
  Used to cache valid strings mapped to kinematics objects
  """

  __slots__ = ['parsed_joint_strs', 'parsed_frame_type_strs', 'parsed_actuator_strs', 'parsed_link_strs', 'parsed_bracket_strs']

  def __init__(self):
    super(KinematicsParserTLS, self).__init__()
    self.parsed_joint_strs = dict()
    self.parsed_frame_type_strs = dict()
    self.parsed_actuator_strs = dict()
    self.parsed_link_strs = dict()
    self.parsed_bracket_strs = dict()

  def get_joint(self, joint):
    return self.parsed_joint_strs.get(joint)

  def set_joint(self, joint, value):
    self.parsed_joint_strs[joint] = value

  def get_frame_type(self, frame_type):
    return self.parsed_frame_type_strs.get(frame_type)

  def set_frame_type(self, frame_type, value):
    self.parsed_frame_type_strs[frame_type] = value

  def get_actuator(self, actuator):
    return self.parsed_actuator_strs.get(actuator)

  def set_actuator(self, actuator, value):
    self.parsed_actuator_strs[actuator] = value


_parser_tls = KinematicsParserTLS()

# TODO: Maybe move this into a catalogue module?

from .raw import (JointTypeRotationX, JointTypeRotationY, JointTypeRotationZ,
                  JointTypeTranslationX, JointTypeTranslationY, JointTypeTranslationZ,
                  FrameTypeCenterOfMass, FrameTypeOutput, FrameTypeEndEffector,
                  ActuatorTypeX5_1, ActuatorTypeX5_4, ActuatorTypeX5_9,
                  ActuatorTypeX8_3, ActuatorTypeX8_9, ActuatorTypeX8_16,
                  BracketTypeX5LightLeft, BracketTypeX5LightRight,
                  BracketTypeX5HeavyLeftInside, BracketTypeX5HeavyLeftOutside,
                  BracketTypeX5HeavyRightInside, BracketTypeX5HeavyRightOutside,
                  LinkTypeX5)

__joint_types = {
  'tx': JointTypeTranslationX, 'x': JointTypeTranslationX,
  'ty': JointTypeTranslationY, 'y': JointTypeTranslationY,
  'tz': JointTypeTranslationZ, 'z': JointTypeTranslationZ,
  'rx': JointTypeRotationX,
  'ry': JointTypeRotationY,
  'rz': JointTypeRotationZ}

__frame_types = {
  'CoM': FrameTypeCenterOfMass,
  'com': FrameTypeCenterOfMass,
  'output': FrameTypeOutput,
  'endeffector': FrameTypeEndEffector, 'EndEffector': FrameTypeEndEffector}

__X5_moi = [0.00015, 0.000255, 0.000350, 0.0000341, 0.0000118, 0.00000229]
__X8_moi = [0.000246, 0.000380, 0.000463, 0.0000444, 0.0000266, 0.00000422]

__actuators = {
  'X5-1': ActuatorDesc('X5-1', 0.315, __X5_moi, [0.0142, -0.0031, 0.0165], [0.0, 0.0, 0.03105]),
  'X5-4': ActuatorDesc('X5-4', 0.335, __X5_moi, [0.0142, -0.0031, 0.0165], [0.0, 0.0, 0.03105]),
  'X5-9': ActuatorDesc('X5-9', 0.360, __X5_moi, [0.0142, -0.0031, 0.0165], [0.0, 0.0, 0.03105]),
  'X8-3': ActuatorDesc('X8-3', 0.460, __X8_moi, [-0.0145, -0.0031, 0.0242], [0.0, 0.0, 0.0451]),
  'X8-9': ActuatorDesc('X8-9', 0.480, __X8_moi, [-0.0145, -0.0031, 0.0242], [0.0, 0.0, 0.0451]),
  'X8-16': ActuatorDesc('X8-16', 0.500, __X8_moi, [-0.0145, -0.0031, 0.0242], [0.0, 0.0, 0.0451])}

__actuator_to_enum = {
  CaseInvariantString('X5-1'): ActuatorTypeX5_1,
  CaseInvariantString('X5-4'): ActuatorTypeX5_4,
  CaseInvariantString('X5-9'): ActuatorTypeX5_9,
  CaseInvariantString('X8-3'): ActuatorTypeX8_3,
  CaseInvariantString('X8-9'): ActuatorTypeX8_9,
  CaseInvariantString('X8-16'): ActuatorTypeX8_16}


def _make_tup(a, b):
  return (CaseInvariantString(a), CaseInvariantString(b))

__actuator_brackets_to_enum = {
  _make_tup('X5-LightBracket', 'left'): BracketTypeX5LightLeft,
  _make_tup('X5-LightBracket', 'right'): BracketTypeX5LightRight,
  _make_tup('X5-HeavyBracket', 'left-inside'): BracketTypeX5HeavyLeftInside,
  _make_tup('X5-HeavyBracket', 'right-inside'): BracketTypeX5HeavyRightInside,
  _make_tup('X5-HeavyBracket', 'left-outside'): BracketTypeX5HeavyLeftOutside,
  _make_tup('X5-HeavyBracket', 'right-outside'): BracketTypeX5HeavyRightOutside}

__actuator_links_to_enum = {
  CaseInvariantString('X5'): LinkTypeX5,
  CaseInvariantString('X8'): LinkTypeX5}

__actuator_links = {
  CaseInvariantString('X5'), CaseInvariantString('X8')}

__brackets = [
  BracketParser('X5-LightBracket',
                ['left', 'right'], 0.1),
  BracketParser('X5-HeavyBracket',
                ['left-inside', 'right-inside',
                 'left-outside', 'right-outside'], 0.215)]

# ------------------------------------------------------------------------------
# Parsing Functions
# ------------------------------------------------------------------------------


def __assert_str(value, name='value'):
  if not isinstance(value, str):
    raise TypeError('{0} must be a str'.format(name))


def __get_from_dict(value, dict_, in_type):
  ret = dict_.get(value, None)
  if ret is not None:
    return ret
  raise ValueError("{0} is not a valid {1}. ".format(value, in_type) +
                   "Valid types are: {0}".format(
                     "'" + "', '".join([str(entry) for entry in dict_.keys()]) + "'"))


def parse_frame_type(value):
  tls_val = _parser_tls.get_frame_type(value)
  if tls_val is not None:
    return tls_val
  # If not cached:
  try:
    tls_val = __frame_types[value]
    _parser_tls.set_frame_type(value, tls_val)
    return tls_val
  except Exception as e:
    __assert_str(value, 'frame type')
    if type(e) is KeyError:
      raise ValueError('{0} is not a valid frame type'.format(value))
    else:
      raise e


def parse_joint_type(value):
  tls_val = _parser_tls.get_joint(value)
  if tls_val is not None:
    return tls_val
  # If not cached:
  try:
    fixed_val = value.lower()
    tls_val = __joint_types[fixed_val]
    _parser_tls.set_joint(value, tls_val)
    return tls_val
  except Exception as e:
    __assert_str(value, 'joint type')
    if type(e) is KeyError:
      raise ValueError('{0} is not a valid joint type'.format(value))
    else:
      raise e


def parse_actuator(value):
  tls_val = _parser_tls.get_actuator(value)
  if tls_val is not None:
    return tls_val
  # If not cached:
  try:
    fixed_val = value.upper()
    tls_val = __actuators[fixed_val]
    _parser_tls.set_frame_type(value, tls_val)
    return tls_val, tls_val.com, tls_val.input_to_axis
  except Exception as e:
    __assert_str(value, 'actuator')
    if type(e) is KeyError:
      raise ValueError('{0} is not a valid actuator'.format(value))
    else:
      raise e


def parse_actuator_link(value, extension, twist):
  __assert_str(value)

  value = value.strip().upper()
  if not (value in __actuator_links):
    raise ValueError('{0} is not a valid actuator link'.format(value))

  try:
    extension = float(extension)
  except ValueError as v:
    raise ValueError('cannot convert extension={0} to a float'.format(extension))
  except TypeError as t:
    raise TypeError('cannot convert extension={0} (type {1}) to a float'.format(extension, type(extension)))

  try:
    twist = float(twist)
  except ValueError as v:
    raise ValueError('cannot convert twist={0} to a float'.format(twist))
  except TypeError as t:
    raise TypeError('cannot convert twist={0} (type {1}) to a float'.format(twist, type(twist)))

  return extension, twist


def parse_bracket(bracket, mount):
  __assert_str(bracket, 'bracket')
  __assert_str(mount, 'mount')

  bracket = bracket.strip()
  mount = mount.strip().lower()

  for entry in __brackets:
    match = entry.match(bracket, mount)
    if match:
      break
  else:
    # Should never happen, but let's make this lint-free
    match = None

  if not match:
    raise ValueError('bracket={0} and mount={1} is not a valid bracket'.format(bracket, mount))

  com = np.identity(4, np.float64)
  output = np.identity(4, np.float64)
  mass = match.mass

  if match.light:
    if match.right:
      mult = -1.0
    else:
      mult = 1.0

    set_translate(com, 0.0, mult*0.0215, 0.02)
    set_rotate_x(output, mult*neg_half_pi)
    set_translate(output, 0.0, mult*0.043, 0.04)

  elif match.heavy:
    if match.right:
      lr_mult = -1.0
    else:
      lr_mult = 1.0

    if match.outside:
      y_dist = 0.0375
    else:
      y_dist = -0.0225

    set_translate(com, 0.0, lr_mult * 0.5 * y_dist, 0.0275)
    set_rotate_x(output, lr_mult * neg_half_pi)
    set_translate(output, 0.0, lr_mult * y_dist, 0.055)

  else:
    raise RuntimeError('Unknown bracket type {0}'.format(match.bracket_name))

  return com, output, mass


def actuator_str_to_enum(actuator):
  __assert_str(actuator, 'actuator')
  actuator = CaseInvariantString(actuator)
  if actuator not in __actuator_to_enum:
    raise ValueError('Unknown actuator type {0}'.format(actuator))
  return __actuator_to_enum[actuator]


def bracket_str_to_enum(bracket, mount):
  __assert_str(bracket, 'bracket')
  __assert_str(mount, 'mount')

  k = _make_tup(bracket, mount)
  if k not in __actuator_brackets_to_enum:
    raise ValueError('Unknown bracket type (bracket: {0}, mount: {1})'.format(bracket, mount))
  return __actuator_brackets_to_enum[k]


def link_str_to_enum(link):
  __assert_str(link, 'link')
  link = CaseInvariantString(link)

  if link not in __actuator_links_to_enum:
    raise ValueError('Unknown link type {0}'.format(link))
  return __actuator_links_to_enum[link]

# -*- coding: utf-8 -*-
# ------------------------------------------------------------------------------
#
#  HEBI Core python API - Copyright 2018 HEBI Robotics
#  See https://hebi.us/softwarelicense for license details
#
# ------------------------------------------------------------------------------

from ._internal.errors import HEBI_Exception

from ._internal.utils import UnmanagedObject as __UnmanagedObject
from ._internal import raw as _raw
from ._internal import type_utils as _type_utils
from ._internal import math_utils as _math_utils
from ._internal import kinematics as _kinematics
import ctypes as _ctypes
import math as _math
import numpy as _np
import threading as _threading

_c_double_p = _ctypes.POINTER(_ctypes.c_double)


class _RobotModelTLS(_threading.local):

  __slots__ = ['base_frame', 'inertia', 'double_buffer', 'com_transform', 'output_transform']

  def __init__(self):
    super(_RobotModelTLS, self).__init__()
    self.base_frame = _type_utils.create_double_buffer(16)
    self.inertia = _np.empty(6, dtype=_np.float64)
    self.double_buffer = _type_utils.create_double_buffer(256)
    self.com_transform = _np.identity(4, dtype=_np.float64)
    self.output_transform = _np.identity(4, dtype=_np.float64)


_tls = _RobotModelTLS()
_FrameTypeCenterOfMass = _raw.FrameTypeCenterOfMass
_FrameTypeOutput = _raw.FrameTypeOutput
_FrameTypeEndEffector = _raw.FrameTypeEndEffector
_StatusSuccess = _raw.StatusSuccess


class RobotModel(__UnmanagedObject):
  """
  Represents a chain or tree of robot elements (rigid bodies and joints).
  Currently, only chains of elements are fully supported.
  """

  __slots__ = ['_dof_count', '_frame_count_map', '_masses_cached']

  def __init__(self, **kwargs):
    has_existing_handle = False
    if 'existing_handle' in kwargs:
      kin_handle = kwargs['existing_handle']
      has_existing_handle = True
    else:
      kin_handle = _raw.hebiRobotModelCreate()

    super(RobotModel, self).__init__(kin_handle, _raw.hebiRobotModelRelease)

    self._dof_count = 0
    self._frame_count_map = dict()
    self._frame_count_map[_FrameTypeCenterOfMass] = 0
    self._frame_count_map[_FrameTypeOutput] = 0
    self._frame_count_map[_FrameTypeEndEffector] = 0
    self._masses_cached = _np.empty(0, _np.float64)

    if has_existing_handle:
      self.__update_cached_frame_counts()

  def __repr__(self):
    masses = self.masses
    total_mass = masses.sum()
    body_count = len(masses)
    dof_count = self.dof_count
    # Note: The C API does not support payloads yet. This is intentionally omitted.
    #payload = self.payload
    ret = 'RobotModel with properties:\n'                       +\
          '\n'                                                  +\
          '    body_count: {0}\n'.format(body_count)            +\
          '     dof_count: {0}\n'.format(dof_count)             +\
          '          mass: {:0.2f} [kg]\n'.format(total_mass)

    # Until the C API has the ability to get an individual body, this will not be included.
    #ret = ret +\
    #      '\n'                                                  +\
    #      '    body  type             is_dof  mass \n'          +\
    #      '    ----  ---------------  ------  -----\n'

    return ret

  @property
  def base_frame(self):
    """
    The transform from the world coordinate system to the root kinematic body

    :return: The base frame 4x4 matrix
    :rtype:  numpy.matrix
    """
    frame = _tls.base_frame
    code = _raw.hebiRobotModelGetBaseFrame(self, frame)
    if code != _StatusSuccess:
      raise HEBI_Exception(code, 'hebiRobotModelGetBaseFrame failed')

    return _np.matrix(frame, dtype=_np.float64).reshape(4, 4)

  @base_frame.setter
  def base_frame(self, value):
    """
    Set the transform from a world coordinate system to the input of the root
    element in this robot model. Defaults to an identity 4x4 matrix.

    The world coordinate system is used for all position, vector,
    and transformation matrix parameters in the member functions.

    :param value: A 4x4 matrix representing the base frame
    :type value:  str, list, numpy.ndarray, ctypes.Array

    :raises HEBI_Exception: If the base frame could not be set
    :raises ValueError:     If the input matrix is not of the right size or type
    """
    base_frame = _type_utils.to_contig_sq_mat(value, size=4)
    if not _math_utils.is_finite(base_frame):
      raise ValueError('Base frame must be entirely finite')

    c_double_p = _c_double_p
    code = _raw.hebiRobotModelSetBaseFrame(self, base_frame.ctypes.data_as(c_double_p))
    if code != _StatusSuccess:
      raise HEBI_Exception(code, 'hebiRobotModelSetBaseFrame failed')

  def __get_frame_count(self, frame_type):
    return self._frame_count_map[frame_type]

  def get_frame_count(self, frame_type):
    """
    The number of frames in the forward kinematics.

    Note that this depends on the type of frame requested:
      * for center of mass frames, there is one per added body.
      * for output frames, there is one per output per body.

    Valid strings for valid frame types are:
      * For center of mass:  ``'CoM'`` or ``'com'``
      * For output:          ``'output'``

    :param frame_type: Which type of frame to consider
    :type frame_type:  str

    :return: the number of frames of the specified type
    :rtype:  int

    :raises ValueError: If the string from ``frame_type`` is invalid
    :raises TypeError:  If the ``frame_type`` argument is not a string
    """
    return self.__get_frame_count(_kinematics.parse_frame_type(frame_type))

  @property
  def dof_count(self):
    """
    The number of settable degrees of freedom in the kinematic tree.
    This is equal to the number of actuators added.

    :return: the degrees of freedom.
    :rtype:  int
    """
    return self._dof_count

  def __update_cached_masses(self, num_com_frames):
    masses = _np.empty(num_com_frames, dtype=_np.float64)
    _raw.hebiRobotModelGetMasses(self, masses.ctypes.data_as(_c_double_p))
    self._masses_cached = masses

  def __update_cached_frame_counts(self):
    self._dof_count = int(_raw.hebiRobotModelGetNumberOfDoFs(self))
    new_com_count = _raw.hebiRobotModelGetNumberOfFrames(self, _FrameTypeCenterOfMass)
    self._frame_count_map[_FrameTypeCenterOfMass] = new_com_count
    self._frame_count_map[_FrameTypeOutput] = _raw.hebiRobotModelGetNumberOfFrames(self, _FrameTypeOutput)
    self._frame_count_map[_FrameTypeEndEffector] = _raw.hebiRobotModelGetNumberOfFrames(self, _FrameTypeEndEffector)
    self.__update_cached_masses(new_com_count)

  def __assert_equals_dof_count(self, positions):
    expect = self._dof_count
    actual = len(positions)
    if actual != expect:
      raise ValueError('Input positions must be of same length of DOFs (expected length of {0}, got {1})'.format(expect, actual))

  def __try_add(self, body, combine):
    res = _raw.hebiRobotModelAdd(self, None, 0, body, int(combine))
    if res != _StatusSuccess:
      _raw.hebiRobotModelElementRelease(body)
      raise HEBI_Exception(res, 'hebiRobotModelAdd failed')
    self.__update_cached_frame_counts()

  def add_rigid_body(self, com, inertia, mass, output, combine):
    """
    Adds a rigid body with the specified properties to the robot model.

    This can be 'combined' with the parent element
    (the element to which this is attaching), which means that the mass,
    inertia, and output frames of this element will be integrated with
    the parent. The mass will be combined, and the reported parent output frame
    that this element attached to will be replaced with the output from
    this element (so that the number of output frames and masses remains constant).

    :param com: 3 element vector or 4x4 matrix.
                If this parameter is a 3 element vector, the elements will be used
                as the translation vector in a homogeneous transformation matrix.
                The homogeneous transform is to the center
                of mass location, relative to the input frame of the element.
                Note that this frame is also the frame in which
                the inertia tensor is given.
    :type com:  str, list, numpy.ndarray, ctypes.Array

    :param inertia: The 6 element representation (Ixx, Iyy, Izz, Ixy, Ixz, Iyz)
                    of the inertia tensor, in the frame given by the COM.
    :type inertia:  str, list, numpy.ndarray, ctypes.Array

    :param mass:    The mass of this element.
    :type mass:     int, float

    :param output:  4x4 matrix of the homogeneous transform to the output frame,
                    relative to the input frame of the element.
    :type output:   str, list, numpy.ndarray, ctypes.Array

    :param combine: ``True`` if the frames and masses of this body should
                    be combined with the parent, ``False`` otherwise.
    :type combine:  bool

    :return: ``True`` if the body could be added, ``False`` otherwise.
    :rtype:  bool

    :raises ValueError: if com, inertia, or output are of wrong size
    """
    inertia = _np.asarray(inertia)
    if len(inertia) != 6:
      raise ValueError('inertia must be a 6 element array')

    c_double_p = _c_double_p
    user_com = _np.asarray(com, _np.float64)
    if user_com.shape == (3,):
      # User provided 3 element array [x,y,z]
      # Use this in the translation verctor of transform
      com = _tls.com_transform
      com[0:3, 3] = user_com
    elif user_com.shape == (4, 4):
      _np.copyto(com, user_com)

    _np.copyto(_tls.output_transform, output)
    com = com.ctypes.data_as(c_double_p)
    output = output.ctypes.data_as(c_double_p)

    inertia = inertia.ctypes.data_as(c_double_p)
    body = _raw.hebiRobotModelElementCreateRigidBody(com, inertia, mass, 1, output)
    self.__try_add(body, combine)
    return True

  def add_joint(self, joint_type, combine=False):
    """
    Adds a degree of freedom about the specified axis.

    This does not represent an element with size or mass, but only a
    connection between two other elements about a particular axis.

    :param joint_type: The axis of rotation or translation about which this
                       joint allows motion.

                       For a linear joint, use:
                       ``tx``, ``x``, ``y``, ``ty``, ``tz``, or ``z``

                       For a rotation joint, use:
                       ``rx``, ``ry``, or ``rz``

                       This argument is case insensitive.
    :type joint_type:  str

    :param combine: Whether or not to combiner with previous element in body
    :type combine:  bool

    :raises ValueError: If the string from ``joint_type`` is invalid
    :raises TypeError:  If the ``joint_type`` argument is not a string
    """
    return self.__try_add(_raw.hebiRobotModelElementCreateJoint(_kinematics.parse_joint_type(joint_type)), combine)

  def add_actuator(self, actuator_type):
    """
    Add an element to the robot model with the kinematics/dynamics of an
    X5 actuator.

    :param actuator_type: The type of actuator to add.
    :type actuator_type:  str, unicode

    :return: ``True`` if the actuator could be added, ``False`` otherwise.
    :rtype:  bool

    :raises ValueError: If the string from ``actuator_type`` is invalid
    :raises TypeError:  If the ``actuator_type`` argument is not a string
    """
    return self.__try_add(_raw.hebiRobotModelElementCreateActuator(_kinematics.actuator_str_to_enum(actuator_type)), False)

  def add_link(self, link_type, extension, twist):
    """
    Add an element to the robot model with the kinematics/dynamics
    of a link between two actuators.

    :param link_type: The type of link between the actuators, e.g. a tube link
                      between two X5 or X8 actuators.
    :type link_type:  str, unicode

    :param extension: The center-to-center distance between the actuator
                      rotational axes.
    :type extension:  int, float

    :param twist:     The rotation (in radians) between the actuator axes of
                      rotation. Note that a 0 radian rotation will result
                      in a z-axis offset between the two actuators,
                      and a pi radian rotation will result in the actuator
                      interfaces to this tube being in the same plane, but the
                      rotational axes being anti-parallel.
    :type twist:      int, float

    :return: ``True`` if link was added, ``False`` otherwise
    :rtype:  bool

    :raises ValueError: If the string from ``link_type`` is invalid
    :raises TypeError:  If the ``link_type`` argument is not a string
    """
    extension = float(extension)
    twist = float(twist)
    return self.__try_add(_raw.hebiRobotModelElementCreateLink(_kinematics.link_str_to_enum(link_type), extension, twist), False)

  def add_bracket(self, bracket_type, mount):
    """
    Add an element to the robot model with the kinematics/dynamics of a bracket
    between two actuators.

    :param bracket_type: The type of bracket to add.
    :type bracket_type:  str, unicode

    :param mount: The mount type of the bracket
    :type mount:  str, unicode

    :return: ``True`` if bracket was added, ``False`` otherwise
    :rtype:  bool

    :raises ValueError: If the string from either ``bracket_type`` or ``mount`` are invalid
    :raises TypeError:  If the either ``bracket_type`` or ``mount`` arguments are not strings
    """
    return self.__try_add(_raw.hebiRobotModelElementCreateBracket(_kinematics.bracket_str_to_enum(bracket_type, mount)), False)

  def get_forward_kinematics(self, frame_type, positions, output=None):
    """
    Generates the forward kinematics for the given robot model.

    The order of the returned frames is in a depth-first tree.

    :param frame_type: Which type of frame to consider. See :meth:`.get_frame_count` for valid values.
    :type frame_type:  str

    :param positions: A vector of joint positions/angles (in SI units of meters
                      or radians) equal in length to the number of DoFs
                      of the kinematic tree.
    :type positions:  str, list, numpy.ndarray, ctypes.Array

    :param output:    An optional parameter, which, if not ``None``, specifies
                      a list into which to put the output frames. If this
                      parameter is not ``None``, it must be large enough
                      to fit all of the frames. No type or size checking
                      is performed, and it is required that you handle error
                      cases yourself.
    :type output:     list

    :return:  An list of 4x4 transforms; this is resized as necessary
              in the function and filled in with the 4x4 homogeneous transform
              of each frame. Note that the number of frames depends
              on the frame type.
    :rtype:   list

    :raises TypeError:  If ``frame_type`` is not a string
    :raises ValueError: If the ``positions`` input is not equal to the
                        degrees of freedom of the RobotModel
    """
    frame_type = _kinematics.parse_frame_type(frame_type)
    num_frames = self.__get_frame_count(frame_type)

    positions = _np.asarray(positions, _np.float64)
    self.__assert_equals_dof_count(positions)
    if not _math_utils.is_finite(positions):
      raise ValueError('Input positions must be entirely finite')

    # Edge case
    if num_frames == 0:
      return []

    c_double_p = _c_double_p
    positions = positions.ctypes.data_as(c_double_p)

    if len(_tls.double_buffer) < (num_frames * 16):
      _tls.double_buffer = _type_utils.create_double_buffer(num_frames * 16)

    frames = _tls.double_buffer
    _raw.hebiRobotModelGetForwardKinematics(self, frame_type, positions, frames)

    if output is None:
      mat = _np.asmatrix(_np.empty((4, 4), dtype=_np.float64))
      output = [None] * num_frames
      output[0] = mat
      for i in range(1, num_frames):
        output[i] = mat.copy()
    for i in range(0, num_frames):
      start = i * 16
      end = start + 16
      _np.copyto(output[i].ravel(), frames[start:end])
    return output

  def get_end_effector(self, positions, output=None):
    """
    Generates the forward kinematics to the end effector (leaf node)

    Note: for center of mass frames, this is one per leaf node; for output
    frames, this is one per output per leaf node, in depth first order.

    This overload is for kinematic chains that only have a single leaf node frame.

    *Currently, kinematic trees are not fully supported -- only kinematic
    chains -- and so there are not other overloads for this function.*

    :param positions: A vector of joint positions/angles
                      (in SI units of meters or radians) equal in length
                      to the number of DoFs of the kinematic tree.
    :type positions:  str, list, numpy.ndarray, ctypes.Array

    :param output:    An optional parameter which allows you to avoid an allocation
                      by copying the results into this parameter. The size of this
                      parameter is not checked, so you must be certain that it is
                      a numpy array of dtype :class:`float` (``np.float64``)
                      with 16 elements (e.g., a 4x4 matrix or 16 element array)
    :type output:     numpy.ndarray

    :return:  A 4x4 transform that is resized as necessary in the
              function and filled in with the homogeneous transform to the end
              effector frame.
    :rtype:   numpy.matrix

    :raises RuntimeError: If the RobotModel has no output frames
    :raises ValueError:   If the ``positions`` input is not equal to the
                          degrees of freedom of the RobotModel
    """
    num_frames = self.__get_frame_count(_FrameTypeOutput)
    if num_frames == 0:
      raise RuntimeError('Cannot get end effector because RobotModel has no frames')

    positions = _np.asarray(positions, _np.float64)
    self.__assert_equals_dof_count(positions)
    if not _math_utils.is_finite(positions):
      raise ValueError('Input positions must be entirely finite')

    c_double_p = _c_double_p
    positions = positions.ctypes.data_as(c_double_p)

    transforms = _tls.base_frame
    _raw.hebiRobotModelGetForwardKinematics(self, _FrameTypeEndEffector,
                                            positions, transforms)
    if output is None:
      return _np.asmatrix(transforms, dtype=_np.float64).reshape(4, 4)
    else:
      _np.copyto(output.ravel(), transforms)
      return output

  def solve_inverse_kinematics(self, initial_positions, *objectives, **kwargs):
    """
    Solves for an inverse kinematics solution given a set of objectives.

    To avoid unnecessary allocations, provide ``output`` as a keyword argument.
    This argument must be a numpy array or matrix with dtype
    :class:`float` (``np.float64``) with size equal to the initial positions. *e.g.*,

    ``robot.solve_inverse_kinematics(positions, obj1, obj2, output=calc_pos) # calc_pos.size == positions.size``

    :param initial_positions: The seed positions/angles (in SI units of meters
                              or radians) from which to start the IK search;
                              equal in length to the number of DoFs of the
                              kinematic tree.
    :type initial_positions:  str, list, numpy.ndarray, ctypes.Array

    :param objectives:  A variable number of objectives used to define the IK
                        search (e.g., target end effector positions, etc).
                        Each argument must have a base class of Objective.

    :param kwargs:      An optional keyword arguments map, which currently
                        only allows an ``output`` argument

    :return:  A vector equal in length to the number of DoFs of the kinematic tree;
              this will be filled in with the IK solution
              (in SI units of meters or radians) and resized as necessary.
    :rtype:   numpy.ndarray

    :raises HEBI_Exception: If the IK solver failed
    :raises TypeError:      If any of the provided objectives are not
                            an objective type
    :raises ValueError:     If the ``initial_positions`` input is not equal
                            to the degrees of freedom of the RobotModel or has
                            non-finite elements (_i.e_, ``nan``, ``+/-inf``)
    """
    initial_positions = _np.asarray(initial_positions, _np.float64)
    self.__assert_equals_dof_count(initial_positions)
    if not _math_utils.is_finite(initial_positions):
      raise ValueError('Input initial positions must be entirely finite')

    c_double_p = _c_double_p
    dof_count = len(initial_positions)
    has_output = 'output' in kwargs
    if has_output:
      if kwargs['output'].size != dof_count:
        raise ValueError('output size must be same as initial_positions')
    if len(_tls.double_buffer) < dof_count:
      _tls.double_buffer = _type_utils.create_double_buffer(dof_count)

    result = _tls.double_buffer
    initial_positions = initial_positions.ctypes.data_as(c_double_p)

    ik = _raw.hebiIKCreate()

    for entry in objectives:
      if not isinstance(entry, _ObjectiveBase):
        raise TypeError('{0} is not an Objective'.format(entry))
      entry.add_objective(ik)

    code = _raw.hebiIKSolve(ik, self, initial_positions, result, None)
    _raw.hebiIKRelease(ik)

    if code != _StatusSuccess:
      raise HEBI_Exception(code, 'hebiIKSolve failed')
    if has_output:
      output = kwargs['output']
      _np.copyto(output.ravel(), result[0:dof_count])
      return output
    return _np.asarray(result[0:dof_count], _np.float64)

  def __get_jacobians(self, frame_type, positions, num_frames):
    """
    Callee is required to copy results back from TLS buffer
    """
    buffer_len_needed = 6 * num_frames * self.dof_count
    if len(_tls.double_buffer) < buffer_len_needed:
      _tls.double_buffer = _type_utils.create_double_buffer(buffer_len_needed)

    jacobians = _tls.double_buffer
    _raw.hebiRobotModelGetJacobians(self, frame_type, positions, jacobians)

  def get_jacobians(self, frame_type, positions, output=None):
    """
    Generates the Jacobian for each frame in the given kinematic tree.

    :param frame_type: Which type of frame to consider. See :meth:`.get_frame_count` for valid values.
    :param frame_type: str

    :param positions: A vector of joint positions/angles
                      (in SI units of meters or radians)
                      equal in length to the number of DoFs of the
                      kinematic tree.
    :type positions:  str, list, numpy.ndarray, ctypes.Array

    :param output:    An optional parameter which allows you to avoid an allocation
                      by copying the results into this parameter. The size of this
                      parameter is not checked, so you must be certain that it is
                      a list of the proper size with all numpy arrays
                      of dtype :class:`float` (``np.float64``) with (frames x dofs) elements
    :type output:     list

    :return:  A vector (length equal to the number of frames) of
              matrices; each matrix is a (6 x number of dofs)
              jacobian matrix for the corresponding frame of reference
              on the robot. It is resized as necessary inside this function.
    :rtype:   list
    """
    frame_type = _kinematics.parse_frame_type(frame_type)
    num_frames = self.__get_frame_count(frame_type)

    positions = _np.asarray(positions, _np.float64)
    self.__assert_equals_dof_count(positions)
    if not _math_utils.is_finite(positions):
      raise ValueError('Input positions must be entirely finite')

    # Edge case
    if num_frames == 0:
      return []

    c_double_p = _c_double_p
    positions = positions.ctypes.data_as(c_double_p)

    dofs = self.dof_count
    rows = 6 * num_frames
    cols = dofs

    if output is None:
      mat = _np.asmatrix(_np.empty((6, cols), dtype=_np.float64))
      output = [None] * num_frames
      output[0] = mat
      for i in range(1, num_frames):
        output[i] = mat.copy()

    self.__get_jacobians(frame_type, positions, num_frames)
    jacobians = _tls.double_buffer

    stride = cols * 6
    for i in range(0, num_frames):
      start = i * stride
      end = start + stride
      _np.copyto(output[i].ravel(), jacobians[start:end])
    
    return output

  def get_jacobian_end_effector(self, positions, output=None):
    """
    Generates the Jacobian for the end effector (leaf node) frames(s).

    Note: for center of mass frames, this is one per leaf node; for output
    frames, this is one per output per leaf node, in depth first order.

    This overload is for kinematic chains that only have a single leaf node frame.

    *Currently, kinematic trees are not fully supported -- only kinematic
    chains -- and so there are not other overloads for this function.*

    :param positions: A vector of joint positions/angles (in SI units of
                      meters or radians) equal in length to the number of
                      DoFs of the kinematic tree.
    :type positions:  str, list, numpy.ndarray, ctypes.Array

    :param output:    An optional parameter which allows you to avoid an allocation
                      by copying the results into this parameter. The size of this
                      parameter is not checked, so you must be certain that it is
                      a numpy array or matrix of dtype :class:`float` (``np.float64``)
                      with (frames x dofs) elements
    :type output:     numpy.ndarray

    :return:  A (6 x number of dofs) jacobian matrix for the corresponding
              end effector frame of reference on the robot. It is resized as
              necessary inside this function.
    :rtype:   numpy.matrix

    :raises RuntimeError: If the RobotModel has no output frames
    :raises ValueError:   If the ``positions`` input is not equal to the
                          degrees of freedom of the RobotModel
    """
    num_frames = self.__get_frame_count(_FrameTypeOutput)
    if num_frames == 0:
      raise RuntimeError('Cannot get end effector because RobotModel has no frames')

    positions = _np.asarray(positions, _np.float64)
    self.__assert_equals_dof_count(positions)

    if output is None:
      output = _np.asmatrix(_np.zeros((6, self.dof_count), dtype=_np.float64))
    
    positions = positions.ctypes.data_as(_c_double_p)
    self.__get_jacobians(_FrameTypeOutput, positions, num_frames)
    jacobians = _tls.double_buffer

    stride = self.dof_count * 6
    start = (num_frames - 1) * stride
    end = start + stride
    _np.copyto(output.A1, jacobians[start:end])

    return output

  @property
  def masses(self):
    """
    The mass of each rigid body (or combination of rigid bodies) in the robot.

    :return: The masses as an array
    :rtype:  numpy.ndarray
    """
    return self._masses_cached.copy()


# ------------------------------------------------------------------------------
# HRDF functionality
# ------------------------------------------------------------------------------


def _get_hrdf_import_error():
  c_buffer = _raw.hebiRobotModelGetImportError()
  return _type_utils.decode_string_buffer(c_buffer, 'utf-8')


def _get_hrdf_import_warning_count():
  return _raw.hebiRobotModelGetImportWarningCount()


def _get_hrdf_import_warning_at(index):
  if index < 0:
    raise IndexError('{0} < 0'.format(index))
  count = _get_hrdf_import_warning_count()
  if index >= count:
    raise IndexError('{0} >= {1}'.format(index, count))
  c_buffer = _raw.hebiRobotModelGetImportWarning(index)
  return _type_utils.decode_string_buffer(c_buffer, 'utf-8')


def _get_all_hrdf_import_warnings():
  count = _get_hrdf_import_warning_count()
  if count == 0:
    return []

  ret = [None] * count
  for i in range(0, count):
    ret[i] = _get_hrdf_import_warning_at(i)

  return ret


def import_from_hrdf(hrdf_file, warning_callback=None):
  """
  Import a robot description in the HRDF format as a RobotModel instance.

  Any warnings generated while importing will be printed to stderr,
  unless explicitly instructed otherwise by providing a callback.

  :param hrdf_file: The location of the HRDF file to import
  :param warning_callback: A function, which can accept one argument,
                           to call back if warnings are generated. If `None`,
                           warnings are printed to stderr

  :return: the RobotModel from the description file
  :rtype:  RobotModel

  :raises IOError:      If the provided file does not exist
  :raises RuntimeError: If the hrdf file could not be imported.
                        The associated error message will be displayed.
  """

  import os
  hrdf_file = str(hrdf_file)
  if not os.path.isfile(hrdf_file):
    raise IOError('{0} is not a file'.format(hrdf_file))

  from ._internal.type_utils import create_string_buffer_compat as create_str
  handle = _raw.hebiRobotModelImport(create_str(hrdf_file))

  # Check if any warnings were generated
  warnings = _get_all_hrdf_import_warnings()
  if len(warnings) > 0:
    if warning_callback is None:
      msg = 'Importing HRDF file {0} generated the following warning(s):\n'.format(hrdf_file)
      import sys
      sys.stderr.write(msg + '\n'.join(warnings) + '\n')
    else:
      warning_callback(warnings)

  if handle is None:
    error_message = _get_hrdf_import_error()
    raise RuntimeError('Failed to import HRDF file {0}: {1}'.format(hrdf_file, error_message))

  return RobotModel(existing_handle=handle)


# ------------------------------------------------------------------------------
# IK Objective functions
# ------------------------------------------------------------------------------

class _ObjectiveBase(object):

  __slots__ = ['__impl']

  def __init__(self, impl):
    self.__impl = impl

  def add_objective(self, internal):
    self.__impl(internal)


def endeffector_position_objective(xyz, weight=1.0):
  """
  Create a position end effector objective with the given parameters.
  Analogous to `EndEffectorPositionObjective
  <http://docs.hebi.us/docs/cpp/cpp-1.0.0/classhebi_1_1robot__model_1_1
  EndEffectorPositionObjective.html>`_ in the C++ API.

  :param xyz: list of x, y, and z position objective points
  :type xyz:  list, numpy.ndarray, ctypes.Array
  
  :param weight: The weight of the objective
  :type weight:  int, float

  :return: the created objective
  
  :raises ValueError: if xyz does not have at least 3 elements
  """
  xyz = _np.array(xyz, dtype=_np.float64, copy=True)
  if len(xyz) < 3:
    raise ValueError('xyz must have length of at least 3')

  class Objective(_ObjectiveBase):
    def __init__(self, xyz, weight):
      self._x = xyz[0]
      self._y = xyz[1]
      self._z = xyz[2]
      self._weight = float(weight)

      def impl(internal):
        res = _raw.hebiIKAddObjectiveEndEffectorPosition(internal,
                                                         self._weight, 0,
                                                         self._x, self._y,
                                                         self._z)
        if res != _StatusSuccess:
          raise HEBI_Exception(res,
                               'hebiIKAddObjectiveEndEffectorPosition failed')

      super(Objective, self).__init__(impl)

    @property
    def x(self):
      return self._x

    @property
    def y(self):
      return self._y

    @property
    def z(self):
      return self._z

    @property
    def weight(self):
      return self._weight

  return Objective(xyz, weight)


def endeffector_so3_objective(rotation, weight=1.0):
  """
  Create an SO3 end effector objective with the given parameters.
  Analogous to `EndEffectorSO3Objective
  <http://docs.hebi.us/docs/cpp/cpp-1.0.0/classhebi_1_1robot__model_1_1
  EndEffectorSO3Objective.html>`_ in the C++ API.

  :param rotation: SO3 rotation matrix
  :type rotation:  str, list, numpy.ndarray, ctypes.Array
  
  :param weight: The weight of the objective
  :type weight:  int, float

  :return: the created objective
  
  :raises ValueError: if rotation matrix is not convertible to a 3x3 matrix,
                      or if the rotation matrix is not in the
                      `SO(3) <https://en.wikipedia.org/wiki/Rotation_group_SO(3)>`_
                      group.
  """
  rotation = _np.array(rotation, dtype=_np.float64, copy=True)
  rotation = _type_utils.to_contig_sq_mat(rotation, size=3)

  if not _math_utils.is_finite(rotation):
    raise ValueError('Input rotation matrix must be entirely finite')

  if not _math_utils.is_so3_matrix(rotation):
    det = _np.linalg.det(rotation)
    raise ValueError('Input rotation matrix is not SO(3). '
                     'Determinant={0}'.format(det))

  class Objective(_ObjectiveBase):
    def __init__(self, rotation, weight):
      self._rotation = rotation
      self._weight = float(weight)

      def impl(internal):
        rotation = self._rotation.ctypes.data_as(_c_double_p)
        res = _raw.hebiIKAddObjectiveEndEffectorSO3(internal,
                                                    self._weight, 0, rotation)
        if res != _StatusSuccess:
          raise HEBI_Exception(res, 'hebiIKAddObjectiveEndEffectorSO3 failed')

      super(Objective, self).__init__(impl)

    @property
    def rotation(self):
      return self._rotation

    @property
    def weight(self):
      return self._weight

  return Objective(rotation, weight)


def endeffector_tipaxis_objective(axis, weight=1.0):
  """
  Create a tip axis end effector objective with the given parameters.
  Analogous to `EndEffectorTipAxisObjective
  <http://docs.hebi.us/docs/cpp/cpp-1.0.0/classhebi_1_1robot__model_1_1
  EndEffectorTipAxisObjective.html>`_ in the C++ API.

  :param axis: list of x, y, and z tipaxis objective points
  :type axis:  list, numpy.ndarray, ctypes.Array
  
  :param weight: The weight of the objective
  :type weight:  int, float

  :return: the created objective
  
  :raises ValueError: if axis does not have at least 3 elements
  """
  axis = _np.array(axis, dtype=_np.float64, copy=True)
  if len(axis) < 3:
    raise ValueError('axis must have length of at least 3')

  class Objective(_ObjectiveBase):
    def __init__(self, axis, weight):
      self._x = axis[0]
      self._y = axis[1]
      self._z = axis[2]
      self._weight = float(weight)

      def impl(internal):
        res = _raw.hebiIKAddObjectiveEndEffectorTipAxis(internal,
                                                        self._weight, 0,
                                                        self._x, self._y, self._z)
        if res != _StatusSuccess:
          raise HEBI_Exception(res, 'hebiIKAddObjectiveEndEffectorTipAxis failed')

      super(Objective, self).__init__(impl)

    @property
    def x(self):
      return self._x

    @property
    def y(self):
      return self._y

    @property
    def z(self):
      return self._z

    @property
    def weight(self):
      return self._weight

  return Objective(axis, weight)


def joint_limit_constraint(minimum, maximum, weight=1.0):
  """
  Create a joint limit constraint objective.
  Analogous to `JointLimitConstraint
  <http://docs.hebi.us/docs/cpp/cpp-1.0.0/classhebi_1_1robot__model_1_1
  JointLimitConstraint.html>`_ in the C++ API.

  :param minimum: 
  :type minimum:  str, list, numpy.ndarray, ctypes.Array

  :param maximum: 
  :type maximum:  str, list, numpy.ndarray, ctypes.Array

  :param weight: The weight of the objective
  :type weight:  int, float

  :return: the created objective

  :raises ValueError: if minimum and maximum are not of the same size
  """
  minimum = _np.array(minimum, dtype=_np.float64, copy=True)
  maximum = _np.array(maximum, dtype=_np.float64, copy=True)

  if minimum.size != maximum.size:
    raise ValueError('size of min and max joint limit constraints must be equal')

  class Objective(_ObjectiveBase):
    def __init__(self, minimum, maximum, weight):
      self._minimum = minimum
      self._maximum = maximum
      self._weight = float(weight)

      def impl(internal):
        minimum = self._minimum.ctypes.data_as(_c_double_p)
        maximum = self._maximum.ctypes.data_as(_c_double_p)
        res = _raw.hebiIKAddConstraintJointAngles(internal, self._weight,
                                                  self._minimum.size,
                                                  minimum, maximum)
        if res != _StatusSuccess:
          raise HEBI_Exception(res, 'hebiIKAddConstraintJointAngles failed')

      super(Objective, self).__init__(impl)

    @property
    def minimum(self):
      return self._minimum

    @property
    def maximum(self):
      return self._maximum

    @property
    def weight(self):
      return self._weight

  return Objective(minimum, maximum, weight)


def custom_objective(num_errors, func, user_data=None, weight=1.0):
  """
  Construct a custom objective using a provided function.
  The `func` parameter is a function which accepts 3 parameters:
  `positions`, `errors` and `user_data`.

  The first two parameters are guaranteed to be numpy arrays
  with `dtype=np.float64`. The third parameter, `user_data`, may be `None`,
  or set by the user when invoking this function. It is simply used
  to share application state with the callback function.

  The length of `errors` in the callback will be equal to the `num_errors`
  parameter provided to this function.
  The elements in the `errors` parameter should be modified by the function
  to influence the IK solution.

  The `positions` parameter is the joints positions (or angles) at the current
  point in the optimization. This is a read only array - any attempt
  to modify its elements will raise an Exception.

  :param num_errors: The number of independent error values that this objective
                     returns
  :type num_errors:  int

  :param func:       The callback function

  :param weight:     The weight of the objective
  :type weight:      int, float

  :return: the created objective

  :raises ValueError: if num_errors is less than 1
  """
  if num_errors < 1:
    raise ValueError('num_errors must be a positive number')

  class Objective(_ObjectiveBase):

    def __callback(self, user_data, num_positions, c_positions, c_errors):
      """
      The actual callback the C API invokes. Don't let the user mess
      with ffi calls.
      """
      positions = _type_utils.np_array_from_dbl_ptr(c_positions, (num_positions,))
      positions.setflags(write=False) # positions is `const double*`
      errors = _type_utils.np_array_from_dbl_ptr(c_errors, (self._num_errors,))
      self._func(positions, errors, self._user_data)

    def __init__(self, num_errors, func, user_data, weight):
      self._num_errors = num_errors
      self._func = func
      self._weight = float(weight)
      self._user_data = user_data

      from ctypes import CFUNCTYPE, c_size_t, c_void_p
      self._c_func = CFUNCTYPE(None, c_void_p, c_size_t, _c_double_p, _c_double_p)(self.__callback)

      def impl(internal):
        res = _raw.hebiIKAddObjectiveCustom(internal, self._weight,
                                            self._num_errors,
                                            self._c_func, c_void_p(0))
        if res != _StatusSuccess:
          raise HEBI_Exception(res, 'hebiIKAddObjectiveCustom failed')

      super(Objective, self).__init__(impl)

    @property
    def num_errors(self):
      return self._num_errors

    @property
    def func(self):
      return self._func

    @property
    def weight(self):
      return self._weight

  return Objective(num_errors, func, user_data, weight)

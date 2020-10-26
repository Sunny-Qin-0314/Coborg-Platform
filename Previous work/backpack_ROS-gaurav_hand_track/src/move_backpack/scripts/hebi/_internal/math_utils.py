# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
#
#  HEBI Core python API - Copyright 2018 HEBI Robotics
#  See https://hebi.us/softwarelicense for license details
#
# -----------------------------------------------------------------------------


import numpy as np


def is_finite(m):
  """
  Determine if the input matrix has all finite values.
  :param m: a matrix or array of any shape and size
  :type m:  str, list, numpy.ndarray, ctypes.Array
  """
  if isinstance(m, str):
    m = np.asmatrix(m)
  res = np.isfinite(m)
  if isinstance(res, bool):
    return res
  return res.all()


def is_so3_matrix(m):
  """
  Determine if the matrix belongs to the SO(3) group.
  This is found by calculating the determinant and seeing if it equals 1.

  :param m: a 3x3 numpy matrix
  :type m:  str, list, numpy.ndarray, ctypes.Array
  """
  try:
    # Use highest possible precision
    if isinstance(m, str):
      m = np.asmatrix(m, dtype=np.float64)
    det = np.linalg.det(m)

    # Arbitrarily determined. This may change in the future.
    tolerance = 1e-5
    diff = abs(det-1.0)

    return diff < tolerance
  except Exception as e:
    return False

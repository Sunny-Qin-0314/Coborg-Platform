# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
#
#  HEBI Core python API - Copyright 2018 HEBI Robotics
#  See https://hebi.us/softwarelicense for license details
#
# -----------------------------------------------------------------------------


class HEBI_Exception(Exception):
  """
  Internal exception with HEBI error message
  """

  def __init__(self, errcode, message=''):
    self._errcode = errcode
    self._message = message

  def __str__(self):
    return '{0} ({1})'.format(self._message, self._errors.get(int(self._errcode),
                                                    'unknown error'))

  _errors = {
    0x00 : 'no error',
    0x01 : 'invalid argument passed to API call',
    0x02 : 'provided buffer passed to API call is too small',
    0x03 : 'field value not set',
    0x04 : 'API call failed',
    0x05 : 'provided argument passed to API call is out of expected range',
  }

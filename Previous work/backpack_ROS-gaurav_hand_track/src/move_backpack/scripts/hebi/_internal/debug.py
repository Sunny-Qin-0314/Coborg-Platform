# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
#
#  HEBI Core python API - Copyright 2017 HEBI Robotics
#  See https://hebi.us/softwarelicense for license details
#
# -----------------------------------------------------------------------------


__hebi_debug = False

def set_debug_mode(dbg):
  global __hebi_debug
  __hebi_debug = bool(dbg)


def debug_mode():
  return __hebi_debug


def debug_log(msg):
  if __hebi_debug:
    if '\n' in msg:
      for entry in msg.splitlines():
        print('[HEBI Debug] {0}'.format(entry))
    else:
      print('[HEBI Debug] {0}'.format(msg))


def warn_log(msg):
  if '\n' in msg:
    for entry in msg.splitlines():
      print('[HEBI Warning] {0}'.format(entry))
  else:
    print('[HEBI Warning] {0}'.format(msg))

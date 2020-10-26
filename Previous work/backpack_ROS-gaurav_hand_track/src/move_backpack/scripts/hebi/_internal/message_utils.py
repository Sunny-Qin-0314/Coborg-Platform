# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
#
#  HEBI Core python API - Copyright 2018 HEBI Robotics
#  See https://hebi.us/softwarelicense for license details
#
# -----------------------------------------------------------------------------


import math
import numpy
from ctypes import (byref, create_string_buffer,
                    c_int, c_int32, c_int64, c_uint8,
                    c_uint64, c_float, c_double)
from functools import partial as funpart

from .graphics import Color, color_from_int, string_to_color
from .raw import *
from .utils import WeakReferenceContainer
from . import type_utils
from .type_utils import decode_string_buffer as decode_str
from .type_utils import create_string_buffer_compat as create_str


# -----------------------------------------------------------------------------
# Pretty Printers
# -----------------------------------------------------------------------------


def _fmt_float_array(array):
  return '[' + ', '.join(['{:.2f}'.format(i) for i in array]) + ']'


def _numbered_float_repr(c, enum_type):
  try:
    enum_name = enum_type.name
  except:
    enum_name = enum_type
  desc = 'Numbered float (Enumeration {0}):\n'.format(enum_name)
  try:
    container = c._get_ref()
    return desc +\
      '  float1: {0}\n'.format(_fmt_float_array(c.float1)) +\
      '  float2: {0}\n'.format(_fmt_float_array(c.float2)) +\
      '  float3: {0}\n'.format(_fmt_float_array(c.float3)) +\
      '  float4: {0}\n'.format(_fmt_float_array(c.float4)) +\
      '  float5: {0}\n'.format(_fmt_float_array(c.float5)) +\
      '  float6: {0}\n'.format(_fmt_float_array(c.float6)) +\
      '  float7: {0}\n'.format(_fmt_float_array(c.float7)) +\
      '  float8: {0}\n'.format(_fmt_float_array(c.float8)) +\
      '  float9: {0}\n'.format(_fmt_float_array(c.float9))
  except:
    return desc + '  <Group message was finalized>'


def _fmt_io_bank_pin(pins, indent=4):
  indent_str = ''.join([' '] * indent)
  pins_i = pins[0]
  pins_f = pins[1]

  pins_i_str = '[' + ', '.join(['{0:9g}'.format(entry) for entry in pins_i]) + ']'
  pins_f_str = '[' + ', '.join(['{0:9.8g}'.format(entry) for entry in pins_f]) + ']'

  return '{0}Int:   {1}\n'.format(indent_str, pins_i_str) +\
         '{0}Float: {1}'.format(indent_str, pins_f_str)


def _io_bank_repr(bank_container, bank, bank_readable):
  try:
    enum_name = bank.name
  except:
    enum_name = bank
  desc = 'IO Bank \'{0}\' (Enumeration {1}):\n'.format(bank_readable, enum_name)
  try:
    io_container = bank_container._get_ref()
  except:
    # Handles the case where IO Container object was finalized already 
    return desc + "  <IO Container was finalized>"

  # TODO: Maybe replace non-existent pins with `N/A` instead of `nan` or `0`?
  def get_fmt_pin(pin):
    return _fmt_io_bank_pin((io_container.get_int(bank, pin), io_container.get_float(bank, pin)))

  return desc +\
    '  Pin 1:\n{0}\n'.format(get_fmt_pin(1)) +\
    '  Pin 2:\n{0}\n'.format(get_fmt_pin(2)) +\
    '  Pin 3:\n{0}\n'.format(get_fmt_pin(3)) +\
    '  Pin 4:\n{0}\n'.format(get_fmt_pin(4)) +\
    '  Pin 5:\n{0}\n'.format(get_fmt_pin(5)) +\
    '  Pin 6:\n{0}\n'.format(get_fmt_pin(6)) +\
    '  Pin 7:\n{0}\n'.format(get_fmt_pin(7)) +\
    '  Pin 8:\n{0}\n'.format(get_fmt_pin(8))


def _io_repr(io):
  try:
    container = io._get_ref()
    return 'IO Banks: [A, B, C, D, E, F]\n' +\
      '{0}\n{1}\n{2}\n{3}\n{4}\n{5}'.format(io.a, io.b, io.c, io.d, io.e, io.f)
  except:
    return 'IO Banks: [A, B, C, D, E, F]\n  <Group message was finalized>'


def _led_repr(led, led_field):
  try:
    enum_name = led_field.name
  except:
    enum_name = led_field
  desc = 'LED (Enumeration {0}):\n'.format(enum_name)
  try:
    container = led._get_ref()
    colors = led.color
    return desc + '  [' + ', '.join([repr(color) for color in colors]) + ']'
  except:
    return desc + '  <Group message was finalized>'


# -----------------------------------------------------------------------------
# Debug Field Containers
# -----------------------------------------------------------------------------


def _get_numbered_floats(getter, group_message, field, number):
  modules = group_message.modules
  ret = np.empty(len(modules), np.float32)
  tmp = c_float()
  for i, module in enumerate(modules):
    if getter(module, field, number, byref(tmp)) != StatusSuccess:
      ret[i] = np.nan
    else:
      ret[i] = tmp.value
  return ret


def _set_numbered_floats(setter, group_message, field, number, val):
  modules = group_message.modules
  tmp = c_float()
  values = _do_broadcast(group_message, field, val, numpy.float32)
  for i, module in enumerate(modules):
    tmp.value = values[i]
    setter(module, field, number, byref(tmp))


class GroupNumberedFloatFieldContainer(WeakReferenceContainer):
  """
  A read only view into a set of numbered float fields
  """

  __slots__ = ['__get_numbered_float', '_field']

  def __init__(self, internal, message_str, field):
    super(GroupNumberedFloatFieldContainer, self).__init__(internal)
    from . import raw

    get_func = getattr(raw, 'hebi{0}GetNumberedFloat'.format(message_str))
    
    def get(number):
      return _get_numbered_floats(get_func, self._get_ref(), field, number)

    self._field = field
    self.__get_numbered_float = get

  def __repr__(self):
    return _numbered_float_repr(self, self._field)

  @property
  def float1(self):
    return self.__get_numbered_float(1)

  @property
  def float2(self):
    return self.__get_numbered_float(2)

  @property
  def float3(self):
    return self.__get_numbered_float(3)

  @property
  def float4(self):
    return self.__get_numbered_float(4)

  @property
  def float5(self):
    return self.__get_numbered_float(5)

  @property
  def float6(self):
    return self.__get_numbered_float(6)

  @property
  def float7(self):
    return self.__get_numbered_float(7)

  @property
  def float8(self):
    return self.__get_numbered_float(8)

  @property
  def float9(self):
    return self.__get_numbered_float(9)


class MutableGroupNumberedFloatFieldContainer(WeakReferenceContainer):
  """
  A mutable view into a set of numbered float fields
  """

  __slots__ = ['__get_numbered_float', '__set_numbered_float', '_field']

  def __init__(self, internal, message_str, field):
    super(MutableGroupNumberedFloatFieldContainer, self).__init__(internal)
    from . import raw
    
    get_func = getattr(raw, 'hebi{0}GetNumberedFloat'.format(message_str))
    set_func = getattr(raw, 'hebi{0}SetNumberedFloat'.format(message_str))
    
    def setter(number, value):
      _set_numbered_floats(set_func, self._get_ref(), field, number, value)
    def get(number):
      return _get_numbered_floats(get_func, self._get_ref(), field, number)

    self._field = field
    self.__get_numbered_float = get
    self.__set_numbered_float = setter

  def __repr__(self):
    return _numbered_float_repr(self, self._field)

  @property
  def float1(self):
    return self.__get_numbered_float(1)

  @property
  def float2(self):
    return self.__get_numbered_float(2)

  @property
  def float3(self):
    return self.__get_numbered_float(3)

  @property
  def float4(self):
    return self.__get_numbered_float(4)

  @property
  def float5(self):
    return self.__get_numbered_float(5)

  @property
  def float6(self):
    return self.__get_numbered_float(6)

  @property
  def float7(self):
    return self.__get_numbered_float(7)

  @property
  def float8(self):
    return self.__get_numbered_float(8)

  @property
  def float9(self):
    return self.__get_numbered_float(9)

  @float1.setter
  def float1(self, value):
    self.__set_numbered_float(1, value)

  @float2.setter
  def float2(self, value):
    self.__set_numbered_float(2, value)

  @float3.setter
  def float3(self, value):
    self.__set_numbered_float(3, value)

  @float4.setter
  def float4(self, value):
    self.__set_numbered_float(4, value)

  @float5.setter
  def float5(self, value):
    self.__set_numbered_float(5, value)

  @float6.setter
  def float6(self, value):
    self.__set_numbered_float(6, value)

  @float7.setter
  def float7(self, value):
    self.__set_numbered_float(7, value)

  @float8.setter
  def float8(self, value):
    self.__set_numbered_float(8, value)

  @float9.setter
  def float9(self, value):
    self.__set_numbered_float(9, value)


# -----------------------------------------------------------------------------
# IoField classes
# -----------------------------------------------------------------------------


class GroupMessageIoFieldBankContainer(WeakReferenceContainer):
  """
  Represents a read only IO bank
  """

  __slots__ = ['_bank', '_bank_readable']

  def __init__(self, bank, bank_readable, io_field_container):
    super(GroupMessageIoFieldBankContainer, self).__init__(io_field_container)
    self._bank = bank
    self._bank_readable = bank_readable.strip().upper()

  def __repr__(self):
    return _io_bank_repr(self, self._bank, self._bank_readable)

  def has_int(self, pin_number):
    container = self._get_ref()
    return container.has_int(self._bank, pin_number)

  def has_float(self, pin_number):
    container = self._get_ref()
    return container.has_float(self._bank, pin_number)

  def get_int(self, pin_number):
    container = self._get_ref()
    return container.get_int(self._bank, pin_number)

  def get_float(self, pin_number):
    container = self._get_ref()
    return container.get_float(self._bank, pin_number)


class MutableGroupMessageIoFieldBankContainer(WeakReferenceContainer):
  """
  Represents a mutable IO Bank
  """

  __slots__ = ['_bank', '_bank_readable']

  def __init__(self, bank, bank_readable, io_field_container):
    super(MutableGroupMessageIoFieldBankContainer, self).__init__(io_field_container)
    self._bank = bank
    self._bank_readable = bank_readable.strip().upper()

  def __repr__(self):
    return _io_bank_repr(self, self._bank, self._bank_readable)

  def has_int(self, pin_number):
    return self._get_ref().has_int(self._bank, pin_number)

  def has_float(self, pin_number):
    return self._get_ref().has_float(self._bank, pin_number)

  def get_int(self, pin_number):
    return self._get_ref().get_int(self._bank, pin_number)

  def get_float(self, pin_number):
    return self._get_ref().get_float(self._bank, pin_number)

  def set_int(self, pin_number, value):
    return self._get_ref().set_int(self._bank, pin_number, value)

  def set_float(self, pin_number, value):
    return self._get_ref().set_float(self._bank, pin_number, value)


class GroupMessageIoFieldContainer(WeakReferenceContainer):
  """
  Represents a read only view into IO banks
  """

  __slots__ = ['_a', '_b', '_c', '_d', '_e', '_f', '__get_float', '__get_int', '__weakref__']

  def __init__(self, group_message, message_str):
    super(GroupMessageIoFieldContainer, self).__init__(group_message)
    from . import raw
    self.__get_int = getattr(raw, 'hebi{0}GetIoPinInt'.format(message_str))
    self.__get_float = getattr(raw, 'hebi{0}GetIoPinFloat'.format(message_str))

    bank_a = getattr(raw, '{0}IoBankA'.format(message_str))
    bank_b = getattr(raw, '{0}IoBankB'.format(message_str))
    bank_c = getattr(raw, '{0}IoBankC'.format(message_str))
    bank_d = getattr(raw, '{0}IoBankD'.format(message_str))
    bank_e = getattr(raw, '{0}IoBankE'.format(message_str))
    bank_f = getattr(raw, '{0}IoBankF'.format(message_str))
    self._a = GroupMessageIoFieldBankContainer(bank_a, 'a', self)
    self._b = GroupMessageIoFieldBankContainer(bank_b, 'b', self)
    self._c = GroupMessageIoFieldBankContainer(bank_c, 'c', self)
    self._d = GroupMessageIoFieldBankContainer(bank_d, 'd', self)
    self._e = GroupMessageIoFieldBankContainer(bank_e, 'e', self)
    self._f = GroupMessageIoFieldBankContainer(bank_f, 'f', self)

  def __repr__(self):
    return _io_repr(self)

  def has_int(self, bank, pin_number):
    group_message = self._get_ref()
    ret = numpy.empty(group_message.size, bool)
    pin_number = c_size_t(pin_number)
    for i, message in enumerate(group_message.modules):
      ret[i] = self.__get_int(message, bank, pin_number, None) == StatusSuccess
    return ret

  def has_float(self, bank, pin_number):
    group_message = self._get_ref()
    ret = numpy.empty(group_message.size, bool)
    pin_number = c_size_t(pin_number)
    for i, message in enumerate(group_message.modules):
      ret[i] = self.__get_float(message, bank, pin_number, None) == StatusSuccess
    return ret

  def get_int(self, bank, pin_number):
    group_message = self._get_ref()
    ret = numpy.empty(group_message.size, numpy.int64)
    pin_number = c_size_t(pin_number)
    tmp = c_int64(0)
    for i, message in enumerate(group_message.modules):
      if self.__get_int(message, bank, pin_number, byref(tmp)) != StatusSuccess:
        ret[i] = 0
      else:
        ret[i] = tmp.value
    return ret

  def get_float(self, bank, pin_number):
    group_message = self._get_ref()
    ret = numpy.empty(group_message.size, numpy.float32)
    pin_number = c_size_t(pin_number)
    tmp = c_float(0.0)
    for i, message in enumerate(group_message.modules):
      if self.__get_float(message, bank, pin_number, byref(tmp)) != StatusSuccess:
        ret[i] = numpy.nan
      else:
        ret[i] = tmp.value
    return ret

  @property
  def a(self):
    return self._a

  @property
  def b(self):
    return self._b

  @property
  def c(self):
    return self._c

  @property
  def d(self):
    return self._d

  @property
  def e(self):
    return self._e

  @property
  def f(self):
    return self._f


class MutableGroupMessageIoFieldContainer(WeakReferenceContainer):
  """
  Represents a mutable view into IO banks
  """

  __slots__ = ['_a', '_b', '_c', '_d', '_e', '_f', '__get_float', '__get_int', '__set_float', '__set_int', '__weakref__']

  def __init__(self, group_message, message_str):
    super(MutableGroupMessageIoFieldContainer, self).__init__(group_message)
    from . import raw
    self.__get_int = getattr(raw, 'hebi{0}GetIoPinInt'.format(message_str))
    self.__get_float = getattr(raw, 'hebi{0}GetIoPinFloat'.format(message_str))
    c_set_int = getattr(raw, 'hebi{0}SetIoPinInt'.format(message_str))
    c_set_float = getattr(raw, 'hebi{0}SetIoPinFloat'.format(message_str))

    def seti(bank, pin_number, value):
      group_message = self._get_ref()
      pin_number = c_size_t(pin_number)
      value = _do_broadcast(group_message, bank, value, numpy.int64)
      tmp = c_int64(0)
      for i, message in enumerate(group_message.modules):
        tmp.value = value[i]
        c_set_int(message, bank, pin_number, byref(tmp))

    def setf(bank, pin_number, value):
      group_message = self._get_ref()
      pin_number = c_size_t(pin_number)
      value = _do_broadcast(group_message, bank, value, numpy.float32)
      tmp = c_float(0.0)
      for i, message in enumerate(group_message.modules):
        tmp.value = value[i]
        c_set_float(message, bank, pin_number, byref(tmp))


    self.__set_int = seti
    self.__set_float = setf

    bank_a = getattr(raw, '{0}IoBankA'.format(message_str))
    bank_b = getattr(raw, '{0}IoBankB'.format(message_str))
    bank_c = getattr(raw, '{0}IoBankC'.format(message_str))
    bank_d = getattr(raw, '{0}IoBankD'.format(message_str))
    bank_e = getattr(raw, '{0}IoBankE'.format(message_str))
    bank_f = getattr(raw, '{0}IoBankF'.format(message_str))

    self._a = MutableGroupMessageIoFieldBankContainer(bank_a, 'a', self)
    self._b = MutableGroupMessageIoFieldBankContainer(bank_b, 'b', self)
    self._c = MutableGroupMessageIoFieldBankContainer(bank_c, 'c', self)
    self._d = MutableGroupMessageIoFieldBankContainer(bank_d, 'd', self)
    self._e = MutableGroupMessageIoFieldBankContainer(bank_e, 'e', self)
    self._f = MutableGroupMessageIoFieldBankContainer(bank_f, 'f', self)

  def __repr__(self):
    return _io_repr(self)

  def set_int(self, bank, pin_number, value):
    self.__set_int(bank, pin_number, value)

  def set_float(self, bank, pin_number, value):
    self.__set_float(bank, pin_number, value)

  def has_int(self, bank, pin_number):
    group_message = self._get_ref()
    ret = numpy.empty(group_message.size, bool)
    pin_number = c_size_t(pin_number)
    for i, message in enumerate(group_message.modules):
      ret[i] = self.__get_int(message, bank, pin_number, None) == StatusSuccess
    return ret

  def has_float(self, bank, pin_number):
    group_message = self._get_ref()
    ret = numpy.empty(group_message.size, bool)
    pin_number = c_size_t(pin_number)
    for i, message in enumerate(group_message.modules):
      ret[i] = self.__get_float(message, bank, pin_number, None) == StatusSuccess
    return ret

  def get_int(self, bank, pin_number):
    group_message = self._get_ref()
    ret = numpy.empty(group_message.size, numpy.int64)
    pin_number = c_size_t(pin_number)
    tmp = c_int64(0)
    for i, message in enumerate(group_message.modules):
      if self.__get_int(message, bank, pin_number, byref(tmp)) != StatusSuccess:
        ret[i] = 0
      else:
        ret[i] = tmp.value
    return ret

  def get_float(self, bank, pin_number):
    group_message = self._get_ref()
    ret = numpy.empty(group_message.size, numpy.float32)
    pin_number = c_size_t(pin_number)
    tmp = c_float(0.0)
    for i, message in enumerate(group_message.modules):
      if self.__get_float(message, bank, pin_number, byref(tmp)) != StatusSuccess:
        ret[i] = numpy.nan
      else:
        ret[i] = tmp.value
    return ret

  @property
  def a(self):
    return self._a

  @property
  def b(self):
    return self._b

  @property
  def c(self):
    return self._c

  @property
  def d(self):
    return self._d

  @property
  def e(self):
    return self._e

  @property
  def f(self):
    return self._f


# -----------------------------------------------------------------------------
# LED Field Containers
# -----------------------------------------------------------------------------


class GroupMessageLEDFieldContainer(WeakReferenceContainer):

  __slots__ = ['__get_led_color', '_field']

  def __init__(self, internal, message_str, field):
    super(GroupMessageLEDFieldContainer, self).__init__(internal)
    from . import raw
    self.__get_led_color = getattr(raw, 'hebi{0}GetLedColor'.format(message_str))
    self._field = field

  def __repr__(self):
    return _led_repr(self, self._field)

  def __call_get_led_color(self):
    group_message = self._get_ref()
    r = c_uint8(0)
    g = c_uint8(0)
    b = c_uint8(0)
    ret = [Color(0, 0, 0)] * group_message.size

    for i, message in enumerate(group_message.modules):
      if (self.__get_led_color(message, self._field, byref(r), byref(g), byref(b)) != StatusSuccess):
        # TODO: handle error
        pass
      color = ret[i]
      color.r = r.value
      color.g = g.value
      color.b = b.value
      ret[i] = color

    return ret

  @property
  def color(self):
    return self.__call_get_led_color()


class MutableGroupMessageLEDFieldContainer(GroupMessageLEDFieldContainer):
  __slots__ = ['__has_led_module_control', '__set_led_override_color', '__set_led_module_control', '__clear_led']

  def __init__(self, internal, message_str, field):
    super(MutableGroupMessageLEDFieldContainer, self).__init__(internal, message_str, field)
    from . import raw
    self.__has_led_module_control = getattr(raw, 'hebi{0}HasLedModuleControl'.format(message_str))
    self.__set_led_override_color = getattr(raw, 'hebi{0}SetLedOverrideColor'.format(message_str))
    self.__set_led_module_control = getattr(raw, 'hebi{0}SetLedModuleControl'.format(message_str))
    self.__clear_led = getattr(raw, 'hebi{0}ClearLed'.format(message_str))

  def _clear_all_leds(self):
    group_message = self._get_ref()
    for i, message in enumerate(group_message.modules):
      self.__clear_led(message, self._field)

  def _clear_led(self, index):
    message = self._get_ref().modules[index]
    self.__clear_led(message, self._field)

  def _set_all_led_module_controls(self):
    group_message = self._get_ref()
    for i, message in enumerate(group_message.modules):
      self.__set_led_module_control(message, self._field)

  def _set_led_module_control(self, index):
    message = self._get_ref().modules[index]
    self.__set_led_module_control(message, self._field)

  def _get_has_led_module_control(self, index):
    message = self._get_ref().modules[index]
    return bool(self.__has_led_module_control(message, self._field))

  def _set_led_color(self, index, color):
    message = self._get_ref().modules[index]
    r = c_uint8(color.r)
    g = c_uint8(color.g)
    b = c_uint8(color.b)
    self.__set_led_override_color(message, self._field, r, g, b)

  def __set_colors(self, colors):
    if colors is None:
      self._clear_all_leds()
      return
    elif type(colors) is str:
      colors = [string_to_color(colors)] * self._get_ref().size
    elif type(colors) is int:
      colors = [color_from_int(colors)] * self._get_ref().size
    elif isinstance(colors, Color):
      colors = [colors] * self._get_ref().size
    elif not (hasattr(colors, '__len__')):
      raise ValueError('Cannot broadcast input to array of colors')

    for i, color in enumerate(colors):
      if color is None:
        self._clear_led(i)
      elif color.a == 0:
        """ Let module have control of LED color """
        self._set_led_module_control(i)
      elif color.a == 255:
        """ Set LED override color without alpha blending (opaque color) """
        self._set_led_color(i, color)
      else:
        """ Do alpha blending """
        # For now, just set led color without blending :(
        self._set_led_color(i, color)

  @property
  def color(self):
    return super(MutableGroupMessageLEDFieldContainer, self).color

  @color.setter
  def color(self, value):
    self.__set_colors(value)


# -----------------------------------------------------------------------------
# Helper Functions
# -----------------------------------------------------------------------------


def __check_broadcastable(group_type, field, value):
  if group_type.size > 1:
    if not field.allow_broadcast:
      raise ValueError('Cannot broadcast scalar value \'{0}\' '.format(value) +
                       'to the field \'{0}\' '.format(field.name) +
                       'in all modules of the group.' +
                       '\nReason: {0}'.format(field.not_broadcastable_reason))


def _do_broadcast(group_message, field, value, dtype):
  if hasattr(value, '__len__') and type(value) is not str:
    __assert_same_length(group_message, value)
    if dtype is str:
      return value
    else:
      return numpy.array(value, dtype=dtype)
  else: # Is scalar; each module in group will be set to this
    __check_broadcastable(group_message, field, value)
    return numpy.array([value] * group_message.size, dtype=dtype)


def __assert_same_length(group_message, b):
  if group_message.size != len(b):
    raise ValueError('Input array must have same size as number of modules in group message')


# -----------------------------------------------------------------------------
# Accessor Functions
# -----------------------------------------------------------------------------


import threading
class MessagesTLS(threading.local):

  __slots__ = ['c_int32', 'c_uint8', 'c_int64', 'c_uint64', 'c_size_t', 'c_float', 'c_vector3f', 'c_quaternionf',
               'c_null_str', 'c_str']

  def __init__(self):
    super(MessagesTLS, self).__init__()
    self.c_int32 = c_int32(0)
    self.c_uint8 = c_uint8(0)
    self.c_int64 = c_int64(0)
    self.c_uint64 = c_uint64(0)
    self.c_size_t = c_size_t(0)
    self.c_float = c_float(0)
    self.c_vector3f = type_utils.create_float_buffer(3)
    self.c_quaternionf = type_utils.create_float_buffer(4)
    self.c_null_str = c_char_p(None)
    self.c_str = create_str(512)


_tls = MessagesTLS()


def __get_flag(group_message, field, getter=None, output=None):
  if output is None:
    ret = numpy.empty(group_message.size, numpy.bool)
  else:
    ret = output
  for i, message in enumerate(group_message.modules):
    ret[i] = bool(getter(message, field))
  return ret


def __get_bool(group_message, field, getter=None, output=None):
  if output is None:
    ret = numpy.empty(group_message.size, numpy.bool)
  else:
    ret = output
  tmp = _tls.c_int32
  for i, message in enumerate(group_message.modules):
    if getter(message, field, byref(tmp)) != StatusSuccess:
      ret[i] = False
    else:
      ret[i] = bool(tmp.value)
  return ret


def __get_uint8(group_message, field, getter=None, output=None):
  if output is None:
    ret = numpy.empty(group_message.size, numpy.uint8)
  else:
    ret = output
  tmp = _tls.c_uint8
  for i, message in enumerate(group_message.modules):
    if getter(message, field, byref(tmp)) != StatusSuccess:
      ret[i] = 0
    else:
      ret[i] = tmp.value
  return ret


def __get_int32(group_message, field, getter=None, output=None):
  if output is None:
    ret = numpy.empty(group_message.size, numpy.int32)
  else:
    ret = output
  tmp = _tls.c_int32
  for i, message in enumerate(group_message.modules):
    if getter(message, field, byref(tmp)) != StatusSuccess:
      ret[i] = 0
    else:
      ret[i] = tmp.value
  return ret


def __get_uint64(group_message, field, getter=None, output=None):
  if output is None:
    ret = numpy.empty(group_message.size, numpy.uint64)
  else:
    ret = output
  tmp = _tls.c_uint64
  for i, message in enumerate(group_message.modules):
    if getter(message, field, byref(tmp)) != StatusSuccess:
      ret[i] = 0
    else:
      ret[i] = tmp.value
  return ret


def __get_float(group_message, field, getter=None, output=None):
  if output is None:
    ret = numpy.empty(group_message.size, numpy.float32)
  else:
    ret = output
  tmp = _tls.c_float
  for i, message in enumerate(group_message.modules):
    if getter(message, field, byref(tmp)) != StatusSuccess:
      ret[i] = numpy.nan
    else:
      ret[i] = tmp.value
  return ret


def __get_highresangle(group_message, field, getter=None, output=None):
  if output is None:
    ret = numpy.empty(group_message.size, numpy.float64)
  else:
    ret = output
  revolutions = _tls.c_int64
  radian_offset = _tls.c_float
  for i, message in enumerate(group_message.modules):
    if getter(message, field, byref(revolutions), byref(radian_offset)) != StatusSuccess:
      ret[i] = numpy.nan
    else:
      ret[i] = revolutions.value*two_pi + radian_offset.value
  return ret


def __get_vector_3f(group_message, field, getter=None, output=None):
  if output is None:
    ret = numpy.empty((group_message.size, 3), numpy.float64)
  else:
    ret = output
  ptr = _tls.c_vector3f
  for i, message in enumerate(group_message.modules):
    if getter(message, field, ptr) != StatusSuccess:
      ret[i, 0:3] = numpy.nan
    else:
      ret[i, 0:3] = ptr
  return ret


def __get_quaternionf(group_message, field, getter=None, output=None):
  if output is None:
    ret = numpy.empty((group_message.size, 4), numpy.float64)
  else:
    ret = output
  ptr = _tls.c_quaternionf
  for i, message in enumerate(group_message.modules):
    if getter(message, field, ptr) != StatusSuccess:
      ret[i, 0:4] = numpy.nan
    else:
      ret[i, 0:4] = ptr
  return ret


def __get_string(group_message, field, getter=None, output=None):
  alloc_size_c = _tls.c_size_t
  alloc_size = 0
  null_str = _tls.c_null_str

  for i, message in enumerate(group_message.modules):
    res = getter(message, field, null_str, byref(alloc_size_c))
    alloc_size = max(alloc_size, alloc_size_c.value + 1)

  if output is not None:
    ret = output
  else:
    ret = [None] * group_message.size

  if alloc_size > len(_tls.c_str):
    string_buffer = create_string_buffer(alloc_size)
  else:
    string_buffer = _tls.c_str

  for i, message in enumerate(group_message.modules):
    alloc_size_c.value = alloc_size
    if getter(message, field, string_buffer, byref(alloc_size_c)) == StatusSuccess:
      ret[i] = decode_str(string_buffer.value)
    else:
      ret[i] = None
  return ret


# -----------------------------------------------------------------------------
# Mutator Functions
# -----------------------------------------------------------------------------


nan = float('nan')
inv_half_pi = 0.5/math.pi
two_pi = 2.0*math.pi


def __set_flag(group_message, field, value, setter=None):
  value = _do_broadcast(group_message, field, value, numpy.bool)
  val = _tls.c_int32
  for i, message in enumerate(group_message.modules):
    val.value = int(value[i])
    setter(message, field, val)


def __set_bool(group_message, field, value, setter=None):
  value = _do_broadcast(group_message, field, value, numpy.bool)
  tmp = _tls.c_int32
  for i, message in enumerate(group_message.modules):
    tmp.value = value[i]
    setter(message, field, byref(tmp))


def __set_uint8(group_message, field, value, setter=None):
  value = _do_broadcast(group_message, field, value, numpy.uint8)
  tmp = _tls.c_uint8
  for i, message in enumerate(group_message.modules):
    tmp.value = value[i]
    setter(message, field, byref(tmp))


def __set_int32(group_message, field, value, setter=None):
  value = _do_broadcast(group_message, field, value, numpy.int32)
  tmp = _tls.c_int32
  for i, message in enumerate(group_message.modules):
    tmp.value = value[i]
    setter(message, field, byref(tmp))


def __set_uint64(group_message, field, value, setter=None):
  value = _do_broadcast(group_message, field, value, numpy.uint64)
  tmp = _tls.c_uint64
  for i, message in enumerate(group_message.modules):
    tmp.value = value[i]
    setter(message, field, byref(tmp))


def __set_float(group_message, field, value, setter=None):
  value = _do_broadcast(group_message, field, value, numpy.float32)
  tmp = _tls.c_float
  for i, message in enumerate(group_message.modules):
    tmp.value = value[i]
    setter(message, field, byref(tmp))


def __set_highresangle(group_message, field, value, setter=None):
  value = _do_broadcast(group_message, field, value, numpy.float64)
  revolutions = _tls.c_int64
  offset = _tls.c_float
  for i, message in enumerate(group_message.modules):
    radians = value[i]
    if math.isnan(radians):
      revolutions.value = 0
      offset.value = nan
    else:
      revolutions_raw = radians * inv_half_pi
      offset.value, rev_d = math.modf(revolutions_raw)
      revolutions.value = int(rev_d)
      offset.value = offset.value * two_pi
    setter(message, field, byref(revolutions), byref(offset))


def __set_string(group_message, field, value, setter=None):
  value = _do_broadcast(group_message, field, value, str)
  alloc_size_c = _tls.c_size_t

  for i, message in enumerate(group_message.modules):
    val = value[i]
    alloc_size = len(val) + 1
    # TODO: use tls string buffer and copy val into it instead
    string_buffer = type_utils.create_string_buffer_compat(val, size=alloc_size)
    alloc_size_c.value = alloc_size
    setter(message, field, string_buffer, byref(alloc_size_c))


# -----------------------------------------------------------------------------
# Command
# -----------------------------------------------------------------------------


get_group_command_flag = funpart(__get_flag, getter=hebiCommandGetFlag)
get_group_command_bool = funpart(__get_bool, getter=hebiCommandGetBool)
get_group_command_enum = funpart(__get_int32, getter=hebiCommandGetEnum)
get_group_command_float = funpart(__get_float, getter=hebiCommandGetFloat)
get_group_command_highresangle = funpart(__get_highresangle, getter=hebiCommandGetHighResAngle)
get_group_command_string = funpart(__get_string, getter=hebiCommandGetString)

set_group_command_flag = funpart(__set_flag, setter=hebiCommandSetFlag)
set_group_command_bool = funpart(__set_bool, setter=hebiCommandSetBool)
set_group_command_enum = funpart(__set_int32, setter=hebiCommandSetEnum)
set_group_command_float = funpart(__set_float, setter=hebiCommandSetFloat)
set_group_command_highresangle = funpart(__set_highresangle, setter=hebiCommandSetHighResAngle)
set_group_command_string = funpart(__set_string, setter=hebiCommandSetString)


# -----------------------------------------------------------------------------
# Feedback
# -----------------------------------------------------------------------------


get_group_feedback_vector3f = funpart(__get_vector_3f, getter=hebiFeedbackGetVector3f)
get_group_feedback_quaternionf = funpart(__get_quaternionf, getter=hebiFeedbackGetQuaternionf)
get_group_feedback_uint64 = funpart(__get_uint64, getter=hebiFeedbackGetUInt64)
get_group_feedback_enum = funpart(__get_int32, getter=hebiFeedbackGetEnum)
get_group_feedback_float = funpart(__get_float, getter=hebiFeedbackGetFloat)
get_group_feedback_highresangle = funpart(__get_highresangle, getter=hebiFeedbackGetHighResAngle)


# -----------------------------------------------------------------------------
# Info
# -----------------------------------------------------------------------------


get_group_info_flag = funpart(__get_flag, getter=hebiInfoGetFlag)
get_group_info_enum = funpart(__get_int32, getter=hebiInfoGetEnum)
get_group_info_bool = funpart(__get_bool, getter=hebiInfoGetBool)
get_group_info_float = funpart(__get_float, getter=hebiInfoGetFloat)
get_group_info_highresangle = funpart(__get_highresangle, getter=hebiInfoGetHighResAngle)
get_group_info_string = funpart(__get_string, getter=hebiInfoGetString)


# -----------------------------------------------------------------------------
# Parsers
# -----------------------------------------------------------------------------


def __map_input_setter_delegate(group_message, values, setter, setter_field):
  mapped_values = [None] * group_message.size
  str_map = setter_field.substrates

  try:
    if type(values) is str:
      val = str_map[values.lower()]
      for i in range(0, group_message.size):
        mapped_values[i] = val
    else:
      for i, entry in enumerate(values):
        mapped_values[i] = str_map[entry.lower()]
  except KeyError as key:
    print('{0} is not a valid string parameter.'.format(key))
    print('Valid string parameters: {0}'.format("'" + "', '".join(setter_field.substrates.keys()) + "'"))
    raise
  setter(group_message, setter_field, mapped_values)


def setter_input_parser_delegate(group_message, values, setter, setter_field):
  """
  Maps strings (case insensitive) to a non-string type.
  Only used for fields which are not of type string

  This function assumes that `setter_field` has an attribute called `substrates`
  which returns a dictionary with :type str: keys and values of the type which
  the function `setter` expects
  """
  if not hasattr(setter_field, 'substrates'):
    raise RuntimeError('Field {0} has no substrates map field'.format(setter_field))
  if hasattr(values, '__len__') and type(values) is not str:  # Is "array-like" according to numpy
    for entry in values:
      if type(entry) is not str:
        # By default, delegate to regular `setter` routine
        setter(group_message, setter_field, values)
        return
    __map_input_setter_delegate(group_message, values, setter, setter_field)
  elif type(values) == str:
    __map_input_setter_delegate(group_message, values, setter, setter_field)
  else:
    setter(group_message, setter_field, values)

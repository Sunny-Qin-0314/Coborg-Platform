# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
#
#  HEBI Core python API - Copyright 2018 HEBI Robotics
#  See https://hebi.us/softwarelicense for license details
#
# -----------------------------------------------------------------------------


class Color(object):
  """
  Accepts integers in [0, 255] or floats in [0.0, 1.0] as input.
  """

  __slots__ = ['__a', '__b', '__g', '__r']

  def __init__(self, r, g, b, a=1.0):
    self.__r = self.__convert_input(r)
    self.__g = self.__convert_input(g)
    self.__b = self.__convert_input(b)
    self.__a = self.__convert_input(a)

  def __repr__(self):
    return 'Color(R:{0}, G:{1}, B:{2}, A:{3})'.format(self.r, self.g, self.b, self.a)

  def __clamp(self, val):
    return max(min(val, 255), 0)

  def __convert_input(self, val):
    if isinstance(val, float):
      return self.__clamp(int(val * 255.0))
    else:
      return self.__clamp(int(val))

  def __int__(self):
    return ((self.__a & 0xff) << 24) | ((self.__r & 0xff) << 16) | ((self.__g & 0xff) << 8) | (self.__b & 0xff)

  @property
  def r(self):
    return self.__r

  @r.setter
  def r(self, value):
    self.__r = self.__convert_input(value)

  @property
  def g(self):
    return self.__g

  @g.setter
  def g(self, value):
    self.__g = self.__convert_input(value)

  @property
  def b(self):
    return self.__b

  @b.setter
  def b(self, value):
    self.__b = self.__convert_input(value)

  @property
  def a(self):
    return self.__a

  @a.setter
  def a(self, value):
    self.__a = self.__convert_input(value)

  @property
  def opaque(self):
      return self.__a == 255

  def alpha_blend(self, dst):
    """
    "This" instance refers to src (foreground)

    :param dst: destination color (background)
    :return: blended color
    """
    src_r = float(self.__r) / 255.0
    src_g = float(self.__g) / 255.0
    src_b = float(self.__b) / 255.0
    src_a = float(self.__a) / 255.0
    dst_r = float(dst.r) / 255.0
    dst_g = float(dst.g) / 255.0
    dst_b = float(dst.b) / 255.0
    dst_a = float(dst.a) / 255.0

    if dst.opaque:
      a = 1.0
      r = (src_r * src_a) + (dst_r * dst_a * (1 - src_a))
      g = (src_g * src_a) + (dst_g * dst_a * (1 - src_a))
      b = (src_b * src_a) + (dst_b * dst_a * (1 - src_a))
    else:
      a = src_a + dst_a * (1.0 - src_a)
      if a == 0.0:
        raise ValueError('both source and destination alphas were 0')
      r = ((src_r * src_a) + (dst_r * dst_a * (1 - src_a))) / a
      g = ((src_g * src_a) + (dst_g * dst_a * (1 - src_a))) / a
      b = ((src_b * src_a) + (dst_b * dst_a * (1 - src_a))) / a
    return Color(r, g, b, a)


def color_from_int(color):
  color= int(color)
  A = (color >> 24) & 0xff
  R = (color >> 16) & 0xff
  G = (color >> 8) & 0xff
  B = color & 0xff
  return Color(R, G, B, A)


# TODO - see if there is any built in way to do this

def string_to_color(color):
  try:
    return color_from_int(color)
  except:
    o_color = color
    color = str(color).strip().lower()
    color = __str_colors.get(color, None)
    if color:
      return color
    raise ValueError('Cannot parse \'{0}\' to color'.format(o_color))


__str_colors = {
  'red' : Color(255, 0, 0),
  'green' : Color(0, 255, 0),
  'blue' : Color(0, 0, 255),
  'black' : Color(0, 0, 0),
  'white' : Color(255, 255, 255),
  'cyan' : Color(0, 255, 255),
  'magenta' : Color(255, 0, 255),
  'yellow' : Color(255, 255, 0),
  'transparent' : Color(0, 0, 0, 0)}

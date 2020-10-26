# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
#
#  HEBI Core python API - Copyright 2018 HEBI Robotics
#  See https://hebi.us/softwarelicense for license details
#
# -----------------------------------------------------------------------------


import weakref


# -----------------------------------------------------------------------------
# Classes
# -----------------------------------------------------------------------------


class UnmanagedObject(object):
  """
  Base class for an object created by the HEBI C API.
  """

  __slots__ = ['_finalized', '_internal', '_lock', '_on_delete']

  def __init__(self, internal, on_delete=None):
    self._finalized = False
    self._internal = internal
    from threading import Lock
    self._lock = Lock()
    self._on_delete = on_delete

  @property
  def _as_parameter_(self):
    with self._lock:
      if self._finalized:
        import sys
        sys.stderr.write('Warning: Attempting to reference finalized HEBI object\n')
        raise RuntimeError('Object has already been finalized')
      return self._internal

  @property
  def finalized(self):
    with self._lock:
      return self._finalized

  def force_delete(self):
    with self._lock:
      if self._finalized:
        return
      if self._on_delete and self._internal:
        self._on_delete(self._internal)
      self._internal = None
      self._finalized = True

  def __del__(self):
    self.force_delete()


class UnmanagedSharedObject(object):
  """
  Base class for an object from the HEBI C API which requires
  reference counting.
  """

  __slots__ = ['_holder']

  def __init__(self, internal=None, on_delete=None, existing=None):
    class Proxy(object):
      def __init__(self, internal=None, on_delete=None, proxy=None):
        if proxy is not None:
          with proxy._lock:
            if proxy._refcount.count == 0:
              raise RuntimeError('Object has already been finalized')
            self._lock = proxy._lock
            self._internal = proxy._internal
            self._on_delete = proxy._on_delete
            proxy._refcount.increment()
            self._refcount = proxy._refcount
        else:
          from threading import Lock
          self._lock = Lock()
          self._internal = internal
          self._refcount = Counter()
          self._on_delete = on_delete
          
      def __del__(self):
        with self._lock:
          if self._refcount.count > 0:
            # Object has not been explicitly deleted
            self._refcount.decrement()
          if self._refcount.count == 0 and self._on_delete is not None:
            self._on_delete(self._internal)

      def force_delete(self):
        with self._lock:
          if self._refcount.count == 0:
            # Already deleted - return
            return
          self._refcount.clear()
          if self._on_delete is not None:
            self._on_delete(self._internal)
            self._on_delete = None
          self._internal = None

      def get(self):
        with self._lock:
          if self._refcount.count == 0:
            raise RuntimeError('Object has already been finalized')
          return self._internal

    if existing is not None:
      if not isinstance(existing, UnmanagedSharedObject):
        raise TypeError('existing parameter must be an UnmanagedSharedObject instance')
      self._holder = Proxy(proxy=existing._holder)
    else:
      self._holder = Proxy(internal=internal, on_delete=on_delete)

  @property
  def _as_parameter_(self):
    return self._holder.get()


class WeakReferenceContainer(object):
  """
  Small wrapper around a weak reference. For internal use - do not use directly.
  """

  __slots__ = ['_weak_ref']

  def _get_ref(self):
    ref = self._weak_ref()
    if ref is not None:
      return ref
    raise RuntimeError('Reference no longer valid due to finalization')

  def __init__(self, ref):
    self._weak_ref = weakref.ref(ref)


class Counter(object):
  """
  Counter class. For internal use - do not use directly.
  """

  __slots__ = ['_counter']

  def __init__(self):
    self._counter = 1

  def decrement(self):
    self._counter = self._counter - 1
    return self._counter

  def increment(self):
    self._counter = self._counter + 1
    return self._counter

  def clear(self):
    self._counter = 0

  @property
  def count(self):
    return self._counter


class CaseInvariantString(object):
  """
  Represents an immutable string with a custom hash implementation and case invariant comparison
  """

  __slots__ = ['__hash', '__lower_val', '__val']

  def __init__(self, val):
    val = str(val)
    self.__val = val
    self.__lower_val = val.strip().lower()
    self.__hash = hash(self.__lower_val)

  @property
  def value(self):
    return self.__lower_val

  def __hash__(self):
    return self.__hash

  def __eq__(self, other):
    if type(other) is CaseInvariantString:
      return self.__lower_val == other.value
    return str(other).lower() != self.__lower_val

  def __ne__(self, other):
    if type(other) is CaseInvariantString:
      return self.__lower_val != other.value
    return str(other).lower() != self.__lower_val

  def __str__(self):
    return self.__val

  def __repr__(self):
    return self.__val


# -----------------------------------------------------------------------------
# Pretty Strings
# -----------------------------------------------------------------------------


def truncate_with_r_justify(data, length):
  data = str(data)
  data_len = len(data)

  if data_len > length:
    # Truncate the string to [length-3], then add ellipsis
    data = data[0:length-3] + '...'

  fmt_str = '{' + ':<{0}'.format(length) + '}'
  return fmt_str.format(data[0:length])


def lookup_table_string(lookup_entries):
  length = len(lookup_entries)
  if length < 1:
    return 'No modules on network'
  max_module_length = 6
  max_family_length = 16
  max_name_length   = 14
  ret = 'Module  Family            Name          \n'
  ret = ret +\
        '------  ----------------  --------------\n'
  for i, entry in enumerate(lookup_entries):
    module_str = truncate_with_r_justify(i, max_module_length)
    family_str = truncate_with_r_justify(entry.family, max_family_length)
    name_str   = truncate_with_r_justify(entry.name, max_name_length)
    ret = ret +\
      '{0}  {1}  {2}\n'.format(module_str, family_str, name_str)
  return ret


# -----------------------------------------------------------------------------
# Compatibility Layer
# -----------------------------------------------------------------------------

import platform
import sys

__is_pypy = platform.python_implementation().lower() == 'pypy'


def is_pypy():
  return __is_pypy


if sys.version_info[0] == 3:
  def intern_string(s):
    return sys.intern(s)
else:
  def intern_string(s):
    return intern(s)

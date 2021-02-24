# -*- coding: utf-8 -*-
# ------------------------------------------------------------------------------
#
#  HEBI Core python API - Copyright 2018 HEBI Robotics
#  See https://hebi.us/softwarelicense for license details
#
# ------------------------------------------------------------------------------

from . import version
__version__ = str(version.py_version())

# ------------------------------------------------------------------------------
# Lookup API
# ------------------------------------------------------------------------------

from ._internal import lookup as _lookup_internal

class Lookup(object):
  """
  Maintains a registry of network-connected modules
  and returns Group objects to the user.
  """

  DEFAULT_TIMEOUT_MS = 500
  """
  The default timeout (in milliseconds)
  """

  __slots__ = ['__delegate']

  def __init__(self):
    self.__delegate = _lookup_internal.LookupDelegate.get_singleton()

  def __repr__(self):
    lookup_freq = self.lookup_frequency
    # We can't get lookup addresses from C API yet, so nothing to print re: that
    ret = 'lookup_frequency: {0}\n\n'.format(lookup_freq)
    # Mobules table
    from ._internal.utils import lookup_table_string  
    return ret + lookup_table_string([entry for entry in self.entrylist])

  def __str__(self):
    lookup_freq = self.lookup_frequency
    modules = [entry for entry in self.entrylist]
    modules_string = None
    num_modules = len(modules)
    if num_modules > 1:
      modules_string = '{0} modules'.format(num_modules)
    elif num_modules == 1:
      modules_string = '1 module'
    else:
      modules_string = 'no modules'
    return 'Lookup(lookup_frequency={0}; {1} found)'.format(
      lookup_freq, modules_string)

  @property
  def entrylist(self):
    """
    A list of discovered network connected modules.

    :return: The list of modules
    :rtype: EntryList
    """
    return self.__delegate.entrylist

  @property
  def lookup_frequency(self):
    return self.__delegate.lookup_frequency

  @lookup_frequency.setter
  def lookup_frequency(self, freq):
    self.__delegate.lookup_frequency = freq

  def get_group_from_names(self, families, names, timeout_ms=None):
    """
    Get a group from modules with the given names and families.

    If the families or names provided as input is only a single element,
    then that element is assumed to pair with each item in the other parameter.

    This is a blocking call which returns a Group with the given parameters.
    This will time out after :attr:`Lookup.DEFAULT_TIMEOUT_MS` milliseconds,
    if a matching group cannot be constructed.

    :param families:
    :param names:
    :param timeout_ms: The maximum amount of time to wait, in milliseconds.
                       This is an optional parameter with a default value of
                       :attr:`Lookup.DEFAULT_TIMEOUT_MS`.

    :return:
    """
    return self.__delegate.get_group_from_names(families, names, timeout_ms)

  def get_group_from_macs(self, addresses, timeout_ms=None):
    """
    Get a group from modules with the given mac addresses.

    This is a blocking call which returns a Group with the given parameters.
    This will time out after :attr:`Lookup.DEFAULT_TIMEOUT_MS` milliseconds,
    if a matching group cannot be constructed.

    :param addresses:
    :param timeout_ms: The maximum amount of time to wait, in milliseconds.
                       This is an optional parameter with a default value of
                       :attr:`Lookup.DEFAULT_TIMEOUT_MS`.

    :return:
    """
    return self.__delegate.get_group_from_macs(addresses, timeout_ms)

  def get_group_from_family(self, family, timeout_ms=None):
    """
    Get a group from all known modules with the given family.

    This is a blocking call which returns a Group with the given parameters.
    This will time out after :attr:`Lookup.DEFAULT_TIMEOUT_MS` milliseconds,
    if a matching group cannot be constructed.

    :param family:
    :param timeout_ms: The maximum amount of time to wait, in milliseconds.
                       This is an optional parameter with a default value of
                       :attr:`Lookup.DEFAULT_TIMEOUT_MS`.

    :return:
    """
    return self.__delegate.get_group_from_family(family, timeout_ms)

  def get_connected_group_from_name(self, family, name, timeout_ms=None):
    """
    Get a group from all modules known to connect to a module with
    the given name and family.

    This is a blocking call which returns a Group with the given parameters.
    This will time out after :attr:`Lookup.DEFAULT_TIMEOUT_MS` milliseconds,
    if a matching group cannot be constructed.

    :param family:
    :param name:
    :param timeout_ms: The maximum amount of time to wait, in milliseconds.
                       This is an optional parameter with a default value of
                       :attr:`Lookup.DEFAULT_TIMEOUT_MS`.

    :return:
    """
    return self.__delegate.get_connected_group_from_name(family, name, timeout_ms)

  def get_connected_group_from_mac(self, address, timeout_ms=None):
    """
    Get a group from all modules known to connect to a module with
    the given mac address.

    This is a blocking call which returns a Group with the given parameters.
    This will time out after :attr:`Lookup.DEFAULT_TIMEOUT_MS` milliseconds,
    if a matching group cannot be constructed.

    :param address:
    :param timeout_ms: The maximum amount of time to wait, in milliseconds.
                       This is an optional parameter with a default value of
                       :attr:`Lookup.DEFAULT_TIMEOUT_MS`.

    :return:
    """
    return self.__delegate.get_connected_group_from_mac(address, timeout_ms)


# ------------------------------------------------------------------------------
# Message Types
# ------------------------------------------------------------------------------

from ._internal.messages import GroupCommand, GroupFeedback, GroupInfo
from ._internal.graphics import Color
from . import robot_model, trajectory, util

__all__ = ['Lookup', 'GroupCommand', 'GroupFeedback', 'GroupInfo', 'Color',
           'robot_model', 'trajectory', 'util']

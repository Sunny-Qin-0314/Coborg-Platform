def parse_version(s):
  split = s.split('.')
  if len(split) != 3:
    raise ValueError(s)
  return Version(int(split[0]), int(split[1]), int(split[2]))


class Version(object):
  def __init__(self, maj, min, rev):
    self._major_version = maj
    self._minor_version = min
    self._patch_version = rev

  def __str__(self):
    return '{0}.{1}.{2}'.format(self.major_version, self.minor_version, self.patch_version)

  def __repr__(self):
    return 'Version(major: {0}, minor: {1}, patch: {2})'.format(self.major_version, self.minor_version, self.patch_version)

  def __eq__(self, o):
    if isinstance(o, Version):
      return o.major_version == self.major_version and \
      o.minor_version == self.minor_version and \
      o.patch_version == self.patch_version
    else:
      return self == parse_version(o)

  def __ne__(self, o):
    if isinstance(o, Version):
      return o.major_version != self.major_version or \
      o.minor_version != self.minor_version or \
      o.patch_version != self.patch_version
    else:
      return self != parse_version(o)

  def __gt__(self, o):
    if isinstance(o, Version):
      if self.major_version > o.major_version:
        return True
      elif self.major_version < o.major_version:
        return False
      elif self.minor_version > o.minor_version:
        return True
      elif self.minor_version < o.minor_version:
        return False
      elif self.patch_version > o.patch_version:
        return True
      elif self.patch_version < o.patch_version:
        return False
      return self == o
    else:
      return self > parse_version(o)

  def __lt__(self, o):
    if isinstance(o, Version):
      if self.major_version < o.major_version:
        return True
      elif self.major_version > o.major_version:
        return False
      elif self.minor_version < o.minor_version:
        return True
      elif self.minor_version > o.minor_version:
        return False
      elif self.patch_version < o.patch_version:
        return True
      elif self.patch_version > o.patch_version:
        return False
      return self == o
    else:
      return self < parse_version(o)

  @property
  def major_version(self):
    return self._major_version

  @property
  def minor_version(self):
    return self._minor_version

  @property
  def patch_version(self):
    return self._patch_version



def py_version():
  return Version(1,0,1)


def min_c_api_version():
  return Version(1,5,0)


def loaded_c_api_version():
  from ._internal.raw import _handle
  return _handle.version


if __name__ == "__main__":
  import argparse
  def __disp(txt, vfunc):
    print('{0}: {1}'.format(txt, vfunc()))

  parser = argparse.ArgumentParser()
  parser.add_argument("--min-c-api", help="Show minimum required C API version",
                      default=False, action="store_true")
  parser.add_argument("--py-api", help="Show the version of hebi-py",
                      default=False, action="store_true")
  args = parser.parse_args()
  if args.min_c_api:
    __disp('Minimum C API Version', min_c_api_version)
  if args.py_api:
    __disp('hebi-py Version', py_version)

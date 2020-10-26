# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
#
#  HEBI Core python API - Copyright 2018 HEBI Robotics
#  See http://hebi.us/softwarelicense for license details
#
# -----------------------------------------------------------------------------
from os import path
from setuptools import setup, Extension

api_version = '1.0.1'
allow_upload = True

if (api_version == '{0}{1}{2}'.format('#', 'PY_', 'VERSION_INSERT')):
  allow_upload = False
  print('Warning: Version number not generated. Not allowing upload to PyPI.')
  import sys
  if 'upload' in sys.argv:
    print('Error: Uploading to PyPI without generating api_version is explicitly disabled. Exiting.')
    exit(1)

api_reference_url = "http://docs.hebi.us/docs/python/{0}".format(api_version)
changelog_url     = "http://docs.hebi.us/downloads_changelogs.html#python-api-changelog"
documentation_url = "http://docs.hebi.us/tools.html#python-api"
license_url = "https://www.hebirobotics.com/softwarelicense"

description = """
HEBI Core python API
====================

HEBI python provides bindings for the HEBI Core library.

API Reference available at {api_reference_url}

Documentation available on [docs.hebi.us]({documentation_url}).

Note that this is a pre-release. Future versions may break or change APIs with
little or no notice.
Reference the [changelog]({changelog_url}) for more information.

By using this software, you agree to our [software license]({license_url}).
""".format(api_reference_url=api_reference_url,
           documentation_url=documentation_url,
           changelog_url=changelog_url, license_url=license_url)


setup(name                          = 'hebi-py',
      version                       = api_version,
      description                   = 'HEBI Core python bindings',
      long_description              = description,
      long_description_content_type ="text/markdown",
      author                        = 'Daniel Wright',
      author_email                  = 'support@hebirobotics.com',
      url                           = 'https://docs.hebi.us',
      packages                      = ['hebi'],
      package_data                  = {'hebi': [
        'lib/**/libhebi.so*',
        'lib/**/libhebi.*dylib',
        'lib/**/hebi.dll',
        '_internal/*'
      ]},
      install_requires              = [
        'numpy'
      ],
      classifiers                   = [
        "Development Status :: 5 - Production/Stable",
        "Environment :: MacOS X",
        "Intended Audience :: Developers",
        "Operating System :: MacOS",
        "Operating System :: Microsoft :: Windows",
        "Operating System :: POSIX",
        "Operating System :: Unix",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 2",
        "Programming Language :: Python :: Implementation :: CPython",
      ],
      project_urls                  = {
        "API Reference" : api_reference_url,
        "Changelog"     : changelog_url,
        "Documentation" : documentation_url,
        "Examples"      : "https://github.com/HebiRobotics/HEBI-python-examples"
      }
      )

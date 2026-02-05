'''
    This test module imports tests that come with pyfrc, and can be used
    to test basic functionality of just about any robot.
'''

# from pyfrc.tests.docstring_test import test_docstrings

from pyfrc.tests import *

# pyfrc.test run through disabled, teleop, auto, then back to disabled and looks for crash bugs.  Doesn't test inputs or much else.
# We can write additional tests to verify our code still works after changes.

# You can read more about pyfrc.test's here: https://robotpy.readthedocs.io/projects/pyfrc/en/stable/testing.html
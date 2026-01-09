from pathlib import Path
from pyfrc.test_support.pytest_plugin import PyFrcPlugin
from robot import Robot


def pytest_runtest_setup(item):
    item.config.pluginmanager.register(PyFrcPlugin(Robot, Path("robot.py")))

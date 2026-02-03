from pathlib import Path
from pyfrc.test_support.pytest_plugin import PyFrcPlugin
from pytest import Function
from robot import Robot


def pytest_runtest_setup(item: Function) -> None:
    item.config.pluginmanager.register(
        PyFrcPlugin(Robot, Path("robot.py"), isolated=True)
    )

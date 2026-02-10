import pytest
import importlib
import sys
from unittest.mock import MagicMock
from nicegui.testing import UserInterface

# 1. Mock hardware/local dependencies globally
sys.modules['robot_python_code'] = MagicMock()
sys.modules['parameters'] = MagicMock()

# List of labs to test
LABS = [
    ("base_code_lab_01.robot_python_code.lab01_gui"),
]

@pytest.mark.parametrize("module_path", LABS)
@pytest.fixture
def user_interface(ui_run, module_path):
    lab_module = importlib.import_module(module_path)
    lab_module.main_page()

@pytest.mark.parametrize("module_path", LABS)
async def test_gui_elements_render(user_interface: UserInterface, module_path):
    """Check if the critical UI components exist on the page."""
    await user_interface.open('/')
    await user_interface.should_see('ROB-GY - 6213: Robot Navigation & Localization')
    await user_interface.should_see('Data Logging')
    await user_interface.should_see('Robot Connect')
    await user_interface.should_see('SPEED:')

@pytest.mark.parametrize("module_path", LABS)
async def test_speed_interaction(user_interface: UserInterface, module_path):
    await user_interface.open('/')
    await user_interface.click('Enable') 
    assert await user_interface.find('SPEED:') is not None

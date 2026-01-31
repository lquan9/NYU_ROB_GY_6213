import pytest
from nicegui import ui
from nicegui.testing import UserInterface
from unittest.mock import MagicMock
import sys

sys.modules['robot_python_code'] = MagicMock()
sys.modules['parameters'] = MagicMock()

from lab01_gui import main_page

@pytest.fixture
def user_interface(ui_run):
    main_page()

async def test_gui_elements_render(user_interface: UserInterface):
    """Check if the critical UI components exist on the page."""
    await user_interface.open('/')
    
    # Verify the Title
    await user_interface.should_see('ROB-GY - 6213: Robot Navigation & Localization')
    
    # Verify Switches and Labels exist
    await user_interface.should_see('Data Logging')
    await user_interface.should_see('Robot Connect')
    await user_interface.should_see('SPEED:')
    await user_interface.should_see('STEER:')

async def test_speed_slider_interaction(user_interface: UserInterface):
    """Test that toggling the speed switch works."""
    await user_interface.open('/')
    
    # Find the Speed Enable switch and click it
    await user_interface.click('Enable') 
    
    assert await user_interface.find('SPEED:') is not None

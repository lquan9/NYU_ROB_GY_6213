"""__init__.py"""

def __getattr__(name):
    if name in (
        "RobotSensorSignal", "Robot", "CameraSensor",
        "UDPCommunication", "DataLogger", "MsgSender", "MsgReceiver",
    ):
        from .robot import (
            RobotSensorSignal, Robot, CameraSensor,
            UDPCommunication, DataLogger, MsgSender, MsgReceiver,
        )
        return locals()[name]
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
from sr.robot3.motor_board import MotorBoard

class Movement:
    _motor_board: MotorBoard

    @property
    def left_servo_position(self) -> float:
        return self._motor_board.motors[0]
    
    @left_servo_position.setter
    def left_servo_position(self, value: float):
        self._motor_board.motors[0] = value

    @property
    def right_servo_position(self) -> float:
        return self._motor_board.motors[1]

    @right_servo_position.setter
    def right_servo_position(self, value: float):
        self._motor_board.motors[1] = value

    def __init__(self, motor_board: MotorBoard):
        self._motor_board = motor_board
        
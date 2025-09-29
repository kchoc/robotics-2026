from sr.robot3.motor_board import MotorBoard

import asyncio

class Movement:
    _motor_board: MotorBoard

    @property
    def left_servo_power(self) -> float:
        return self._motor_board.motors[0].power
    
    @left_servo_power.setter
    def left_servo_power(self, power: float) -> None:
        self._motor_board.motors[0].power = power

    @property
    def right_servo_power(self) -> float:
        return self._motor_board.motors[1].power
    
    @right_servo_power.setter
    def right_servo_power(self, power: float) -> None:
        self._motor_board.motors[1].power = power

    @property
    def servo_powers(self) -> tuple[float, float]:
        return (self.left_servo_power, self.right_servo_power)

    @servo_powers.setter
    def servo_powers(self, powers: tuple[float, float]) -> None:
        self.left_servo_power, self.right_servo_power = powers

    def power_for_time(self, left_power: float, right_power: float, duration: float) -> None:
        asyncio.run(self._power_for_time_async(left_power, right_power, duration))
        
    async def _power_for_time_async(self, left_power: float, right_power: float, duration: float) -> None:
        self.servo_powers = (left_power, right_power)
        await asyncio.sleep(duration)
        self.stop()

    def __init__(self, motor_board: MotorBoard):
        self._motor_board = motor_board
        
    def stop(self) -> None:
        self.servo_powers = (0.0, 0.0)
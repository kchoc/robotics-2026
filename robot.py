from sr.robot3 import Robot as RB

class Robot(RB):
    def __init__(self, *, debug = False, wait_for_start = True, trace_logging = False, ignored_arduinos = None, manual_boards = None, raw_ports = None, no_powerboard = False):
        super().__init__(debug=debug, wait_for_start=wait_for_start, trace_logging=trace_logging, ignored_arduinos=ignored_arduinos, manual_boards=manual_boards, raw_ports=raw_ports, no_powerboard=no_powerboard)


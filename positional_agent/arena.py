from threading import Thread

from types import MappingProxyType
from typing import TYPE_CHECKING
import numpy as np
import pyopencl as cl
import asyncio
import traceback

from positional_agent.enums import CAMERAS

if TYPE_CHECKING:
    from sr.robot3.camera import AprilCamera as Camera
    from sr.robot3.marker import Marker
    from robot import Robot

class Arena:
    _robot: "Robot"
    _cameras: MappingProxyType[str, "Camera"]
    _box_marker_ready_update: bool = True

    _width: float = 5560  # in mm
    _cam_markers: dict[str, list["Marker"]] = {cam_type: [] for cam_type in CAMERAS.keys()}
    _markers = np.zeros((200, 3), dtype=np.float32) # [x, y, rot]

    def __init__(self, robot: "Robot") -> None:
        self._cameras = robot._robot._cameras
        self._robot = robot
        # self._robot.add_action(self.display_grid, trigger="on_marker_update")
        self._vision_thread = Thread(target=self._run_vision_loop, daemon=True)
        self._vision_thread.start()

    def _run_vision_loop(self) -> None:
        self.ctx = cl.create_some_context()
        self.queue = cl.CommandQueue(self.ctx)
        self.mf = cl.mem_flags

        # Compile the OpenCL kernel once
        with open("./positional_agent/gpu_scripts/update_markers.cl", "r") as f:
            kernel_code = f.read()
        self.program = cl.Program(self.ctx, kernel_code).build()
        self.update_markers = self.program.update_markers

        while True:
            try:
                self._vision()
            except Exception as e:
                print("Error in vision loop!", str(e))
                traceback.print_exc()
                break

    def _vision(self):
        async def see_async(cam_type, cam):
            return cam_type, await asyncio.to_thread(cam.see)

        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        tasks = [see_async(cam_type, cam) for cam_type, cam in self._cameras.items()]
        # Update the compass reading as close to the camera capture as possible
        results = loop.run_until_complete(asyncio.gather(*tasks))
        heading = self._robot.nav.rotation
        loop.close()

        self._cam_markers = {cam_type: markers for cam_type, markers in results}

        if self._robot.nav.update_transform(self._cam_markers, heading):
            self.update_box_markers()

    def update_box_markers(self) -> None:
        marker_data = []
        for marker in self._cam_markers["Camera"]:
            if marker.id >= 21:
                marker_data.append([
                    marker.id,
                    marker.orientation.roll,
                    marker.orientation.pitch,
                    marker.orientation.yaw,
                    marker.position.distance,
                    marker.position.horizontal_angle
                ])
        
        if not marker_data:
            self.box_marker_ready_update = True
            return
        
        marker_data = np.array(marker_data, dtype=np.float32)

        # Create buffers for marker data
        marker_data_buf = cl.Buffer(self.ctx, self.mf.READ_ONLY | self.mf.COPY_HOST_PTR, hostbuf=marker_data)
        markers_buf = cl.Buffer(self.ctx, self.mf.READ_WRITE | self.mf.COPY_HOST_PTR, hostbuf=self._markers)

        # Set kernel arguments
        self.update_markers.set_args(
            marker_data_buf,
            markers_buf,
            np.float32(self._robot.nav._update_rotation),
            np.float32(self._robot.nav._update_position),
        )

        # Execute the kernel
        cl.enqueue_nd_range_kernel(self.queue, self.update_markers, (len(marker_data),), None)
        cl.enqueue_copy(self.queue, self._markers, markers_buf)

        # Trigger actions for marker updates
        asyncio.run(self._robot.trigger_actions("on_marker_update"))
        self.box_marker_ready_update = True        

    def display_grid(self, res=40) -> None:
        if abs(self._robot.nav._update_position[0]) > self._width / 2 or abs(self._robot.nav._update_position[1]) > self._width / 2:
            print("Robot out of bounds, not displaying grid")
            print(f"Position: {self._robot.nav._update_position} Bearing: {self._robot.nav._update_rotation}")
            return
        grid = np.full((res + 2, res + 2), " ", dtype=str)
        grid[0, :] = grid[-1, :] = grid[:, 0] = grid[:, -1] = "#"
        for i in range(21, len(self._markers)):
            if self._markers[i][0] == 0:
                continue

            x = int((self._markers[i][0] + self._width / 2) * res // self._width)
            y = int((self._markers[i][1] + self._width / 2) * res // self._width)
            
            if 1 <= x < res + 1 and 1 <= y < res + 1:
                grid[y, x] = "X"

        grid[int((self._robot.nav._update_position[1] + self._width / 2) * res // self._width),
             int((self._robot.nav._update_position[0] + self._width / 2) * res // self._width)] = "R"
    
        print("\n".join("".join(char * 2 for char in row) for row in grid[::-1]))

        print(f"Position: {self._robot.nav._update_position} Bearing: {self._robot.nav._update_rotation}")

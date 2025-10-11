# revo2_node.py (Humble-safe, no AsyncIOExecutor)
import asyncio
import sys
import time
from threading import Thread
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64

from .revo2_utils import libstark, logger


def _baud_from_param(name: str) -> "libstark.Baudrate":
    try:
        return getattr(libstark.Baudrate, name)
    except AttributeError:
        # return libstark.Baudrate.Baud115200
        return libstark.Baudrate.Baud1Mpbs


class Revo2Node(Node):
    def __init__(self, loop: asyncio.AbstractEventLoop) -> None:
        super().__init__("revo2_controller")
        self._loop = loop

        # ---- Parameters ----
        self.declare_parameter("port", "/dev/ttyUSB1")
        # self.declare_parameter("baudrate", "Baud115200") 
        self.declare_parameter("baudrate", "Baud1Mbps") 
        self.declare_parameter("left_hand_id", 0x7E)
        self.declare_parameter("right_hand_id", 0x7F)
        self.declare_parameter("period", 2.0)
        self.declare_parameter("loop_dt", 0.01)
        self.declare_parameter("speeds", [750, 750, 750, 750, 750, 750])
        self.declare_parameter("positions_a", [0, 0, 0, 0, 0, 0])
        self.declare_parameter("positions_b", [400, 750, 500, 0, 0, 0])
        self.declare_parameter("grasp", [400, 750, 500, 0, 0, 0])
        
        self._current_right = [0, 750, -500, -500, -500, -500]
        self._current_left = [0, 750, -500, -500, -500, -500]

        self._port: str = self.get_parameter("port").get_parameter_value().string_value
        self._baud = _baud_from_param(self.get_parameter("baudrate").get_parameter_value().string_value)
        self._left_hand_id: int = self.get_parameter("left_hand_id").get_parameter_value().integer_value
        self._right_hand_id: int = self.get_parameter("right_hand_id").get_parameter_value().integer_value
        self._period: float = self.get_parameter("period").get_parameter_value().double_value
        self._loop_dt: float = self.get_parameter("loop_dt").get_parameter_value().double_value

        self._speeds: List[int] = list(self.get_parameter("speeds").get_parameter_value().integer_array_value)
        self._pos_a: List[int] = list(self.get_parameter("positions_a").get_parameter_value().integer_array_value)
        self._pos_b: List[int] = list(self.get_parameter("positions_b").get_parameter_value().integer_array_value)

        for name, arr in (("speeds", self._speeds), ("positions_a", self._pos_a), ("positions_b", self._pos_b)):
            if len(arr) != 6:
                raise ValueError(f"Parameter '{name}' must have 6 integers, got {len(arr)}")

        self._client: Optional["libstark.PyDeviceContext"] = None
        self._shutdown_event = asyncio.Event()
        self._run_task: Optional[asyncio.Task] = None

        # allow period / loop_dt to be tweaked at runtime
        self.add_on_set_parameters_callback(self._on_param_set)

        # start async work
        self._run_task = self._loop.create_task(self._async_setup_and_run(), name="revo2_run_task")
        self.get_logger().info("Revo2Node initialized (Humble-safe).")
        
        # Right Hand Trigger subscriber
        self._sub_positions = self.create_subscription(
            Float64,
            "vr_trig_right",
            self._rh_trig_callback,
            10,  # QoS queue size
        )
        
        # Left Hand Trigger subscriber
        self._sub_positions = self.create_subscription(
            Float64,
            "vr_trig_left",
            self._lh_trig_callback,
            10,  # QoS queue size
        )

    def _rh_trig_callback(self, msg: Float64):
        self._current_right = [int((msg.data-0.5)*500), 750, int((msg.data-0.5)*2000), int((msg.data-0.5)*2000), int((msg.data-0.5)*2000), int((msg.data-0.5)*2000)]

    def _lh_trig_callback(self, msg: Float64):
        self._current_left = [int((msg.data-0.5)*500), 750, int((msg.data-0.5)*2000), int((msg.data-0.5)*2000), int((msg.data-0.5)*2000), int((msg.data-0.5)*2000)]
    
    def _on_param_set(self, params):
        for p in params:
            if p.name == "period" and p.type_ == p.Type.DOUBLE:
                self._period = p.value
            elif p.name == "loop_dt" and p.type_ == p.Type.DOUBLE:
                self._loop_dt = p.value
        from rclpy.parameter import SetParametersResult
        return SetParametersResult(successful=True)

    async def _async_setup_and_run(self):
        # Open device
        try:
            self.get_logger().info(f"Opening Modbus on {self._port} @ {self._baud} ...")
            self._client = await libstark.modbus_open(self._port, self._baud)
            await asyncio.sleep(1.0)
        except Exception as e:
            logger.critical(f"Exception while opening serial port: {e}")
            self.get_logger().fatal(f"Failed to open serial port: {self._port}: {e}")
            return

        if not self._client:
            logger.critical(f"Failed to open serial port: {self._port}")
            self.get_logger().fatal(f"Failed to open serial port: {self._port}")
            return

        # Device info
        info = None
        for attempt in range(3):
            try:
                info = await self._client.get_device_info(self._left_hand_id)
                break  # success
            except Exception as e:
                self.get_logger().warn(f"get_device_info attempt {attempt+1} failed: {e}")
                await asyncio.sleep(0.05)

        if not info:
            msg = f"Failed to get device info. Id: {self._left_hand_id}"
            logger.critical(msg)
            self.get_logger().fatal(msg)
            await self._cleanup()
            return

        logger.info(f"Left: {info.description}")
        self.get_logger().info(f"Device: {info.description}")

        # Mode
        try:
            await self._client.set_finger_unit_mode(self._left_hand_id, libstark.FingerUnitMode.Normalized)
        except Exception as e:
            logger.critical(f"Failed to set finger unit mode: {e}")
            self.get_logger().fatal(f"Failed to set finger unit mode: {e}")
            await self._cleanup()
            return

        # Loop
        self.get_logger().info("Entering control loop.")
        try:
            await self._client.set_finger_positions(self._right_hand_id, [0, 0, 0, 0, 0, 0])
            await self._client.set_finger_positions(self._left_hand_id, [0, 0, 0, 0, 0, 0])
            await asyncio.sleep(1.0)
            while not self._shutdown_event.is_set():
                # await self._client.set_serialport_baudrate(self._left_hand_id, libstark.Baudrate.Baud1Mbps)
                now_ms = int(time.time() * 1000)
                t = (now_ms / 1000.0) % self._period
                current = -1000 if t < self._period / 2.0 else 1000
                # pos = self._pos_a if t < self._period / 2.0 else self._pos_b
                try:
                    await self._client.set_finger_currents(self._right_hand_id, self._current_right)
                    await self._client.set_finger_currents(self._left_hand_id, self._current_left)
                    # self._client.set_finger_positions(self._left_hand_id, self.pos_a)
                except Exception as e:
                    self.get_logger().error(f"set_finger_positions_and_speeds failed: {e}")
                await asyncio.sleep(self._loop_dt)
        except asyncio.CancelledError:
            pass
        finally:
            await self._cleanup()
            self.get_logger().info("Control loop exited.")

    async def _cleanup(self):
        if self._client:
            try:
                libstark.modbus_close(self._client)
            except Exception as e:
                self.get_logger().warn(f"Error during modbus_close: {e}")
            self._client = None

    async def aclose(self):
        self._shutdown_event.set()
        if self._run_task and not self._run_task.done():
            self._run_task.cancel()
            try:
                await self._run_task
            except asyncio.CancelledError:
                pass

    # optional helper for external wait
    async def wait_finished(self):
        if self._run_task:
            try:
                await self._run_task
            except asyncio.CancelledError:
                pass


async def main_async(argv=None):
    rclpy.init(args=argv)
    loop = asyncio.get_running_loop()
    node = Revo2Node(loop=loop)

    # Spin ROS executor in a background thread
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    def spin():
        try:
            executor.spin()
        finally:
            executor.shutdown()

    spin_thread = Thread(target=spin, daemon=True)
    spin_thread.start()

    try:
        # Await node task (runs until shutdown or error)
        await node.wait_finished()
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt - shutting down.")
    finally:
        await node.aclose()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


def main(argv=None):
    # Run asyncio loop in main thread; ROS executor spins in a separate thread
    try:
        asyncio.run(main_async(argv))
    except KeyboardInterrupt:
        pass
    sys.exit(0)


if __name__ == "__main__":
    main()

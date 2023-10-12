import time
import logging

from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from rtde_io import RTDEIOInterface as RTDEIO

# Digital output pins
D_OUT_GRIPPER = 4


class URController(object):
    """Class for controlling the UR robot
    """

    # Logger for URController
    logger = logging.getLogger("URController")

    # Ranges for x, y, z coordinates
    x_range = (-0.05, -0.50)  # (m)
    y_range = (-0.69, -1.08)  # (m)
    z_range = (-0.28, 0.0, 0.15)  # (m)

    # Home position
    home = (-0.30, -0.25, 0.025)  # (m), (x, y, z)

    def __init__(self) -> None:
        """Constructor for URController
        """
        self.logger.info("URController init")

    def __del__(self) -> None:
        """Destructor for URController
        """
        self.logger.info("URController del")

    def _assert_range(self, x: float = None, y: float = None, z: float = None) -> bool:
        """Asserts that the given x, y, z coordinates are within range

        Args:
            x (float, optional): x position. Defaults to None.
            y (float, optional): y position. Defaults to None.
            z (float, optional): z position. Defaults to None.

        Returns:
            bool: True if within range, False otherwise
        """
        if x is not None:
            # Check if x is between the range
            if x < self.x_range[0] or x > self.x_range[1]:
                self.logger.error(f"x={x} is not within range {self.x_range}")
                return False

        if y is not None:
            # Check if y is between the range
            if y < self.y_range[0] or y > self.y_range[1]:
                self.logger.error(f"y={y} is not within range {self.y_range}")
                return False

        if z is not None:
            # Check if z is between the range
            if x is None and y is None:
                if z < self.z_range[0] or z > self.z_range[2]:
                    self.logger.error(f"z={z} is not within range {self.z_range}")
                    return False
            else:
                if z < self.z_range[1] or z > self.z_range[2]:
                    self.logger.error(f"z={z} is not within range {self.z_range}")
                    return False

        return True

    def calibrate_force_sensor(self) -> bool:
        """Calibrates the force sensor

        Returns:
            bool: True if successful, False otherwise
        """
        self.logger.info("URController calibrate_force_sensor()")

        time.sleep(2.0)

        # TODO calibrate

        time.sleep(2.0)
        return True

    def digital_out(self, pin: int, value: bool, sleep_s: float = 1.0) -> bool:
        """Sets the value of a digital output pin

        Args:
            pin (int): pin number
            value (bool): value to set
            sleep_s (float, optional): time to sleep after setting the value. Defaults to 1.

        Returns:
            bool: True if successful, False otherwise
        """
        self.logger.info(f"URController digital_out({pin}, {value})")

        # TODO set value

        time.sleep(sleep_s)
        return True

    def move_height(self, z: float, force: float = None) -> bool:
        """Moves the robot to the given z coordinate

        Args:
            z (float): z coordinate
            force (float, optional): force to apply. Defaults to None.

        Returns:
            bool: True if successful, False otherwise
        """
        self.logger.info(f"URController move_z({z})")

        # Assert z is within range
        if not self._assert_range(z=z):
            return False

        # TODO move

        if force is not None:
            pass

        return True

    def move_to_home(self) -> bool:
        """Moves the robot to the home position

        Returns:
            bool: True if successful, False otherwise
        """
        self.logger.info("URController move_to_home()")

        # Assert z is within range
        x, y, z = 0  # TODO read from the robot
        if not self._assert_range(x=x, y=y, z=z):
            return False

        # TODO move
        return True

    def move_to_object(self, x: float, y: float, z: float = None) -> bool:
        """Moves the robot to the given x, y coordinate

        Args:
            x (float): x coordinate
            y (float): y coordinate
            z (float, optional): z coordinate. Defaults to None.

        Returns:
            bool: True if successful, False otherwise
        """
        self.logger.info(f"URController move_to_object({x}, {y}, {z})")

        if z is None:
            # TODO assert height before moving to x, y
            z = 0  # TODO read z from the robot

        # Assert x, y are within range
        if not self._assert_range(x=x, y=y, z=z):
            return False

        # TODO move
        return True


if __name__ == "__main__":
    # Universal Robots controller
    ur_controller = URController()

    # Test digital output
    ur_controller.digital_out(pin=D_OUT_GRIPPER, value=True)
    ur_controller.digital_out(pin=D_OUT_GRIPPER, value=False)

    ur_controller.move_to_home()

    ur_controller.move_height(z=-0.2)
    ur_controller.move_height(z=0.0)
    ur_controller.move_height(z=-0.2, force=3.0)
    ur_controller.move_height(z=0.0)

    ur_controller.move_to_object(x=-0.75, y=0.0)

    ur_controller.move_to_home()

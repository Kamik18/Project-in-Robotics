import logging


class URController(object):
    """Class for controlling the UR robot
    """

    # Logger for URController
    logger = logging.getLogger("URController")

    def __init__(self) -> None:
        """Constructor for URController
        """
        self.logger.info("URController init")

    def __del__(self) -> None:
        """Destructor for URController
        """
        self.logger.info("URController del")

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

        # TODO assert x, y are within range

        if z is None:
            # TODO assert height before moving to x, y
            pass
        else:
            # TODO assert z are within range
            pass

        # TODO move
        return True

    def move_height(self, z: float) -> bool:
        """Moves the robot to the given z coordinate

        Args:
            z (float): z coordinate

        Returns:
            bool: True if successful, False otherwise
        """
        self.logger.info(f"URController move_z({z})")
        # TODO assert z is within range

        # TODO move
        return True

    def move_to_home(self) -> bool:
        """Moves the robot to the home position

        Returns:
            bool: True if successful, False otherwise
        """
        self.logger.info("URController move_to_home()")

        # TODO move
        return True

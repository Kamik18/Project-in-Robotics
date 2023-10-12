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
    # Ranges for x, y, z coordinates
    x_range = (-0.95, -0.3)  # (m) -300 mm to -950 mm
    y_range = (-0.765, -0.05)  # (m) -100 mm to -765 mm 
    z_range = (0.125, 0.2, 0.55)  # (m) 125 250 to 550

    # Home position
    home = (-0.35, -0.1, 0.25, 1.826, 2.555 ,0.0)  # (m), (x, y, z, rx, ry, rz)
    ORIENTATION:list = [1.826, 2.555 ,0.0]  # (rx, ry, rz)
    VELOCITY:float = 0.1
    ACCELERATION:float = 0.1
    
    def __init__(self) -> None:
        """Constructor for URController
        """
        # Logger for URController
        fh = logging.FileHandler("log/ur_controller.log", mode="w")
        log_level: int = logging.DEBUG
        fh.setLevel(log_level)
        fh.setFormatter(logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s"))
        fh.set_name("URController")
        
        self.logger = logging.getLogger("URController")
        self.logger.setLevel(log_level)
        self.logger.addHandler(fh)


        self.logger.info("URController init")
        IP = "192.168.100.10"
       
        try:
            self.rtde_c = RTDEControl(IP)
            self.rtde_r = RTDEReceive(IP)
            self.rtde_io = RTDEIO(IP)
        except RuntimeError:
            time.sleep(1.0)
            self.logger.debug("Retrying to connect to the robot")
            self.rtde_c = RTDEControl(IP)
            self.rtde_r = RTDEReceive(IP)
            self.rtde_io = RTDEIO(IP)
        
        self.logger.info("URController connected to the robot")

    def __del__(self) -> None:
        """Destructor for URController
        """
        # Robot
        try:
            # Stop the robot controller
            self.rtde_c.speedStop()
            self.rtde_c.stopScript()
            self.rtde_c.disconnect()
            self.rtde_io.disconnect()
        except:
            print("Robot failed to terminate")

        try:
            # Disconnect the receiver
            self.rtde_r.disconnect()
        except:
            print("Robot failed to terminate")
        

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

        time.sleep(0.2)
        if not self.rtde_c.zeroFtSensor():
            self.logger.error("Failed to zero force sensor")
            return False
        time.sleep(0.2)

        self.logger.info("Force sensor calibrated")

        # Force sensor calibrated
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
        if not self.rtde_io.setStandardDigitalOut(pin, value):
            self.logger.error(f"Failed to set digital output {pin} to {value}")
            return False

        time.sleep(sleep_s)

        self.logger.info(f"URController digital_out({pin}, {value}) finished")

        # Digital output set
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
        x, y, z_, _,_,_ = self.rtde_r.getActualTCPPose()
        if not self._assert_range(z=z_):
            return False

        # Make the movement assync if force is given
        assync = False
        if force is not None:
            assync = True

        # Define the position
        pos = [x, y, z] + self.ORIENTATION

        # Move to the given z coordinate
        self.rtde_c.moveL(pos, self.VELOCITY, self.ACCELERATION, assync)

        if force is not None:
            while True:
                force_tcp = self.rtde_r.getActualTCPForce()
                if force_tcp[2] > force:
                    self.logger.info(f"Force {force_tcp[2]} N reached")
                    break

                # Validate the distance between the current z and the target z
                _, _, z_, _,_,_ = self.rtde_r.getActualTCPPose()
                if abs(z - z_) < 0.0001: # 0.1 mm
                    self.logger.warning(f"Pose reached with force {force_tcp[2]} N")
                    break

            # Stop the robot movement
            self.rtde_c.stopL(0.1)
        
        self.logger.info(f"URController move_z({z}) finished")

        # Reached the target z
        return True
    

    def move_to_home(self) -> bool:
        """Moves the robot to the home position

        Returns:
            bool: True if successful, False otherwise
        """
        self.logger.info("URController move_to_home()")

        # Assert z is within range
        x, y, z, _,_,_ = self.rtde_r.getActualTCPPose()
     
        if not self._assert_range(x=x, y=y, z=z):
            return False

        self.rtde_c.moveL(self.home, self.VELOCITY, self.ACCELERATION)

        self.logger.info("URController move_to_home() finished")

        # Reached the home position
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
            # Get the current z
            _, _, z, _,_,_ = self.rtde_r.getActualTCPPose()

        # Assert x, y are within range
        if not self._assert_range(x=x, y=y, z=z):
            return False

        pos = [x, y, z] + self.ORIENTATION
        self.rtde_c.moveL(pos, self.VELOCITY, self.ACCELERATION)

        self.logger.info(f"URController move_to_object({x}, {y}, {z}) finished")

        # Reached the target x, y
        return True


if __name__ == "__main__":
    print("URController started!")
    # Universal Robots controller
    ur_controller = URController()


    
    # Test digital output
    ur_controller.digital_out(pin=D_OUT_GRIPPER, value=True)
    ur_controller.digital_out(pin=D_OUT_GRIPPER, value=False)
    
    ur_controller.move_to_home()
    
    ur_controller.calibrate_force_sensor()

    ur_controller.move_to_object(x=-0.5, y=-0.5)

    ur_controller.move_height(z=0.13)
    ur_controller.move_height(z=0.25)
    ur_controller.move_height(z=0.13, force=3.0)
    ur_controller.move_height(z=0.25)


    ur_controller.move_to_home()

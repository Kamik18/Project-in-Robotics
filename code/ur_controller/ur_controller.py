import time
import logging
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from rtde_io import RTDEIOInterface as RTDEIO
import numpy as np
import cv2
# import sleep
from time import sleep
import os
# Digital output pins
D_OUT_GRIPPER = 4


# Orientations
ORIENTATION_CENTER: list = [1.826, 2.555, 0.0]  # (rx, ry, rz)
ORIENTATION_LEFT: list = [1.826 - np.pi/2, 2.555, 0.0]  # (rx, ry, rz)
ORIENTATION_RIGHT: list = [1.826 + np.pi/2, 2.555, 0.0]  # (rx, ry, rz)


class URController(object):
    """Class for controlling the UR robot
    """
    # Ranges for x, y, z coordinates
    x_range = (-0.1, 0.7)  # (m)
    y_range = (0.0, 0.65)  # (m)
    z_range = (0.09, 0.1, 0.55)  # (m)

    # Home position
    home: list = [0.0, 0.1, 0.25]  # (m), (x, y, z)

    VELOCITY: float = 2.0
    ACCELERATION: float = 1.5

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

        self.rotation: list = [1.826, 2.555, 0.0]

        # Map x,y from table to base coordinates
        table_points: np.ndarray = np.array([[0, 0], [0, 0.6], [0.6, 0.6], [0.6, 0]])  # (m)
        base_points: np.ndarray = np.array([[-95.0, -688.4], [-326.55, -134.1], [-877.4, -363.95], [-647.7, -918.5]]) / 1000  # (m)
        # base_points:np.ndarray = np.array([[-91.4, -689.6], [-327.5, -139.4], [-879.2, -368.6], [-652.8, -922.1]]) / 1000  # (m)
        self.homography_table_base = cv2.findHomography(np.array(table_points), np.array(base_points))[0]
        self.homography_base_table = cv2.findHomography(np.array(base_points), np.array(table_points))[0]

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


    def beep(self):
        os.system(f"play -nq -t alsa synth {0.5} sine {440}")

    def _move(self, position: list, speed: float = VELOCITY, acceleration: float = ACCELERATION, assync: bool = False) -> None:
        """Moves the robot to the given position

        Args:
            position (list): position (x, y, z, rx, ry, rz)
            speed (float, optional): speed. Defaults to VELOCITY.
            acceleration (float, optional): acceleration. Defaults to ACCELERATION.
            assync (bool, optional): True if assync, False otherwise. Defaults to False.
        """
        # Change from base to table coordinates
        if position[0] is not None and position[1] is not None:
            position[0], position[1] = np.matmul(self.homography_table_base, np.array([position[0], position[1], 1]))[:2]
            self.last_pos = position
        else:
            position[0], position[1] = self.last_pos[:2]
        self.rtde_c.moveL(position, speed, acceleration, assync)

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
        time.cp(0.2)

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
        _, _, z_, _, _, _ = self.rtde_r.getActualTCPPose()

        if not self._assert_range(z=z_):
            return False

        # Make the movement assync if force is given
        assync = False
        if force is not None:
            assync = True

        # Define the position
        pos = [None, None, z] + self.rotation

        # Move to the given z coordinate
        self._move(pos, assync=assync)

        if force is not None:
            while True:
                force_tcp = self.rtde_r.getActualTCPForce()
                if force_tcp[2] > force:
                    self.logger.info(f"Force {force_tcp[2]} N reached")
                    break

                # Validate the distance between the current z and the target z
                _, _, z_, _, _, _ = self.rtde_r.getActualTCPPose()
                if abs(z - z_) < 0.0001:  # 0.1 mm
                    self.logger.warning(f"Pose reached with force {force_tcp[2]} N")
                    break

            # Stop the robot movement
            self.rtde_c.stopL(0.1)

        self.logger.info(f"URController move_z({z}) finished")

        # Reached the target z
        return True

    def move_to_home(self, orientation: list = ORIENTATION_CENTER) -> bool:
        """Moves the robot to the home position

        Args:
            orientation (list): orientation (rx, ry, rz)

        Returns:
            bool: True if successful, False otherwise
        """
        self.logger.info("URController move_to_home()")

        # Assert z is within range
        x, y, z, _, _, _ = self.rtde_r.getActualTCPPose()

        # Map x,y from base to table coordinates
        x, y = np.matmul(self.homography_base_table, np.array([x, y, 1]))[:2]

        if not self._assert_range(x=x, y=y, z=z):
            return False

        if len(orientation) != 3:
            self.logger.error(f"Invalid orientation {orientation}")
            return False

        # Assert orientation is ORIENTATION_CENTER, ORIENTATION_LEFT or ORIENTATION_RIGHT
        if orientation not in [ORIENTATION_CENTER, ORIENTATION_LEFT, ORIENTATION_RIGHT]:
            self.logger.error(f"Invalid orientation {orientation}")
            return False

        self._move(self.home + orientation)

        self.logger.info("URController move_to_home() finished")

        # Reached the home position
        return True

    def move_to_object(self, x: float, y: float, z: float = None) -> bool:
        """Moves the robot to the given x, y coordinate

        Args:
            x (float): x coordinate (m)
            y (float): y coordinate (m)
            z (float, optional): z coordinate (m). Defaults to None.

        Returns:
            bool: True if successful, False otherwise
        """
        self.logger.info(f"URController move_to_object({x}, {y}, {z})")

        if z is None:
            # Get the current z
            _, _, z, _, _, _ = self.rtde_r.getActualTCPPose()

        # Assert x, y are within range
        if not self._assert_range(x=x, y=y, z=z):
            return False

        pos = [x, y, z] + self.rotation

        self._move(pos)

        self.logger.info(f"URController move_to_object({x}, {y}, {z}) finished")

        # Reached the target x, y
        return True

    def set_orientation(self, orientation: list) -> bool:
        """Sets the orientation of the robot

        Args:
            orientation (list): orientation (rx, ry, rz)

        Returns:
            bool: True if successful, False otherwise
        """
        self.logger.info(f"URController set_orientation({orientation})")

        if len(orientation) != 3:
            self.logger.error(f"Invalid orientation {orientation}")
            return False

        # Assert orientation is ORIENTATION_CENTER, ORIENTATION_LEFT or ORIENTATION_RIGHT
        if orientation not in [ORIENTATION_CENTER, ORIENTATION_LEFT, ORIENTATION_RIGHT]:
            self.logger.error(f"Invalid orientation {orientation}")
            return False

        self.rotation = orientation

        self.logger.info(f"URController set_orientation({orientation}) finished")

        # Orientation set
        return True
    
    def place_cup_script(self, n_cup):
        
        # home pick 
        self.rtde_c.moveL([-0.08311, -0.44345, 0.26290, 3.167, 0.021, 0.0], self.VELOCITY, self.ACCELERATION, False)
       

        self.digital_out(pin=D_OUT_GRIPPER, value=True)
        self.rtde_c.moveL([-0.2027, -0.81958, 0.25188, 3.142, 0.0, 0.0], self.VELOCITY, self.ACCELERATION, False) # Cup home
        self.rtde_c.moveL([-0.12413, -0.89818, 0.25199, 3.142, 0.0, 0.0], self.VELOCITY, self.ACCELERATION, False) # Cup PICK
        self.digital_out(pin=D_OUT_GRIPPER, value=False)
        self.rtde_c.moveL([-0.11319, -0.88726, 0.25204, 3.142, 0.0, 0.0], self.VELOCITY, self.ACCELERATION, False) # Cup VENSTRE
        self.rtde_c.moveL([-0.13407, -0.90812, 0.25199, 3.142, 0.0, 0.0], self.VELOCITY, self.ACCELERATION, False) # Cup HOJRE
        self.rtde_c.moveL([-0.12413, -0.89818, 0.25199, 3.142, 0.0, 0.0], self.VELOCITY, self.ACCELERATION, False) # Cup PICK
        self.rtde_c.moveL([-0.12415, -0.89818, 0.22666, 3.142, 0.0, 0.0], self.VELOCITY, self.ACCELERATION, False) # Cup down
        self.rtde_c.moveL([-0.19675, -0.82559, 0.21834, 3.142, 0.0, 0.0], self.VELOCITY, self.ACCELERATION, False) # Cup lEAVE

        if n_cup == 1:
            self.rtde_c.moveL([-0.05950, -0.50416, 0.21832, 3.142, 0.0, 0.0], self.VELOCITY, self.ACCELERATION, False) # Place 1 home
            self.rtde_c.moveL([-0.06629, -0.49892, 0.15369, 3.142, 0.0, 0.0], self.VELOCITY, self.ACCELERATION, False) # Place 1 
            
        if n_cup == 2:
            
            # self.rtde_c.moveL([0.00981, -0.57683, 0.21432, 3.142, 0.0, 0.0], self.VELOCITY, self.ACCELERATION, False)# Place 2 home
            # self.rtde_c.moveL([0.00981, -0.57683, 0.15369, 3.142, 0.0, 0.0], self.VELOCITY, self.ACCELERATION, False)# Place 2 
        
            self.rtde_c.moveL([0.00166, -0.57061, 0.21432, 3.142, 0.0, 0.0], self.VELOCITY, self.ACCELERATION, False)# Place 2 home
            self.rtde_c.moveL([0.00166, -0.57061, 0.15369, 3.142, 0.0, 0.0], self.VELOCITY, self.ACCELERATION, False)# Place 2 
        
        if n_cup == 3:
            
            self.rtde_c.moveL([0.00554, -0.43208, 0.21832, 3.142, 0.0, 0.0], self.VELOCITY, self.ACCELERATION, False)# Place 3 home
            self.rtde_c.moveL([0.00554, -0.43208, 0.15369, 3.142, 0.0, 0.0], self.VELOCITY, self.ACCELERATION, False)# Place 3 
        
        if n_cup == 4:
            self.rtde_c.moveL([0.07255, -0.49863, 0.21832, 3.142, 0.0, 0.0], self.VELOCITY, self.ACCELERATION, False)# Place 4 home
            self.rtde_c.moveL([0.07255, -0.49863, 0.15369, 3.142, 0.0, 0.0], self.VELOCITY, self.ACCELERATION, False)# Place 4 
            
            # self.rtde_c.moveL([-0.06372, -0.36292, 0.21832, 3.142, 0.0, 0.0], self.VELOCITY, self.ACCELERATION, False)# Place 4 home
            # self.rtde_c.moveL([-0.06372, -0.36292, 0.15369, 3.142, 0.0, 0.0], self.VELOCITY, self.ACCELERATION, False)# Place 4 

        
            
        self.digital_out(pin=D_OUT_GRIPPER, value=True)
        x, y, z, rx, ry, rz = self.rtde_r.getActualTCPPose()
        self.rtde_c.moveL([x, y, 0.21832, rx, ry, rz], self.VELOCITY, self.ACCELERATION, False) # Home leave
        
        print("Cup script done")

    def move_home(self): 
        self.rtde_c.moveL([0.32978, -0.43165, 0.28722, 3.142, 0.0, 0.0], self.VELOCITY, self.ACCELERATION, False) # Home pos

    def move_home_pick_block(self): 
        self.rtde_c.moveL([-0.15395, -0.525, 0.29154, 3.142, 0.0, 0.0], self.VELOCITY, self.ACCELERATION, False) # Home pos

    def pick_plant_block(self):

        self.rtde_c.moveL([-0.08313, -0.44345, 0.17514, 3.449, -0.017, 2.413], self.VELOCITY, self.ACCELERATION, False)
        self.rtde_c.moveL([-0.40676, -0.74598, 0.23014, 3.531, -0.055, 1.58], self.VELOCITY, self.ACCELERATION, False)
    
        self.rtde_c.moveL([-0.53622, -0.87409, 0.32362, 3.531, -0.055, 1.586], self.VELOCITY, self.ACCELERATION, False)
        
        self.rtde_c.moveL([-0.56960, -0.90760, 0.40429, 3.531, -0.055, 1.586], self.VELOCITY, self.ACCELERATION, False)
        
        self.rtde_c.moveL([-0.31115, -0.64913, 0.41214, 3.531, -0.055, 1.586], self.VELOCITY, self.ACCELERATION, False)
        self.rtde_c.moveL([-0.26546, -0.59825, 0.40548, 3.475, -0.107, 2.181], self.VELOCITY, self.ACCELERATION, False)
        self.rtde_c.moveL([-0.32450, -0.67816, 0.12594, 3.367, -0.023, 2.373], self.VELOCITY, self.ACCELERATION, False)
        
        self.rtde_c.moveL([-0.32449, -0.67819, 0.08564, 3.367, -0.023, 2.373], self.VELOCITY, self.ACCELERATION, False)
        self.rtde_c.moveL([-0.08316, -0.44349, 0.08217, 3.449, -0.017, 2.412], self.VELOCITY, self.ACCELERATION, False)# waypoint 8 

        self.rtde_c.moveL([-0.08311, -0.44344, 0.23896, 3.449, -0.017, 2.413], self.VELOCITY, self.ACCELERATION, False)# waypoint 9
        
        joint = [54.16, -77.21, 124.56, -47.11, 41.04, 135.80]
        # deg to rad
        joint = [i * np.pi / 180 for i in joint]
        self.rtde_c.moveJ(joint, np.pi/2, np.pi, False)

        #self.rtde_c.moveL([-0.13816, -0.63873, 0.23830, 0.948, -1.750, 1.917], self.VELOCITY, self.ACCELERATION, False)# waypoint 7

        # Time to push 
        self.rtde_c.moveL([-0.40623, -0.74586, 0.29038, 1.116, -1.104, -0.010], self.VELOCITY, self.ACCELERATION, False)# push 
        self.rtde_c.moveL([-0.50253, -0.66999, 0.18183, 1.116, -1.104, -0.010], self.VELOCITY, self.ACCELERATION, False)
        self.rtde_c.moveL([-0.50256, -0.66997, 0.11111, 1.116, -1.104, -0.010], self.VELOCITY, self.ACCELERATION, False)
        self.rtde_c.moveL([-0.50253, -0.66999, 0.18183, 1.116, -1.104, -0.010], self.VELOCITY, self.ACCELERATION, False)
        
        self.rtde_c.moveL([-0.31937, -0.85095, 0.18689, 1.116, -1.104, -0.010], self.VELOCITY, self.ACCELERATION, False)
        self.rtde_c.moveL([-0.31943, -0.85098, 0.11551, 1.116, -1.104, -0.010], self.VELOCITY, self.ACCELERATION, False)
        self.rtde_c.moveL([-0.31937, -0.85095, 0.18689, 1.116, -1.104, -0.010], self.VELOCITY, self.ACCELERATION, False)
        self.rtde_c.moveL([-0.40623, -0.74586, 0.29038, 1.116, -1.104, -0.010], self.VELOCITY, self.ACCELERATION, False)# push 
        
        # Home
        joint = [51.60, -66.78, 104.97, -42.21, -8.20, 42.93]
        # deg to rad
        joint = [i * np.pi / 180 for i in joint]
        self.rtde_c.moveJ(joint,  np.pi/2,  np.pi, False)
        self.rtde_c.moveL([-0.08313, -0.44345, 0.17514, 3.449, -0.017, 2.413], self.VELOCITY, self.ACCELERATION, False)
        


    def pic_pland_and_place(self):
        # open gripper
        self.digital_out(pin=D_OUT_GRIPPER, value=True) # open gripper
        # move to home
        self.rtde_c.moveL([-0.08311, -0.44345, 0.26290, 3.167, 0.021, 0.0], self.VELOCITY, self.ACCELERATION, False)

        # pick 1
        self.rtde_c.moveL([-0.48297, -0.68931, 0.22378, 3.139, 0.021, 0.0], self.VELOCITY, self.ACCELERATION, False) # Home pick 1
        self.rtde_c.moveL([-0.48297, -0.68931, 0.13597, 3.139, 0.021, 0.0], self.VELOCITY, self.ACCELERATION, False) # pick pos
        self.digital_out(pin=D_OUT_GRIPPER, value=False)
        self.rtde_c.moveL([-0.48297, -0.68931, 0.22378, 3.139, 0.021, 0.0], self.VELOCITY, self.ACCELERATION, False)

        # place 1
        self.rtde_c.moveL([0.00464, -0.57504, 0.204, 3.142, 0.0, 0.0], self.VELOCITY, self.ACCELERATION, False) # Place 1 home
        self.rtde_c.moveL([0.00464, -0.57504, 0.147, 3.142, 0.0, 0.0], self.VELOCITY, self.ACCELERATION, False) # Place 1 
        self.digital_out(pin=D_OUT_GRIPPER, value=True)
        self.rtde_c.moveL([0.00464, -0.57504, 0.204, 3.142, 0.0, 0.0], self.VELOCITY, self.ACCELERATION, False) # Place 1 home
           
        #HOME
        self.rtde_c.moveL([-0.08311, -0.44345, 0.26290, 3.167, 0.021, 0.0], self.VELOCITY, self.ACCELERATION, False)

    

        # pick 2
        self.rtde_c.moveL([-0.54185, -0.68828, 0.22378, 3.139, 0.021, 0.0], self.VELOCITY, self.ACCELERATION, False)
        self.rtde_c.moveL([-0.54185, -0.68828, 0.13597, 3.139, 0.021, 0.0], self.VELOCITY, self.ACCELERATION, False)
        self.digital_out(pin=D_OUT_GRIPPER, value=False)
        self.rtde_c.moveL([-0.54185, -0.68828, 0.22378, 3.139, 0.021, 0.0], self.VELOCITY, self.ACCELERATION, False)
        

        # place 2
        self.rtde_c.moveL([0.07667, -0.64458, 0.204, 3.142, 0.0, 0.0], self.VELOCITY, self.ACCELERATION, False)
        self.rtde_c.moveL([0.07667, -0.64458, 0.147, 3.142, 0.0, 0.0], self.VELOCITY, self.ACCELERATION, False)
        self.digital_out(pin=D_OUT_GRIPPER, value=True)
        self.rtde_c.moveL([0.07667, -0.64458, 0.204, 3.142, 0.0, 0.0], self.VELOCITY, self.ACCELERATION, False)

        #HOME
        self.rtde_c.moveL([-0.08311, -0.44345, 0.26290, 3.167, 0.021, 0.0], self.VELOCITY, self.ACCELERATION, False)
       


        # pick 3
        self.rtde_c.moveL([-0.50773, -0.66181, 0.22378, 3.139, 0.021, 0.0], self.VELOCITY, self.ACCELERATION, False)
        self.rtde_c.moveL([-0.50773, -0.66181, 0.13597, 3.139, 0.021, 0.0], self.VELOCITY, self.ACCELERATION, False)
        self.digital_out(pin=D_OUT_GRIPPER, value=False)

        self.rtde_c.moveL([-0.50773, -0.66181, 0.22378, 3.139, 0.021, 0.0], self.VELOCITY, self.ACCELERATION, False)
        

        # place 3
        self.rtde_c.moveL([0.14653, -0.57453, 0.204, 3.142, 0.0, 0.0], self.VELOCITY, self.ACCELERATION, False)
        self.rtde_c.moveL([0.14653, -0.57453, 0.147, 3.142, 0.0, 0.0], self.VELOCITY, self.ACCELERATION, False)
        self.digital_out(pin=D_OUT_GRIPPER, value=True)
        self.rtde_c.moveL([0.14653, -0.57453, 0.204, 3.142, 0.0, 0.0], self.VELOCITY, self.ACCELERATION, False)


        #HOME
        self.rtde_c.moveL([-0.08311, -0.44345, 0.26290, 3.167, 0.021, 0.0], self.VELOCITY, self.ACCELERATION, False)
        


        
        # pick 4
        self.rtde_c.moveL([-0.51091, -0.72199, 0.22378, 3.139, 0.021, 0.0], self.VELOCITY, self.ACCELERATION, False)
        self.rtde_c.moveL([-0.51091, -0.72199, 0.13597, 3.139, 0.021, 0.0], self.VELOCITY, self.ACCELERATION, False)
        self.digital_out(pin=D_OUT_GRIPPER, value=False)
        self.rtde_c.moveL([-0.51091, -0.72199, 0.22378, 3.139, 0.021, 0.0], self.VELOCITY, self.ACCELERATION, False)
        self.beep()

        # place 4
        self.rtde_c.moveL([0.07759, -0.50158, 0.204, 3.142, 0.0, 0.0], self.VELOCITY, self.ACCELERATION, False)
        self.rtde_c.moveL([0.07759, -0.50158, 0.147, 3.142, 0.0, 0.0], self.VELOCITY, self.ACCELERATION, False)
        self.digital_out(pin=D_OUT_GRIPPER, value=True)
        self.rtde_c.moveL([0.07759, -0.50158, 0.204, 3.142, 0.0, 0.0], self.VELOCITY, self.ACCELERATION, False)

        # Home pos again
        self.rtde_c.moveL([-0.08311, -0.44345, 0.26290, 3.167, 0.021, 0.0], self.VELOCITY, self.ACCELERATION, False)



if __name__ == "__main__":
    print("URController started!")
    # Universal Robots controller
    ur_controller = URController()
    
    for j in range(70):    
        ur_controller.pick_plant_block()
        for i in range(4):    
            ur_controller.place_cup_script(i+1)

        ur_controller.pic_pland_and_place()
        
        print ("repeat script j: ", j+24)
        
        sleep(10)
    
    print ("Done")
    # Test digital output
    
    


from ur_controller.ur_controller import URController, D_OUT_GRIPPER
#from .ur_controller.ur_controller import URController, D_OUT_GRIPPER

import logging

if __name__ == "__main__":
    # Logger for URController
    fh = logging.FileHandler("log/main.log", mode="w")
    log_level: int = logging.DEBUG
    fh.setLevel(log_level)
    fh.setFormatter(logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s"))
    fh.set_name("main")
    
    logger = logging.getLogger("main")
    logger.setLevel(log_level)
    logger.addHandler(fh)


    # Universal Robots controller
    ur_controller = URController()

    # Camera
    # TODO

    state = 0

    iterations:int = 0
    while True:
        logger.info(f"Main loop iteration {iterations}")


        # State machine
        #   1. Move to home                (state 0)
        #   2. Move to seedling            (state 1)
        #   3. Pick up seedling            (state 2)
        #   4. Detect and move to object   (state 3)
        #   5. Insert seedling in object   (state 4)
        #   6. Move to home (repeat)       (state 0)

        # State 0: Move to home
        if state == 0:
            logger.info("Moving to home")

            if not ur_controller.move_to_home():
                logger.error("Error moving to home")
                break
            if not ur_controller.calibrate_force_sensor():
                logger.error("Error calibrating force sensor")
                break

            state = 1

            # Break after 1 complete iteration
            if iterations > 1:
                break

        # State 1: Move to seedling
        elif state == 1:
            logger.info("Moving to seedling")

            x: float = -0.5
            y: float = -0.5
            if ur_controller.move_to_object(x=x, y=y):
                state = 2
            else:
                logger.error("Error moving to seedling")
                break

        # State 2: Pick up seedling
        elif state == 2:
            logger.info("Picking up seedling")

            z = 0.13
            if not ur_controller.move_height(z=z, force=3.0):
                logger.error("Error inserting seedling in object")
                break

            # Grasp seedling
            ur_controller.digital_out(pin=D_OUT_GRIPPER, value=True)

            z = 0.25
            if not ur_controller.move_height(z=z):
                logger.error("Error inserting seedling in object")
                break

            state = 3

        # State 3: Detect and move to object
        elif state == 3:
            logger.info("Detecting and moving to object")

            x: float = -0.8
            y: float = -0.5
            positioned = False


            if ur_controller.move_to_object(x=x, y=y):
                state = 4
            else:
                logger.error("Error moving to object")
                break

        # State 4: Insert seedling in object
        elif state == 4:
            logger.info("Inserting seedling in object")

            z = 0.13
            if not ur_controller.move_height(z=z, force=3.0):
                logger.error("Error inserting seedling in object")
                break

            # Release seedling
            ur_controller.digital_out(pin=D_OUT_GRIPPER, value=False)

            z = 0.25
            if not ur_controller.move_height(z=z):
                logger.error("Error inserting seedling in object")
                break

            state = 0

            # Increment the number of complete iterations
            iterations += 1

        else:
            logger.error("Invalid state")
            break


        
from .URController import URController

if __name__ == "__main__":
    print("Hello World!")

    ur_controller = URController()

    state = 0
    while True:
        # State machine
        #   1. Move to home                (state 0)
        #   2. Move to seedling            (state 1)
        #   3. Pick up seedling            (state 2)
        #   4. Detect and move to object   (state 3)
        #   5. Insert seedling in object   (state 4)
        #   6. Move to home (repeat)       (state 0)

        # State 0: Move to home
        if state == 0:
            if ur_controller.move_to_home():
                state = 1
            else:
                print("Error moving to home")
                break

        # State 1: Move to seedling
        elif state == 1:
            # TODO: get x, y for the seedling
            x, y = 0.5, 0.5
            if ur_controller.move_to_object(x=x, y=y):
                state = 2
            else:
                print("Error moving to seedling")
                break

        # State 2: Pick up seedling
        elif state == 2:
            z = 0.0
            if not ur_controller.move_height(z=z):
                print("Error inserting seedling in object")
                break

            # TODO Grasp seedling

            z = 0.5
            if not ur_controller.move_height(z=z):
                print("Error inserting seedling in object")
                break

            state = 3

        # State 3: Detect and move to object
        elif state == 3:
            # TODO: get x, y from camera and if the object is positioned between the gripper
            x, y = 0.5, 0.5
            positioned = False

            while not positioned:
                if ur_controller.move_to_object(x=x, y=y):
                    state = 4
                else:
                    print("Error moving to object")
                    break

        # State 4: Insert seedling in object
        elif state == 4:
            z = 0.0
            if not ur_controller.move_height(z=z):
                print("Error inserting seedling in object")
                break

            # TODO Release seedling

            z = 0.5
            if not ur_controller.move_height(z=z):
                print("Error inserting seedling in object")
                break

            state = 0
        else:
            print("Invalid state")
            break

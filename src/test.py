import pygame


def create_base_client(ip_address):
    # Create a connection to the arm's base.
    return BaseClient(ip_address)

def move_to_cartesian_position(base, x, y, z, theta_x, theta_y, theta_z):
    # Create a Cartesian pose
    cartesian_pose = Base_pb2.CartesianPose()
    cartesian_pose.x = x
    cartesian_pose.y = y
    cartesian_pose.z = z
    cartesian_pose.theta_x = theta_x
    cartesian_pose.theta_y = theta_y
    cartesian_pose.theta_z = theta_z

    # Create a movement command
    action = Base_pb2.Action()
    action.reach_pose.target_pose = cartesian_pose

    # Send the command
    base.ExecuteAction(action)

def main():
    ip_address = '192.168.1.10'  # Replace with your robot's IP address
    base = create_base_client(ip_address)

    pygame.init()
    pygame.joystick.init()

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    try:
        while True:
            pygame.event.pump()

            # Example: Read the X and Y axes of the joystick (adjust as needed)
            x_axis = joystick.get_axis(0)
            y_axis = joystick.get_axis(1)

            # Convert joystick input to robot coordinates
            # This will need to be calibrated to your specific setup
            x = x_axis * 0.2
            y = y_axis * 0.2
            z = 0.2  # Static Z for example
            theta_x, theta_y, theta_z = 0, 0, 0  # Static orientation for example

            move_to_cartesian_position(base, x, y, z, theta_x, theta_y, theta_z)

    except KeyboardInterrupt:
        print("Program exited")
    finally:
        pygame.quit()

if __name__ == '__main__':
    main()

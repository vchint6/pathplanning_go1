import yaml

# Corrected path to the YAML configuration file
yaml_file_path = '/home/ros-vm/catkin_ws/src/slamware_ros_sdk/cfg/sim_drive/teb_local_planner_params.yaml'

# Load the YAML configuration
with open(yaml_file_path, 'r') as file:
    config = yaml.safe_load(file)

# Function to update max_vel_x based on user input
def update_max_vel_x():
    user_input = input("Enter 0 for max_vel_x = 0, 1 for max_vel_x = 0.1, 2 for max_vel_x = 0.4: ")
    
    if user_input == '0':
        config['max_vel_x'] = 0.0
    elif user_input == '1':
        config['max_vel_x'] = 0.1
    elif user_input == '2':
        config['max_vel_x'] = 0.4
    else:
        print("Invalid input. Keeping the current value.")

    # Save the updated configuration back to the YAML file
    with open(yaml_file_path, 'w') as file:
        yaml.dump(config, file)

# Call the function to update max_vel_x based on user input
update_max_vel_x()


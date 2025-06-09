from .kukavarproxy import KukaVarProxyClient
import time

def set_variable(robot,variable_name,new_value):
    robot.write(variable_name,new_value)
    #check if the variable was set correctly
    current_value = robot.read(variable_name)
    if current_value == new_value:
        print(f"Variable {variable_name} set to {new_value} successfully.")
    else:
        print(f"Failed to set variable {variable_name}. Current value: {current_value}")

def set_speed(robot, speed):
    robot.write("$OV_PRO", speed)
    current_speed = robot.read("$OV_PRO")
    if current_speed == speed:
        print(f"Speed set to {speed} successfully.")
    else:
        print(f"Failed to set speed. Current speed: {current_speed}")

def start_program(robot):
    variable_name = "KVP_START"
    new_value = "TRUE"
    # Set the start variable to TRUE
    set_variable(robot, variable_name, new_value)

def read_axis_position(robot):
    position = robot.read("$AXIS_ACT")
    #print(f"Current axis position: {position}")
    return position

def read_xyz_position(robot):
    position = robot.read("$POS_ACT")
    #print(f"Current XYZ position: {position}")
    return position

def move_enable(robot, enable=False):
    robot.write("KVPMOVE_ENABLE", "TRUE" if enable else "FALSE")
    current_status = robot.read("KVPMOVE_ENABLE")
    if current_status == "TRUE":
        print("Motion enabled successfully.")
    else:
        print("Failed to enable motion.")

def convert_position_string(position):
    """
    Converts a position string to a dictionary.
    Example input: "{E6POS: X 652.2838, Y 1282.793, Z 591.0475, A -10.04039, B 26.83337, C -140.3085, S 2, T 35, E1 0.0, E2 0.0, E3 0.0, E4 0.0, E5 0.0, E6 0.0}"
    Returns a dictionary with the position values.
    """
    position = position.strip("{}")
    position = position.split(": ")[1]  # Remove the prefix like "E6POS: "
    parts = position.split(", ")
    pos_dict = {}
    for part in parts:
        key, value = part.split(" ")
        pos_dict[key] = float(value)
    return pos_dict


def go_home(robot):
    #read home position from the robot
    home = "{E6AXIS: A1 0.0, A2 -90.00000, A3 90.0, A4 0.0, A5 0.0, A6 0.0, E1 0.0, E2 0.0, E3 0.0, E4 0.0, E5 0.0, E6 0.0}"
    robot.write("P1", home)


def rad_to_deg(radians):
    """
    Converts radians to degrees.
    """
    return radians * (180.0 / 3.141592653589793)
def deg_to_rad(degrees):
    """Converts degrees to radians.
    """
    return degrees * (3.141592653589793 / 180.0)

def enable_linear_motion(robot, enable=True):
    """Enables or disables linear motion.
    """
    robot.write("KVP_LIN_MOTION", "TRUE" if enable else "FALSE")
    current_status = robot.read("KVP_LINEAR_MOTION")
    if current_status == "TRUE":
        print("Linear motion enabled successfully.")
    else:
        print("Failed to enable linear motion.")
        
def enable_ptp_motion(robot, enable=True):
    """Enables or disables PTP motion.
    """
    robot.write("KVP_PTP_MOTION", "TRUE" if enable else "FALSE")
    current_status = robot.read("KVP_PTP_MOTION")
    if current_status == "TRUE":
        print("PTP motion enabled successfully.")
    else:
        print("Failed to enable PTP motion.")

def convert2RosPosition(pos_dict):
    #add 90° to A2 and remove the 90° from A3
    pos_dict[1] += 90.0  # A2
    pos_dict[2] -= 90.0  # A3
    #convert to radians
    pos_dict = {k: deg_to_rad(v) for k, v in pos_dict.items()}
    return pos_dict
    #build the position string

def convert2KukaPositionString(target_position):
    """Converts a target position in radians to a KUKA position string.
    target_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] A1 to A6
    """
    target_degrees = [rad_to_deg(value) for value in target_position]
    #remove the 90° from A2 and add it to A3
    #inverse A1
    target_degrees[0] = -target_degrees[0]  # Inverse A1
    target_degrees[1] -= 90.0  # A2
    target_degrees[2] += 90.0  # A3
    #convert to a dictionary
    target_dict = {
        "A1": target_degrees[0],
        "A2": target_degrees[1],
        "A3": target_degrees[2],
        "A4": target_degrees[3],
        "A5": target_degrees[4],
        "A6": target_degrees[5],
        "E1": 0.0,
        "E2": 0.0,
        "E3": 0.0,
        "E4": 0.0,
        "E5": 0.0,
        "E6": 0.0
    }
    #build the position string
    target_string = "{E6AXIS: " + ", ".join(f"{k} {v:.5f}" for k, v in target_dict.items()) + "}"
    return target_string


def ptp_motion(robot, target_position):
    """ Initiates a PTP motion to the target position.
    target position is a table of radians
    target_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] A1 to A6
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 E1 to E6
    """
    target_string = convert2KukaPositionString(target_position)
    #print(f"Target position: {target_string}")
    
    robot.write("P1", target_string)

def read_robot_state(robot):
    """Reads the current state of the robot, axis
    """
    axis_position = read_axis_position(robot)
    axis_dict = convert_position_string(axis_position)
    #convert axis_dict values to float
    axis_dict = {k: float(v) for k, v in axis_dict.items()}
    #covnert axis_dict values to degrees
    axis_dict = {k: deg_to_rad(v) for k, v in axis_dict.items()}
    #return axis_dict
    return axis_dict




'''
target_position = "{E6AXIS: A1 0, A2 -90.00000, A3 90, A4 0.0, A5 0.0, A6 0.0, E1 0.0, E2 0.0, E3 0.0, E4 0.0, E5 0.0, E6 0.0}"
Pos1 = "{E6POS: X 652.2838, Y 1282.793, Z 591.0475, A -10.04039, B 26.83337, C -140.3085, S 2, T 35, E1 0.0, E2 0.0, E3 0.0, E4 0.0, E5 0.0, E6 0.0}"
Pos2 = "{E6POS: X 652.2838, Y 1282.793, Z 591.0475, A -10.04039, B 26.83337, C -140.3085, S 2, T 35, E1 0.0, E2 0.0, E3 0.0, E4 0.0, E5 0.0, E6 0.0}"
'''

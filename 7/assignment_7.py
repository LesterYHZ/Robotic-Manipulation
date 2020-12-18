import numpy as np
from assignment_7_helper import World
import pdb
import time
np.set_printoptions(precision=3, suppress=True)

OBJ = ["red", "green", "blue", "box"]

def reset():
    robot_command = [np.array([0., 0., 0.1, 0. * np.pi / 180., 0.2])]
    return robot_command

def stack():
    """
    function to stack objects
    :return: average height of objects
    """
    # DO NOT CHANGE: initialize world
    env = World()

    # ============================================
    # YOUR CODE HERE:
    # example get object states
    obj_state = env.get_obj_state()
    for idx in range(len(obj_state)):
        print(OBJ[idx], ": ", obj_state[idx])

    # example send command to robot
    robot_command = [np.array([0., 0., 0.1, 0. * np.pi / 180., 0.2])]
    robot_command.append(np.array([obj_state[-1,0], obj_state[-1,1], obj_state[-1,2]*2, np.pi/2, 0.2]))
    robot_command.append(np.array([obj_state[-1,0], obj_state[-1,1], obj_state[-1,2], np.pi/2, 0.2]))
    robot_command.append(np.array([obj_state[-1,0], obj_state[-1,1], obj_state[-1,2], np.pi/2, 0.025]))
    robot_command.append(np.array([obj_state[-1,0], obj_state[-1,1], 0.25, np.pi/2, 0.025]))
    robot_command.append(np.array([obj_state[-1,0], -obj_state[-1,1], 0.25, np.pi/2, 0.025]))
    robot_command.append(np.array([obj_state[-1,0], -obj_state[-1,1], obj_state[-1,2], np.pi/2, 0.025]))
    robot_command.append(np.array([obj_state[-1,0], -obj_state[-1,1], obj_state[-1,2], np.pi/2, 0.2]))
    robot_command.append(np.array([obj_state[-1,0], -obj_state[-1,1], 0.25, np.pi/2, 0.2]))

    robot_command.append(np.array([obj_state[2,0],obj_state[2,1]*0.6,obj_state[2,2]*2,np.pi/2,0.2]))
    robot_command.append(np.array([obj_state[2,0]-0.1,obj_state[2,1]*0.6-0.025,obj_state[2,2]*0,np.pi/2,0.2]))
    env.robot_command(robot_command)

    # example get robot state
    rob_state = env.get_robot_state()
    print("rob_state: ", rob_state)

    pdb.set_trace()
    # ============================================
    # DO NOT CHANGE: getting average height of objects:
    obj_state = env.get_obj_state()
    avg_height = np.mean(obj_state[:, 2])
    print("Average Object Height: {:4.3f}".format(avg_height))
    return avg_height


if __name__ == "__main__":
    stack()

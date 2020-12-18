import numpy as np
from assignment_7_helper import World
import pdb
import time
np.set_printoptions(precision=3, suppress=True)

OBJ = ["red", "blue", "green", "box"]
# OBJ = ["box", "green", "blue", "red"]
RED = 0
BLUE = 1
GREEN = 2
BOX = 3
WIDTH = 0.05
FIXTURE = np.array([0, -0.15, 0.05])

def reset(current_state):
    '''
    current_state = get_robot_state
    '''
    x,y,z,theta = current_state

    robot_command = []
    if z < 0.35:
        robot_command.append([x,y,z+0.05,theta, 0])
    robot_command.append([0., 0., 0.4, 0., 0])
    return robot_command
    
def push(block, dist = 0, dir = 1):
    '''
    block = [pos x 3, theta x 3, qw]

    dir = 1 / -1 / 0: 1 -- pointing at +y, -1 -- pointing at -y, 0 -- not moving
    '''
    pos = block[0:3]
    x,y,z = pos
    x += 0.025
    theta = block[-2]
    width = 0.03
    d = 0.075

    x_prime = x + dir * d * np.sin(theta)
    y_prime = y + dir * d * (-np.cos(theta))
    x_f = x - dir * dist * np.sin(theta)
    y_f = y - dir * dist * (-np.cos(theta))

    command = []
    command.append([x_prime, y_prime, z*2, theta, width])
    command.append([x_prime, y_prime, z, theta, width])
    command.append([x_f, y_f, z, theta, width])
    command.append([x_prime, y_prime, z, theta, width])
    command.append([x_prime, y_prime, z*2, theta, width])
    return command

def flip(block, dir = -1):
    '''
    block = [pos x 3, theta x 3, qw]
    '''
    x,y,z = block[0:3]
    theta = block[-2]
    width = 0.2

    d = 0.035
    x_prime = x + dir * d * np.sin(theta)
    y_prime = y + dir * d * (-np.cos(theta))
    x_f = FIXTURE[0] + 0.045
    y_f = FIXTURE[1] + 0.075 * -dir

    command = []
    command.append([x_prime, y_prime, z*2, theta, width])
    command.append([x_prime, y_prime, z, theta, width])
    command.append([x_prime, y_prime, z, theta, 0.05])
    command.append([0, 0, 0.4, theta, 0.05])
    theta = 0
    command.append([x_f, y_f, 0.4, theta, 0.05])
    command.append([x_f, y_f, 0.13, theta, 0.05])
    command.append([x_f, y_f, 0.13, theta, width])
    command.append([x_f, y_f, 0.4, theta, width])
    command.append([x_f, y_f-0.05, 0.4, theta, width])
    
    return command

def grasp(block, theta_ = 0):
    '''
    block = [pos x 3, theta x 3, qw]
    '''
    pos = block[0:3]
    x,y,z = pos 
    theta = block[-2] + theta_
    width = 0.2

    command = []
    command.append([x, y, z*2, theta, 0.05])
    command.append([x, y, z*2, theta, width])
    command.append([x, y, z, theta, width])
    command.append([x, y, z, theta, 0.05])
    for z in np.linspace(z, 0.2, 10):
        command.append([x, y, z, theta, 0.05])

    return command

def place(block, theta_ = 0, target = np.array([0, 0, 0.03])):
    '''
    block = [pos x 3, theta x 3, qw]
    target [pos x 3, theta]
    '''
    pos = block[0:3]
    x,y,z = pos 
    theta = block[-2] + theta_
    x_f, y_f, z_f = target 

    command = []
    command.append([x_f, y_f, 0.4, theta, 0.05])
    theta = 0
    command.append([x_f, y_f, 0.4, theta, 0.05])
    command.append([x_f, y_f, z_f, theta, 0.05])
    command.append([x_f, y_f, z_f, theta, 0.20])
    for z in np.linspace(z_f, 0.4, 10):
        command.append([x_f, y_f, z, theta, 0.20])

    return command
    

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

    # initialization
    command = [np.array([0, 0, 0.03, 0, 0])]
    env.robot_command(command)
    cent_height = 0.05 + 0.11
    bias = np.array([[0, 0],
                     [0.01, 0.01], 
                     [0, 0], 
                     [0, 0]])
    for loop in np.array([0,1,2,3]):

        # initialization
        obj_state = env.get_obj_state()
        block = obj_state[loop]
        print("Working on object ", OBJ[loop])
        print(block)
        robot_state = env.get_robot_state()
        print("Robot State: ", robot_state)

        # reset robot position
        robot_command = []
        robot_command = reset(robot_state)    
        env.robot_command(robot_command)
        print("Robot reset")

        # push block
        if (OBJ[loop] != "box"):
            if OBJ[loop] == "green":
                dir = 1
                dist = 0.05
            else:
                dir = -1
                dist = 0.025
            robot_command = []
            robot_command = push(block, dist, dir = dir)        
            env.robot_command(robot_command)
            print(OBJ[loop], "block pushed")

            robot_command = []
            robot_state = env.get_robot_state()
            robot_command = reset(robot_state)
            env.robot_command(robot_command)

        obj_state = env.get_obj_state()
        block = obj_state[loop]
        robot_state = env.get_robot_state()

        # flip block
        if (OBJ[loop] != "box"):
            if OBJ[loop] == "green":
                dir = -1
            else:
                dir = 1
            robot_command = []
            robot_command = flip(block, dir = dir)
            env.robot_command(robot_command) 

        obj_state = env.get_obj_state()
        block = obj_state[loop]
        robot_state = env.get_robot_state()

        # grasp block
        if OBJ[loop] == "box": 
            theta_ = 0
        else:
            theta_ = 0
        robot_command = []
        robot_command = grasp(block, theta_)
        env.robot_command(robot_command)

        obj_state = env.get_obj_state()
        block = obj_state[loop]
        robot_state = env.get_robot_state()

        # place block
        if loop>0:
            target_x, target_y = obj_state[loop-1][0:2]
        else:
            target_x, target_y = 0, 0
        robot_command = []
        robot_command = place(block, theta_, target = np.array([FIXTURE[0]-0.045, FIXTURE[1], cent_height]))     
        env.robot_command(robot_command)

        robot_state = env.get_robot_state()
        robot_command = []
        robot_command = reset(robot_state)       
        env.robot_command(robot_command)

        # mission completed
        obj_state = env.get_obj_state()
        block = obj_state[loop]
        print("Object ", OBJ[loop], "stacked")
        print(block)
        robot_state = env.get_robot_state()
        print("Robot State: ", robot_state)

        print("\n----------------------------\n")

        command = [np.array([0, 0, 0.35, 0, 0])]
        env.robot_command(command)

        if OBJ[loop] != "box":
            cent_height += 0.1
        else:
            cent_height += 0.01

    rb = []
    rb = reset(robot_state)
    env.robot_command(rb)
    # pdb.set_trace()

    # ============================================
    # DO NOT CHANGE: getting average height of objects:
    obj_state = env.get_obj_state()
    avg_height = np.mean(obj_state[:, 2])
    print("Average Object Height: {:4.3f}".format(avg_height))
    return avg_height


if __name__ == "__main__":    
    h = stack()
    input("Press any key to continue...")
    print(h)

import numpy as np
import pybullet as p
import open3d as o3d
import assignment_5_helper as helper


def get_antipodal(pcd):
    """
    function to compute antipodal grasp given point cloud pcd
    :param pcd: point cloud in open3d format (converted to numpy below)
    :return: gripper pose (4, ) numpy array of gripper pose
    """
    # convert pcd to numpy arrays of points and normals
    pc_points = np.asarray(pcd.points)
    pc_normals = np.asarray(pcd.normals)
    # ------------------------------------------------
    # FILL WITH YOUR CODE
    w = 0.15 # m
    x,y,z,theta = 0,0,0,0
    resolution = 0.005
    x_max = np.max(pc_points[0])
    y_max = np.max(pc_points[1])
    z_max = np.max(pc_points[2])
    x_min = np.min(pc_points[0])
    y_min = np.min(pc_points[1])
    z_min = np.min(pc_points[2])
    x_mean = 0.5*(x_min+x_max)
    y_mean = 0.5*(y_min+y_max)
    z_mean = 0.5*(z_min+z_max)

    for idx in range(len(pc_points)):
        p1 = pc_points[idx]
        n1 = pc_normals[idx]
        for jdx in range(len(pc_points)):
            ifbreak = False
            p2 = pc_points[jdx]
            n2 = pc_normals[jdx]
            if (p1 == p2).all():
                break 
            norm = np.linalg.norm(p1-p2)
            if norm <= w:
                if np.arccos(np.dot(n1,n2)/(np.linalg.norm(n1)*np.linalg.norm(n2))) > np.pi/2:
                    if (np.abs(p2[2]-z_mean) < 10*resolution):
                        x = 0.5*(p1[0]+p2[0])
                        y = 0.5*(p1[1]+p2[1])
                        z = 0.5*(p1[2]+p2[2])
                        ifbreak = True 
                        break 
        if ifbreak:
            break

    theta = np.arctan2((p2[1]-p1[1]),(p2[0]-p1[0])) # + np.pi/2
    gripper_pose = np.array([x, y, z, theta]) # gripper pose: (x, y, z, theta) - replace 0. with your calculations
    # ------------------------------------------------
    return gripper_pose


def main(n_tries=5):
    # Initialize the world
    world = helper.World()

    # start grasping loop
    # number of tries for grasping
    for i in range(n_tries):
        # get point cloud from cameras in the world
        pcd = world.get_point_cloud()
        # check point cloud to see if there are still objects to remove
        finish_flag = helper.check_pc(pcd)
        if finish_flag:  # if no more objects -- done!
            print('===============')
            print('Scene cleared')
            print('===============')
            break
        # visualize the point cloud from the scene
        helper.draw_pc(pcd)
        # compute antipodal grasp
        gripper_pose = get_antipodal(pcd)
        # send command to robot to execute
        robot_command = world.grasp(gripper_pose)
        # robot drops object to the side
        world.drop_in_bin(robot_command)
        # robot goes to initial configuration and prepares for next grasp
        world.home_arm()
        # go back to the top!

    # terminate simulation environment once you're done!
    p.disconnect()
    return finish_flag


if __name__ == "__main__":
    flag = main()

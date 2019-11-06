import sys
from os.path import abspath, dirname, join
import numpy as np
from ompl import base as ob
from ompl import app as oa
from ompl import geometric as og
ompl_app_root = "/home/yinglong/Downloads/ompl-1.4.2-Source/omplapp-1.4.2-Source/"
ompl_resources_dir = join(ompl_app_root, 'resources/3D')

def allocatePlanner(si, plannerType):
    if plannerType.lower() == "bfmtstar":
        return og.BFMT(si)
    elif plannerType.lower() == "bitstar":
        return og.BITstar(si)
    elif plannerType.lower() == "fmtstar":
        return og.FMT(si)
    elif plannerType.lower() == "informedrrtstar":
        return og.InformedRRTstar(si)
    elif plannerType.lower() == "prmstar":
        return og.PRMstar(si)
    elif plannerType.lower() == "rrtstar":
        return og.RRTstar(si)
    elif plannerType.lower() == "sorrtstar":
        return og.SORRTstar(si)
    else:
        ou.OMPL_ERROR("Planner-type is not implemented in allocation function.")


def randQ(N):
    #Generates a uniform random quaternion
    #James J. Kuffner 2004
    #A random array 3xN
    s = np.random.rand(3,N)
    sigma1 = np.sqrt(1.0 - s[0])
    sigma2 = np.sqrt(s[0])
    theta1 = 2*np.pi*s[1]
    theta2 = 2*np.pi*s[2]
    w = np.cos(theta2)*sigma2
    x = np.sin(theta1)*sigma1
    y = np.cos(theta1)*sigma1
    z = np.sin(theta2)*sigma2
    return np.array([w, x, y, z])

def QtoAxisAngle(Q):
    # angle = 2 * acos(qw)
    #x = qx / sqrt(1-qw*qw)
    #y = qy / sqrt(1-qw*qw)
    #z = qz / sqrt(1-qw*qw)
    angle = 2 * np.arccos(Q[0])
    x = Q[1] / np.sqrt(1-Q[0]*Q[0])
    y = Q[2] / np.sqrt(1-Q[0]*Q[0])
    z = Q[3] / np.sqrt(1-Q[0]*Q[0])
    return np.array([x, y, z, angle])


###############################################################################



# define start state boundary
start_bound = np.array([[184.95, -70, 49], [295, -330, 120]])

for i in range(5000):
    # generate 5000 paths in total
    setup = oa.SE3RigidBodyPlanning()

    # load the robot and the environment
    #setup.setRobotMesh(join(ompl_resources_dir, 'cubicles_robot.dae'))
    setup.setRobotMesh(join(ompl_resources_dir, 'Home_robot.dae'))
    setup.setEnvironmentMesh(join(ompl_resources_dir, 'Home_env.dae'))



    # setting collision checking resolution to 1% of the space extent
    setup.getSpaceInformation().setStateValidityCheckingResolution(0.01)

    setup.setPlanner(allocatePlanner(setup.getSpaceInformation(), "rrtstar"))
    # we call setup just so print() can show more information

    goal = ob.State(setup.getSpaceInformation())
    goal().setX(262.95)
    goal().setY(75.05)
    goal().setZ(46.19)
    goal().rotation().setIdentity()
    setup.setGoalState(goal)
    setup.setup()
    print(setup)



    # setup random start and goal, and check if the start and goal are in collision
    val = setup.getStateValidityChecker()

    print('generating path %d...' % (i))

    while True:
        # define start state
        start_vec = np.random.uniform(low=start_bound[0], high=start_bound[1])
        start_ori = randQ(1)[:,0]
        # convert from quarternion to Axis Angle Representation
        start_ori = QtoAxisAngle(start_ori)

        start = ob.State(setup.getSpaceInformation())
        start().setX(start_vec[0])
        start().setY(start_vec[1])
        start().setZ(start_vec[2])
        start().rotation().setAxisAngle(start_ori[0], start_ori[1], start_ori[2], start_ori[3])

        if val.isValid(start()):
            break
    # set the start states
    setup.setStartState(start)

    # try to solve the problem
    solve = setup.solve(60)
    if solve:
        # simplify & print the solution
        setup.simplifySolution()
        path = setup.getSolutionPath()
        path_print = path.printAsMatrix()
        f = open('data/paths/path_%d.txt' % (i), 'w')
        f.write(path_print)
        f.close()

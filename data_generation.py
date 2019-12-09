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
    """
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

    """
    # generate random quarternion that is rotation around (0,0,1)
    s = np.random.rand(N)
    theta = 2*np.pi*s
    C = np.cos(theta/2)
    S = np.sin(theta/2)
    return np.array([C,0*S,0*S,S])

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

starts = np.loadtxt('start.txt')
starts = starts[:75]
#np.savetxt('start_75.txt', starts, fmt='%f')
#np.savetxt('goal.txt', goals, fmt='%f')

# define start state boundary

start_goal = []
for i in range(len(starts)):
    for j in range(i+1, len(starts)):
        start_goal.append( (i,j) )
        # backward plan is the same as forward plan, so we don't need to plan twice
print('number of path queries: ')
print(len(start_goal))
#for i in range(len(start_goal)):
for i in range(len(start_goal)):
    print(start_goal[i])
    # try to plan EXACT_SOLUTION
    # generate 5000 paths in total
    setup = oa.SE3RigidBodyPlanning()

    # load the robot and the environment
    #setup.setRobotMesh(join(ompl_resources_dir, 'cubicles_robot.dae'))
    setup.setRobotMesh(join(ompl_resources_dir, 'Home_robot.dae'))
    setup.setEnvironmentMesh(join(ompl_resources_dir, 'Home_env.dae'))



    # setting collision checking resolution to 1% of the space extent
    setup.getSpaceInformation().setStateValidityCheckingResolution(0.01)

    setup.setPlanner(allocatePlanner(setup.getSpaceInformation(), "informedrrtstar"))
    # we call setup just so print() can show more information


    print('generating path %d...' % (i))

    # define start state
    # load start and goal from file
    start_i = start_goal[i][0]
    goal_i = start_goal[i][1]

    start_vec = starts[start_i][:3]
    goal_vec = starts[goal_i][:3]
    start_ori = starts[start_i][3:]
    goal_ori = starts[goal_i][3:]
    # from OMPL Quarternion to AxisAngle, then to OMPL Quarternion
    start_ori = [start_ori[3], start_ori[0], start_ori[1], start_ori[2]]
    goal_ori = [goal_ori[3], goal_ori[0], goal_ori[1], goal_ori[2]]
    start_ori = np.array(start_ori)
    goal_ori = np.array(goal_ori)
    # convert from quarternion to Axis Angle Representation
    start_ori = QtoAxisAngle(start_ori)
    goal_ori = QtoAxisAngle(goal_ori)
    start = ob.State(setup.getSpaceInformation())
    start().setX(start_vec[0])
    start().setY(start_vec[1])
    start().setZ(start_vec[2])
    start().rotation().setAxisAngle(start_ori[0], start_ori[1], start_ori[2], start_ori[3])

    # set the start states
    setup.setStartState(start)

    goal = ob.State(setup.getSpaceInformation())
    goal().setX(goal_vec[0])
    goal().setY(goal_vec[1])
    goal().setZ(goal_vec[2])
    goal().rotation().setAxisAngle(goal_ori[0], goal_ori[1], goal_ori[2], goal_ori[3])

    # set the goal states
    setup.setGoalState(goal)

    setup.setup()

    print("start:")
    print(start)
    print("goal:")
    print(goal)

    # try to solve the problem
    solve = setup.solve(120)
    print('status: ')
    print(solve)
    if solve.getStatus() == ob.PlannerStatus.EXACT_SOLUTION:
        # simplify & print the solution
        setup.simplifySolution()
        path = setup.getSolutionPath()
        path_print = path.printAsMatrix()

        f = open('data/paths/path_%d.txt' % (i), 'w')

        f.write(path_print)
        f.close()

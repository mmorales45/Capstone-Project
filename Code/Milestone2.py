import numpy as np
import modern_robotics as mr
from numpy import pi
'''
How to run the code

To run the code, simply copy and paste the following line to a terminal (linux) to run the script. One can also press the play button if using Visual Studo Code

python3 Milestone2.py


This will run the code below and generate the csv file. 
After this, load CoppeliaSim Scene 8 and load the generated "Trajectory.csv" file.
The end effector should do the following
Move to stand off
Move to grasping position
Close gripper
Move to standoff
Move to standoff above cube's final position
Move to grasping position
Open gripper
Move back to standoff above cube's final position

'''


def add_to_trajList(traj,reference_traj,N,gripper):
    '''
    Function: Addes results from ScrewTrajectory into list of values after picking out r11,r12,etc.
    Arguments:
        traj- list of SE(3) matrices from ScrewTrajectory
        reference_traj- List of values that will be converted to .csv file
        N- Number of points 
        gripper- State of gripper, open or close
    Return:
        reference_traj- the list of 13 variables 
    '''
    for i in range(N):
        r11 = traj[i][0][0]
        r12 = traj[i][0][1]
        r13 = traj[i][0][2]
        r21 = traj[i][1][0]
        r22 = traj[i][1][1]
        r23 = traj[i][1][2]
        r31 = traj[i][2][0]
        r32 = traj[i][2][1]
        r33 = traj[i][2][2]
        px = traj[i][0][3]
        py = traj[i][1][3]
        pz = traj[i][2][3]
        reference_traj.append([r11,r12,r13,r21,r22,r23,r31,r32,r33,px,py,pz,gripper])
    return reference_traj

def TrajectoryGenerator(Tse_initial,Tsc_initial,Tsc_goal,Tce_grasp,Tce_standoff,k):
    '''
    Arguments:
        Tse_initial- Initial configuration of the end effector relative to the world frame
        Tsc_initial-Initial configuration of the cube relative to the world frame
        Tsc_goal-Initial configuration of the cube's end position relative to the world frame
        Tce_grasp-Transformation matrix of the end effector relative to the cube when grasping
        Tce_standoff-Transformation matrix of the end effector relative to the cube at standoff
        k- The number of trajectory configurations per 0.01 seconds
    Returns:
        reference_traj- N number of configurations for the end effector for the eight steps in the trajectory
    '''
    #Time scaling method
    method = 5
    #Get the end effecor configuration relative to the fixed frame during grasping at cube's initial position
    Tse_grasp = Tsc_initial@Tce_grasp
    #Get the end effecor configuration relative to the fixed frame during standoff at cube's initial position
    Tse_standoff = Tsc_initial@Tce_standoff
    #Get the end effecor configuration relative to the fixed frame during grasping at cube's final position
    Tse_end_standoff = Tsc_goal@Tce_standoff #Tse
    #Get the end effecor configuration relative to the fixed frame during grasping at cube's final position
    Tse_end_goal = Tsc_goal@Tce_grasp
    
    #Empty list to store values
    reference_traj = []
    #Number of points
    N = int(k/0.01)*6
    #gripper state, 0 is open
    gripper_state = 0
    #Trajectory of SE(3) matrices to get from initial end effector configuration to standoff, above the inital cube position
    Tse1 = mr.ScrewTrajectory(Tse_initial,Tse_standoff,1,N,method)
    #Append result of ScrewTrajectory to reference_traj
    reference_traj = add_to_trajList(Tse1,reference_traj,N,gripper_state)

    N = int(k/0.01)*2
    gripper_state = 0
    #Trajectory of SE(3) matrices to get from standoff to grasping position
    Tse2 = mr.ScrewTrajectory(Tse_standoff,Tse_grasp,1,N,method)
    reference_traj = add_to_trajList(Tse2,reference_traj,N,gripper_state)

    N = int(k/0.01)*2
    #Close the gripper
    gripper_state = 1
    #Trajectory of SE(3) matrices to get close the gripper
    Tse3 = mr.ScrewTrajectory(Tse_grasp,Tse_grasp,1,N,method)
    reference_traj = add_to_trajList(Tse3,reference_traj,N,gripper_state)
    
    N = int(k/0.01)*2
    gripper_state = 1
    #Trajectory of SE(3) matrices to get from grasping to standoff position
    Tse4 = mr.ScrewTrajectory(Tse_grasp,Tse_standoff,1,N,method)
    reference_traj = add_to_trajList(Tse4,reference_traj,N,gripper_state)
    
    N = int(k/0.01)*6
    gripper_state = 1
    #Trajectory of SE(3) matrices to get from standoff position to standoff position above cube's final position
    Tse5 = mr.ScrewTrajectory(Tse_standoff,Tse_end_standoff,1,N,method)
    reference_traj = add_to_trajList(Tse5,reference_traj,N,gripper_state)
    
    N = int(k/0.01)*2
    gripper_state = 1
    #Trajectory of SE(3) matrices to get from standoff position above cube's final position to grasping position
    Tse6 = mr.ScrewTrajectory(Tse_end_standoff,Tse_end_goal,1,N,method)
    reference_traj = add_to_trajList(Tse6,reference_traj,N,gripper_state)

    N = int(k/0.01)*2
    gripper_state = 0
    #Trajectory of SE(3) matrices to open the gripper
    Tse7 = mr.ScrewTrajectory(Tse_end_goal,Tse_end_goal,1,N,method)
    reference_traj = add_to_trajList(Tse7,reference_traj,N,gripper_state)
    
    N = int(k/0.01)*2
    gripper_state = 0
    #Trajectory of SE(3) matrices to go from releasing position to standoff position
    Tse8 = mr.ScrewTrajectory(Tse_end_goal,Tse_end_standoff,1,N,method)
    reference_traj = add_to_trajList(Tse8,reference_traj,N,gripper_state)
    
    return reference_traj

if __name__ == "__main__": # the main function
    
    Tbo = np.array([[1,0,0,0.1662],[0,1,0,0],[0,0,1,0.0026],[0,0,0,1]])
    Moe = np.array([[1,0,0,0.033],[0,1,0,0],[0,0,1,0.6546],[0,0,0,1]])

    #Initial configuration of the end effector relative to the world frame
    Tse_initial = np.array([[0,0,1,0.25],[0,1,0,0],[-1,0,0,0.75],[0,0,0,1]])
    #Initial configuration of the cube relative to the world frame
    Tsc_initial = np.array([[1,0,0,1],[0,1,0,0],[0,0,1,0.025],[0,0,0,1]])
    #Initial configuration of the cube's end position relative to the world frame
    Tsc_goal = np.array([[0,1,0,0],[-1,0,0,-1],[0,0,1,0.025],[0,0,0,1]])

    #Theta value to rotate about for stand off and grasp
    theta = 3*(pi/4)
    #Transformation matrix of the end effector relative to the cube at standoff
    Tce_standoff=np.array([[np.cos(theta),0,np.sin(theta),0],[0,1,0,0],[-np.sin(theta),0,np.cos(theta),0.15],[0,0,0,1]])
    #Transformation matrix of the end effector relative to the cube when grasping
    Tce_grasp=np.array([[np.cos(theta),0,np.sin(theta),0],[0,1,0,0],[-np.sin(theta),0,np.cos(theta),0.0],[0,0,0,1]])
    
    #Use k to get number of points, divide by 0.01
    k= 1
    #Get the matricies for the robot to pick up a box and move it to another location
    final_traj_list = TrajectoryGenerator(Tse_initial,Tsc_initial,Tsc_goal,Tce_grasp,Tce_standoff,k)
    #Create the .csv file
    np.savetxt("Trajectory.csv", final_traj_list, delimiter= ",")
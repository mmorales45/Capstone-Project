'''
To run this function that generates the "Overshoot" result, run the following line.

python3 Overshoot.py
'''
from modern_robotics.core import MatrixLog6
import numpy as np
import modern_robotics as mr
from numpy import pi
from numpy.core.numerictypes import maximum_sctype
import numpy.linalg
from Milestone1 import NextState, check_velocity
from Milestone2 import TrajectoryGenerator, add_to_trajList
from Milestone3 import Feedbackcontrols
import matplotlib.pyplot as plt
import logging
logging.basicConfig(filename='Overshoot.log', level=logging.INFO)

def Full_program():
    '''
    Function: This function uses the NextState, TrajectoryGenerator and Feedbackcontrols functions from the previous Milestones to make the youbot move a cube from an inital position
            to a final position. It first generates the trajectory from using TrajectoryGenerator, then there is a loop that creates controls using Feedbackcontrols and then controls is sent to
            NextState to create the new configuration of the robot.

    Arguments:
            None
    
    Return:
            csv file of the configurations of the youBot
            csv file of the X_err of the youBot
    
    '''
    #The initial configuration varaibles for 
    chasis_phi = 0.2
    chasis_x = 0.2
    chasis_y = 0.2
    joint_1 = 0
    joint_2 = -0.3
    joint_3 = -0.3
    joint_4 = -0.3
    joint_5 = -0.3
    wheel_1 = 0
    wheel_2 = 0
    wheel_3 = 0
    wheel_4 = 0
    gripper_state = 0
    #Initial configuration of the end effector from the fixed frame
    Tse_initial = np.array([[0,0,1,0],[0,1,0,0],[-1,0,0,0.5],[0,0,0,1]])
    #The initial configuration of the youBot
    initial_configuration = np.array([chasis_phi,chasis_x,chasis_y,joint_1,joint_2,joint_3,joint_4,joint_5,wheel_1,wheel_2,wheel_3,wheel_4,gripper_state]) 
    #Kp and Ki multipliers
    kp_multiplier = 5
    ki_multiplier = 4
    Kp = np.identity(6)*kp_multiplier
    Ki = np.identity(6)*ki_multiplier
    T_b0 = np.array([[1,0,0,0.1662],[0,1,0,0],[0,0,1,0.0026],[0,0,0,1]])
    M_0e = np.array([[1,0,0,0.033],[0,1,0,0],[0,0,1,0.6546],[0,0,0,1]])

    #Initial configuration of the cube relative to the world frame
    Tsc_initial = np.array([[1,0,0,1],[0,1,0,0],[0,0,1,0.025],[0,0,0,1]])
    #Initial configuration of the cube's end position relative to the world frame
    Tsc_goal = np.array([[0,1,0,0],[-1,0,0,-1],[0,0,1,0.025],[0,0,0,1]])
    #Angle at which the end effector comes in
    theta = 3*(pi/4)
    #Transformation matrix of the end effector relative to the cube at standoff
    Tce_standoff=np.array([[np.cos(theta),0,np.sin(theta),0],[0,1,0,0],[-np.sin(theta),0,np.cos(theta),0.15],[0,0,0,1]])
    #Transformation matrix of the end effector relative to the cube when grasping
    Tce_grasp=np.array([[np.cos(theta),0,np.sin(theta),0],[0,1,0,0],[-np.sin(theta),0,np.cos(theta),0.0],[0,0,0,1]])
    #Joint screw axis in the end effector frame
    Blist = np.array([[0,0,1,0,0.033,0],
                    [0,-1,0,-0.5076,0,0],
                    [0,-1,0,-0.3526,0,0],
                    [0,-1,0,-0.2176,0,0],
                    [0,0,1,0,0,0]]).T
    #number of trajectory reference configurations
    k= 1
    #Number of trajectory reference configurationsper depending on k
    logging.info('Creating Trajectory List')
    final_traj_list = TrajectoryGenerator(Tse_initial,Tsc_initial,Tsc_goal,Tce_grasp,Tce_standoff,k)
    logging.info('Finished Creating Trajectory List')
    #timestep
    dt = 0.01
    #time of simulation
    time = 24
    #Number of iterations for the loop
    iterations = int(time/dt)
    #initial x_err integral
    X_err_integral = 0
    #Make empty list called configuration to fill values in the loop
    configuration = np.zeros((iterations,13))
    #the first configuration is the initial configuration from above
    configuration[0] = initial_configuration
    #current end effector reference configuration
    X_d = np.array([[0,0,1,0.5],
                    [0,1,0,0],
                    [-1,0,0,0.5],
                    [0,0,0,1]])
    #end effector configuration at the next timestep at the next timestep
    X_d_next = np.array([[0,0,1,0.6],
                        [0,1,0,0],
                        [-1,0,0,0.3],
                        [0,0,0,1]])


    #Speed limit
    max_speed = 15
    #Empty list to append configurations into a csv file
    next_config = []
    #Empty list to append X_err
    X_err_list = []
    logging.info('Starting to Generate Configurations')
    for i in range(1,iterations-1):
        #Create a variable based on the previous configuration, intial would be the initial configuration
        previous_config = configuration[i-1,:]
        #chassis relative to the fixed frame
        T_sb = np.array([[np.cos(previous_config[0]),-np.sin(previous_config[0]),0,previous_config[1]],
                        [np.sin(previous_config[0]),np.cos(previous_config[0]),0,previous_config[2]],
                        [0,0,1,0.0963],
                        [0,0,0,1]])
        #List of joint values
        thetalist = np.array(previous_config[3:8])
        #End effector relative to the base of the arm
        T_0e = mr.FKinBody(M_0e,Blist,thetalist)
        #Current actual end effector configuration
        X = T_sb@T_b0@T_0e
        #Updated current end effector reference configuration
        X_d = np.array([[final_traj_list[i][0],final_traj_list[i][1],final_traj_list[i][2],final_traj_list[i][9]],
                        [final_traj_list[i][3],final_traj_list[i][4],final_traj_list[i][5],final_traj_list[i][10]],
                        [final_traj_list[i][6],final_traj_list[i][7],final_traj_list[i][8],final_traj_list[i][11]],
                        [0,0,0,1]])
        #Updated end effector configuration at the next timestep at the next timestep
        X_d_next = np.array([[final_traj_list[i+1][0],final_traj_list[i+1][1],final_traj_list[i+1][2],final_traj_list[i+1][9]],
                        [final_traj_list[i+1][3],final_traj_list[i+1][4],final_traj_list[i+1][5],final_traj_list[i+1][10]],
                        [final_traj_list[i+1][6],final_traj_list[i+1][7],final_traj_list[i+1][8],final_traj_list[i+1][11]],
                        [0,0,0,1]])
        #Get controls and X_err from Feedbackcontrols
        V_d, Ad_X_inv_X_d_Vd, V, X_err, controls,X_err_integral  = Feedbackcontrols(X,X_d,X_d_next,Kp,Ki,dt,X_err_integral,previous_config)
        X_err_list.append(X_err)
        #Index the wheel speeds
        u = controls[0:4]
        #Index the joint velocities
        thetadot = controls[4:]
        #Combine wheel and joint speeds
        speeds = np.hstack((thetadot,u))
        #Calculate the new previous configuration
        previous_config = NextState(previous_config[:12],speeds,dt,max_speed)
        #The current configuration is the previous configuration and the girpper state
        configuration[i] = np.hstack((previous_config,final_traj_list[i][12]))
        #Append values to a list for csv
        next_config.append(configuration[i-1,:])

    logging.info('Finished generating Configurations')
    #Create csv file
    np.savetxt("Overshoot_config.csv", next_config, delimiter= ",")
    np.savetxt("Overshoot_Xerr.csv", X_err_list, delimiter= ",")
    y1 = []
    y2 = []
    y3 = []
    y4 = []
    y5 = []
    y6 = []
    #Create variables for each joint error
    for i in X_err_list:
        y1.append(i[0])
        y2.append(i[1])
        y3.append(i[2])
        y4.append(i[3])
        y5.append(i[4])
        y6.append(i[5])
    #Create x span of values and plot!
    x_axis = np.linspace(0,time,len(X_err_list))
    logging.info('Plotting Errors')
    plt.plot(x_axis,y1,label = "Error Twist Xerr[0]")
    plt.plot(x_axis,y2,label = "Error Twist Xerr[1]")
    plt.plot(x_axis,y3,label = "Error Twist Xerr[2]")
    plt.plot(x_axis,y4,label = "Error Twist Xerr[3]")
    plt.plot(x_axis,y5,label = "Error Twist Xerr[4]")
    plt.plot(x_axis,y6,label = "Error Twist Xerr[5]")
    plt.title(f"Overshoot, Kp value:{kp_multiplier},Ki value:{ki_multiplier}")  
    plt.legend()
    plt.show()
    

if __name__ == "__main__": # the main function
    logging.info('python3 Overshoot.py')
    logging.info('Starting the Main Program for Overshoot!')
    Full_program()

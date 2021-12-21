'''
This function takes in the current conifguration and the joint speeds to calculate the future configuration which is the new joint angles, wheel angles and chassis configuration

To run this code, run the following in the terminal

python3 Milestone1.py
'''

import numpy as np
import modern_robotics as mr
from numpy import pi

def check_velocity(velocity,velocity_limit):
    '''
    Function:   Check the velocity array for any velocities greater, or less than, the max and 
                change the elocity if any velcoity magnitude is greater than the max.
    Arguments: 
        velocity-The array of velocities in meters per second
        velocity_limit-The maximim velocity in meters per second
    Return:
        velocity-Array of velocity values where all values are less than the maximum velocity in meters per second

    '''
    #Loop through every index in the velocity array
    for i in range(len(velocity)):
        #If velocity magnitude is greater than the limit, change the values to the maximum velocity
        if velocity[i] > velocity_limit:
            velocity[i] = velocity_limit
        #If velocity magnitude is less than the negative version of the limit, change the values to the maximum negative velocity
        if velocity[i] < -velocity_limit:
            velocity[i] = -velocity_limit
    return velocity

def NextState(current_configuration,joint_speeds,timestep,max_speed):
    '''
    Function: This takes in the current configuration with the joint speeds to calculate the next configuration varaibles which are the joint angles, wheel angles and the chassis configuration
    Arguments:
        current_configuration-Current configuration of robot described in 12 vectors
        joint_speeds-Arm joint speeds for the arm and wheel in a 9-vector 
        timestep-The integration stepsize 
        max_speed- Maximum angular speed for the arm or wheels
    Return:
        future_config-Configuration of the robot in the future in the for of a 12-vector
    '''
    joint_speeds = check_velocity(joint_speeds,max_speed)
    # print(joint_speeds)
    #Create an empty array to place calculated future angles and configurations 
    future_config = np.zeros(12)
    #Parameters of the youBot
    radius = 0.0475 
    l = 0.235
    w = 0.15
    #The chasis configuration values, the first 3 values in current_configuration
    chasis_configuration = current_configuration[:3]
    #The arm joint angles, the next 5 values in current_configuration
    arm_angles = current_configuration[3:8]
    #The wheel joint angles, the last 5 values in current_configuration
    wheel_angles = current_configuration[8:12]
    #Gripper state, 0 for open gripper
    gripper_state = 0
    dt = timestep

    #The joint speeds
    arm_speeds = joint_speeds[:5]
    #The wheel speeds
    wheel_speeds = joint_speeds[5:9]
    
    #Calculate the new theta values 
    new_arm_angles = arm_angles + arm_speeds * dt
    #Calculate the new wheel values 
    new_wheel_angles = wheel_speeds * dt +wheel_angles

    #Create the H matrix 
    H = np.array([ [-1/(l+w),1/(l+w),1/(l+w),-1/(l+w)],[1,1,1,1],[-1,1,-1,1] ])

    #calculate the body twist
    Vb = ((radius/4) * (H@wheel_speeds) *dt)
    #Get the change in coordiantes
    wbz,vbx,vby = Vb
    #Make array of zeros for the cahnge in qb
    delta_qb = np.array([0,0,0])
 
    if wbz == 0:
        delta_qb = np.array([0,vbx,vby])
    else:
        delta_qb = np.array([wbz,(vbx*np.sin(wbz)+vby*(np.cos(wbz)-1))/wbz,(vby*np.sin(wbz)+vbx*(1-np.cos(wbz)))/wbz])
    #Get the phi of the chassis from the configuration
    phi_k = current_configuration[0]
    #Calculate the change in q
    delta_q = (np.array([ [1,0,0],[0,np.cos(phi_k),-np.sin(phi_k)],[0,np.sin(phi_k),np.cos(phi_k)] ]) ) @ delta_qb.T
    # print(delta_q)
    #Insert the values for the future congifugration 
    future_config[:3] = delta_q+chasis_configuration
    future_config[3:8] = new_arm_angles
    future_config[8:12] = new_wheel_angles


    return future_config

if __name__ == "__main__": # the main function
    current_configuration = np.array([0,0,0,0,0,0,0,0,0,0,0,0])

    #Tests
    joint_speeds = np.array([0,0,0,0,0,10,10,10,10]) #Test1
    # joint_speeds = np.array([0,0,0,0,0,-10,10,-10,10]) #Test2
    # joint_speeds = np.array([1,1,1,1,1,-10,10,10,-10]) #Test3
    #Timestep and max speed
    timestep = 0.01
    max_speed = 5

    append_config = []

    for i in range(100):
        current_configuration = NextState(current_configuration,joint_speeds,timestep,max_speed)
        append_config.append(current_configuration)
    np.savetxt("NextState.csv", append_config, delimiter= ",")



'''
This function calculates the control laws by calculating the end effector twist and the pseudoinverse of Je and matrix multiplying them together
How to run the code

To run the code, simply copy and paste the following line to a terminal (linux) to run the script. One can also press the play button if using Visual Studo Code

python3 Milestone3.py
'''
from modern_robotics.core import MatrixLog6
import numpy as np
import modern_robotics as mr
from numpy import pi
import numpy.linalg

def test_joint_limits(J_arm,thetalist):
    '''
    Function-
    Arguments-
        J_arm- The jacobian of the arm, after being transposed
        thetalist- the joint angles of the arm
    Return-
        Return J_arm with the 'rows' made 0 if the joint is outside the joint limits
    '''
    #If the joint values go below a certain joint value, make the joint's row 0(Note that it would be column but it was transposed)
    
    if thetalist[0] < -2.5 or thetalist[0]>2.5:
        J_arm[0] = J_arm[0]*0
    if thetalist[1] < -1 or thetalist[1]>1:
        J_arm[1] = J_arm[1]*0
    if thetalist[2] < -2 or thetalist[2]>2:
        J_arm[2] = J_arm[2]*0
    if thetalist[3] < -5 or thetalist[3]>5:
        J_arm[3] = J_arm[3]*0
    if thetalist[4] < -2.5 or thetalist[4]>2.5:
        J_arm[4] = J_arm[4]*0
    return J_arm
    
def Feedbackcontrols(X,X_d,X_d_next,Kp,Ki,dt,X_err_integral,configuration):
    '''
    Function- Takes in the configuration and calculates the controls based on the Kp,Ki and other arguments.
    Arguments-
        X- The current actual end effector configuration 
        X_d- The current end effector reference configuration
        X_d_next- The end effector reference configuration at the next timestep in the reference trajectory
        Kp- Proportional gain
        Ki_plus- Integral gain
        dt- Timestep between reference trajetory configurations, seconds
        X_err_integral- Error Twist
        configuration- phi,x,y,and joints 1 through 5

    return-
        V- The commanded end-effector twist in the end effector frame (e)
        V_d- Feedforward twist
        Ad_X_inv_X_d_Vd- Adjoint of the x inverse and feedforward twist
        X_err- Twist error
        controls- wheel and theta angles
        X_err_integral- The error intergral of the twist
    '''
    #Screw Axis for the 5 Joints
    Blist = np.array([[0,0,1,0,0.033,0],
                  [0,-1,0,-0.5076,0,0],
                  [0,-1,0,-0.3526,0,0],
                  [0,-1,0,-0.2176,0,0],
                  [0,0,1,0,0,0]]).T
    #List of theta values from the current configuation
    thetalist = np.array(configuration[3:8])

    #The fixed offset from the chasis frame to the base frame
    T_b0 = np.array([[1,0,0,0.1662],[0,1,0,0],[0,0,1,0.0026],[0,0,0,1]]) 
    #The end effector frame relative to the base frame
    M_0e = np.array([[1,0,0,0.033],[0,1,0,0],[0,0,1,0.6546],[0,0,0,1]])
    #End effector relative to the base of the arm
    T_0e = mr.FKinBody(M_0e,Blist,thetalist)
    #Inverse of T_B0to get base of arm relative to the chassis
    T_b0_inv = mr.TransInv(T_b0)
    #Inverse of T_0e to get base of arm relative to the end effector
    T_0e_inv = mr.TransInv(T_0e)
    #youBot parameters
    radius = 0.0475 
    l = 0.235
    w = 0.15
    #Calculate the inverses for X,X_d 
    X_inv =  mr.TransInv(X)
    X_d_inv = mr.TransInv(X_d)
    #Create the X_err in matrixlog6 form
    X_err_mat = mr.MatrixLog6(X_inv@X_d)
    #Make x_err to vector
    X_err = mr.se3ToVec(X_err_mat)

    #Calculate feedforwad twist in matrixlog6 form then convert to vector
    V_d_mat = mr.MatrixLog6(X_d_inv@X_d_next)
    V_d = mr.se3ToVec(V_d_mat*(1/dt))

    #Calculate the Adjoint by using X_inv and X_d then multiply by V_d
    Ad_X_inv_X_d = mr.Adjoint(X_inv@X_d)
    Ad_X_inv_X_d_Vd = Ad_X_inv_X_d@V_d
    #Calculate the x_err integral by adding the previous value to X_err times dt
    X_err_integral = X_err*dt + X_err_integral
    #Calculate the end effector twist
    V = Ad_X_inv_X_d_Vd + Kp@X_err + Ki@X_err_integral
    #Calculate the Jacobian of the base by multiplying the adjoint of T_e0 and T_0b and multiplying that product by F
    Ad_T_0e_inv_T_b0_inv = mr.Adjoint(T_0e_inv@T_b0_inv)
    F = (radius/4)* np.array([[0,0,0,0],
                [0,0,0,0],
                [-1/(l+w),1/(l+w),1/(l+w),-1/(l+w)],
                [1,1,1,1],
                [-1,1,-1,1],
                [0,0,0,0]])
    J_base = Ad_T_0e_inv_T_b0_inv@F
    #Use JacobianBody to get the Jacobian of the arm
    J_arm = mr.JacobianBody(Blist,thetalist)
    #Transpose the Jacobian of the arm
    J_arm =J_arm.T
    #Call the test_joint_limits function to test joints
    # J_arm = test_joint_limits(J_arm,thetalist)                                                             #Uncomment if you want to use test_joint_limits
    #Transpose the Jacobian of the arm again to revert the previous transpose
    J_arm =J_arm.T
    #Calculate Je by stacking jacobian of the base and the arm
    Je = np.hstack((J_base,J_arm))
    #use pinv of Je and multiply by end effector twist to get the controls
    controls = numpy.linalg.pinv(Je,1e-2) @ V
    #Round answers
    controls = np.around(controls,2)
    return V_d, Ad_X_inv_X_d_Vd, V, X_err, controls,X_err_integral


if __name__ == "__main__": # the main function
    #Initialize the x_err intergral with zeros
    X_err_integral = np.zeros(6)
    #Initialize the configuration with zeros
    configuration = np.array([0,0,0,0,0,0.2,-1.6,0])
    X_d = np.array([[0,0,1,0.5],
                    [0,1,0,0],
                    [-1,0,0,0.5],
                    [0,0,0,1]])
    X_d_next = np.array([[0,0,1,0.6],
                        [0,1,0,0],
                        [-1,0,0,0.3],
                        [0,0,0,1]])
    X = np.array([[0.17,0,0.985,0.387],
                [0,1,0,0],
                [-0.985,0,0.170,0.570],
                [0,0,0,1]])
    #Timestep
    dt = 0.01
    #Kp value when it 0 and the identity matrix below
    Kp = np.zeros((6,6))  
    # Kp = np.array([[1,0,0,0,0,0],
    #                  [0,1,0,0,0,0],
    #                  [0,0,1,0,0,0],
    #                  [0,0,0,1,0,0],
    #                  [0,0,0,0,1,0],
    #                  [0,0,0,0,0,1]])
    #Ki value when it 0 and the identity matrix below
    Ki = np.zeros((6,6))
    # Ki = np.array([[1,0,0,0,0,0],
    #                 [0,1,0,0,0,0],
    #                 [0,0,1,0,0,0],
    #                 [0,0,0,1,0,0],
    #                 [0,0,0,0,1,0],
    #                 [0,0,0,0,0,1]])


    V_d, Ad_X_inv_X_d_Vd, V, X_err, controls,X_err_integral  = Feedbackcontrols(X,X_d,X_d_next,Kp,Ki,dt,X_err_integral,configuration)

    print(controls)


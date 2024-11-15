import numpy as np
import sympy as sp
from scipy.optimize import fsolve
q=[np.radians(90),0,0,0]
q=[-1.1699, np.radians(39.6),np.radians(-81.7),np.radians(42)]
L1, L2, L3, L4 = 0.14, 0.13, 0.12, 0.1  
q1, q2, q3,q4 = sp.symbols('q1 q2 q3 q4')
vx, vy, vz = sp.symbols('vx vy vz')
def sysCall_init():
    sim = require('sim')
    self.target_position = -90 * (3.14159/180)
    self.required_duration = 5
    position1=sim.getObject("../1")
    position2=sim.getObject("../2")
    self.position3=sim.getObject("../3")
    position4=sim.getObject("../4")
    position5=sim.getObject("../5")
    EEHandle=sim.getObject("../Grip_respondable")
    distance21=sim.getObjectPosition(position2,position1)
    distance32=sim.getObjectPosition(self.position3,position2)
    distance43=sim.getObjectPosition(position4,self.position3)
    distance54=sim.getObjectPosition(EEHandle,position4)
    BaseHandle=sim.getObject("../base_link_visual")
    EEPosition=sim.getObjectPosition(EEHandle,BaseHandle)
    #print("EE Position: ")
    #print(EEPosition)
    
 
    #print(f"Theta 0: {np.degrees(qInverse[0]):.2f} degrees")
    #print(f"Theta 1: {np.degrees(qInverse[1]):.2f} degrees")
    #print(f"Theta 2: {np.degrees(qInverse[2]):.2f} degrees")
    #print(f"Theta 3: {np.degrees(qInverse[3]):.2f} degrees")
    forwardPosition = forward_kinematics_func()
#    print("Forward Position: ")
#    print(forwardPosition)
    q_dot = [0.1, 0.2, 0.3, 0.4]  
    
    
    J = jacobian_matrix()
    #print("Symbolic Jacobian Matrix J:")
    #print(J)


    #print("\nForward Velocity V_F:")
    #print(V_F)
    v_desired = [EEPosition[0], EEPosition[1], EEPosition[2]]

    print(v_desired)
    #q_dot_symbolic = inverse_kinematics_velocity(v_desired)
    q_dot_numeric = inverse_kinematics_velocity1(v_desired, q)
    print(q_dot_numeric)
    V_F = forward_velocity_kinematics(q_dot_numeric, q_dot)
    print(V_F)
    #print("Symbolic Joint Velocities (q_dot) required to achieve the desired end-effector velocity:")
    #print(q_dot_numeric)

def sysCall_actuation():
    curr_time = sim.getSimulationTime()
    if curr_time < self.required_duration:
        new_position = (curr_time/self.required_duration) * self.target_position
        sim.setJointTargetPosition(self.position3, new_position)
    else: 
        sim.setJointTargetPosition(self.position3, self.target_position)
        EEHandle=sim.getObject("../Grip_respondable")
        BaseHandle=sim.getObject("../base_link_visual")
        EEPosition=sim.getObjectPosition(EEHandle,BaseHandle)
#        print("EE Position: ")
#        print(EEPosition)
        qInverse=inverse_position_kinematics(EEPosition[0], EEPosition[1], EEPosition[2])
#        print("Inverse Position: ")
#        print(qInverse)
    pass

def sysCall_sensing():
    
    pass

def sysCall_cleanup():
   
    q[1]=90
    forward_kinematics_func()
    pass


def transformation_func(q,d,a,adg):
    return np.array([[np.cos(q),-np.sin(q)*np.cos(adg),np.sin(q)*np.sin(adg),a*np.cos(q)],
    [np.sin(q),np.cos(q)*np.cos(adg),-np.cos(q)*np.sin(adg),a*np.sin(q)],
    [0,np.sin(adg),np.cos(adg),d],
    [0,0,0,1]])
    
def forward_kinematics_func():
    position1=sim.getObject("../1")
    position2=sim.getObject("../2")
    position3=sim.getObject("../3")
    position4=sim.getObject("../4")
    position5=sim.getObject("../5")
    distance21=sim.getObjectPosition(position2,position1)
    distance32=sim.getObjectPosition(position3,position2)
    distance43=sim.getObjectPosition(position4,position3)
    distance54=sim.getObjectPosition(position5,position4)
    t1=transformation_func(q[0],distance21[2],0,np.pi/2)
    t2=transformation_func(q[1],0,distance32[0],0)
    t3=transformation_func(q[2],0,-distance43[0],0)
    t4=transformation_func(q[3],0,distance54[0],0)
    tTotal= np.eye(4)
    tTotal=np.dot(tTotal, t1)
    tTotal=np.dot(tTotal, t2)
    tTotal=np.dot(tTotal, t3)
    tTotal=np.dot(tTotal, t4)
    return(tTotal[0][3],tTotal[1][3],tTotal[2][3])

target=[0.0,0.0]
def forward_kinematics_func_symoblic():

    X = L1 * sp.cos(q1) + L2 * sp.cos(q1 + q2) + L3 * sp.cos(q1 + q2 + q3) + L4 * sp.cos(q1 + q2 + q3 + q4)
    Y = L1 * sp.sin(q1) + L2 * sp.sin(q1 + q2) + L3 * sp.sin(q1 + q2 + q3) + L4 * sp.sin(q1 + q2 + q3 + q4)
    Z = 0 
    return X, Y, Z
def inverse_position_kinematics(X, Y, Z):
    q1=np.arctan(Y/X)

    initial_guess = [0.0, 0.0, 0.0]
    target[0] = X**2+Y**2 
    target[1] = Z  
    solution = fsolve(equations, initial_guess)

    theta1, theta2, theta3 = solution
    return [q1,theta1, theta2, theta3]

def equations(angles):
    theta1, theta2, theta3 = angles
   
    x = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2) + L3 * np.cos(theta1 + theta2 + theta3)
    y = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2) + L3 * np.sin(theta1 + theta2 + theta3)
    
    return [x - target[0], y - target[1], theta1 + theta2 + theta3] 

def jacobian_matrix():
    
    X, Y, Z = forward_kinematics_func_symoblic()
    

    J11 = sp.diff(X, q1)
    J12 = sp.diff(X, q2)
    J13 = sp.diff(X, q3)
    J14 = sp.diff(X, q4)
    J21 = sp.diff(Y, q1)
    J22 = sp.diff(Y, q2)
    J23 = sp.diff(Y, q3)
    J24 = sp.diff(Y, q4)
    J31 = sp.diff(Z, q1)
    J32 = sp.diff(Z, q2)
    J33 = sp.diff(Z, q3)
    J34 = sp.diff(Z, q4)

    J = sp.Matrix([
        [J11, J12, J13, J14],
        [J21, J22, J23, J24],
        [J31, J32, J33, J34]
    ])
    
    return J
    
def forward_velocity_kinematics(q, q_dot):
    
    J = jacobian_matrix()
    
    J_num = J.subs({q1: q[0], q2: q[1], q3: q[2], q4: q[3]})
    
    J_np = np.array(J_num).astype(np.float64)
    
    V_F = J_np @ np.array(q_dot)
    return V_F
    
def inverse_kinematics_velocity(v_desired):
    
    J = jacobian_matrix()
    
    
    if J.shape[0] == J.shape[1]: 
        J_inv = J.inv()
    else:  
        J_inv = J.pinv()
    
   
    v = sp.Matrix(v_desired)
    
    
    q_dot = J_inv * v
    
    return q_dot

def inverse_kinematics_velocity1(v_desired, q):
    J = jacobian_matrix()
    J_func = sp.lambdify((q1, q2, q3, q4), J)  # Precompile symbolic Jacobian to numeric function
    J_num = np.array(J_func(q[0], q[1], q[2], q[3]))

    try:
        J_inv = np.linalg.pinv(J_num)  # Use numerical pseudoinverse
    except np.linalg.LinAlgError:
        raise ValueError("Jacobian is singular or ill-conditioned.")

    q_dot = J_inv @ v_desired
    return q_dot



    

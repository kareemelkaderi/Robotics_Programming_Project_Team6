import numpy as np

def sysCall_init():
    sim = require('sim')

    self.objectHandle = sim.getObject("../q1")
    self.target_position = 90 * (3.14159/180)
    self.required_duration = 5 #seconds
    sim.setJointPosition(self.objectHandle, self.target_position)
    x=forward_kinematics_func()
    print(x)
 
    
    t1 = [[np.cos(self.objectHandle), 0, np.sin(self.objectHandle), 0],
          [np.sin(self.objectHandle), 0, -1*np.cos(self.objectHandle), 0],
          [0, 1, 0, sim.getObject("../q2")-sim.getObject("../q1")],
          [0, 0, 0, 1]]
    # do some initialization here
    #
    # Instead of using globals, you can do e.g.:
    # self.myVariable = 21000000
def transformation_func(q,d,a,adg):
    return [[np.cos(q),-np.sin(q)*np.cos(adg),np.sin(q)*np.sin(adg),a*np.cos(q)],
    [np.sin(q),np.cos(q)*np.cos(adg),-np.cos(q)*np.sin(adg),a*np.sin(q)],
    [0,np.sin(adg),np.cos(adg),d],
    [0,0,0,1]]
def convertRadiansToDegree(x):
    return x*180/np.pi
def forward_kinematics_func():
    refFrame1Handle = sim.getObject('../q1/ReferenceFrame1') 
    refFrame2Handle = sim.getObject('../q2/ReferenceFrame2')
    refFrame3Handle = sim.getObject('../q3/ReferenceFrame3') 
    refFrame4Handle = sim.getObject('../q4/ReferenceFrame4')
    refFrame5Handle = sim.getObject('../q5/ReferenceFrame5')
    orientation1 = sim.getObjectOrientation(refFrame1Handle, -1)
    orientation2 = sim.getObjectOrientation(refFrame2Handle, -1)
    orientation3 = sim.getObjectOrientation(refFrame3Handle, -1)
    orientation4 = sim.getObjectOrientation(refFrame4Handle, -1)
    orientation5 = sim.getObjectOrientation(refFrame5Handle, -1)
    relativeOrientation1 = sim.getObjectOrientation(refFrame2Handle, refFrame1Handle)
    relativeOrientation2 = sim.getObjectOrientation(refFrame3Handle, refFrame2Handle)
    relativeOrientation3 = sim.getObjectOrientation(refFrame4Handle, refFrame3Handle)
    relativeOrientation4 = sim.getObjectOrientation(refFrame5Handle, refFrame4Handle)
    q1=relativeOrientation1[2];
    l1=sim.getObjectPosition(refFrame2Handle,refFrame1Handle)[2]
    q2=relativeOrientation2[2]
    q2=convertRadiansToDegree(q2)
    l2=sim.getObjectPosition(refFrame3Handle,refFrame2Handle)[2]
    l2=l2/np.sin(q2)
    q3=relativeOrientation3[2]
    q4=relativeOrientation4[2]
    l3=sim.getObjectPosition(refFrame4Handle,refFrame3Handle)[2]
    l4=sim.getObjectPosition(refFrame5Handle,refFrame4Handle)[2]
    delta4=l3+l4
    T12=transformation_func(q1,l1,0,1.5708)
    T23=transformation_func(q2,0,l2,0)
    T34=transformation_func(q3,0,0,-1.5708)
    T4EE=transformation_func(q4,delta4,0,0)
    z=(np.dot(T4EE,np.dot(T34,np.dot(T12,T23))))
    return [z[0][3],z[1][3],z[2][3]]
def sysCall_actuation():
    curr_time = sim.getSimulationTime()
    if curr_time < self.required_duration:
        new_position = (curr_time/self.required_duration) * self.target_position
        sim.setJointTargetPosition(self.objectHandle, new_position)
    else: 
        sim.setJointTargetPosition(self.objectHandle, self.target_position)
        EE_handle = sim.getObject("/endEffector_respondable")
        base_handle = sim.getObject("../base_link_visual")
        EE_position = sim.getObjectPosition(EE_handle, base_handle)
      #  print(EE_position)
    # put your actuation code here
    pass

def sysCall_sensing():
    # put your sensing code here
    pass

def sysCall_cleanup():
    # do some clean-up here
    pass

# See the user manual or the available code snippets for additional callback functions and details

def sysCall_init():
    sim = require('sim')

    self.objectHandle = sim.getObject("/q5")
    self.target_position = -45 * (3.14159/180)
    self.required_duration = 4 #seconds
    sim.setJointPosition(self.objectHandle, self.target_position)
    # do some initialization here
    #
    # Instead of using globals, you can do e.g.:
    # self.myVariable = 21000000

def sysCall_actuation():
    pass
def sysCall_sensing():
    curr_time = sim.getSimulationTime()
    if curr_time < self.required_duration:
        new_position = (curr_time/self.required_duration) * self.target_position
        sim.setJointTargetPosition(self.objectHandle, new_position)
    else: 
        sim.setJointTargetPosition(self.objectHandle, self.target_position)
        EE_handle = sim.getObject("/endEffector_respondable")
        base_handle = sim.getObject("/base_link_visual")
        EE_position = sim.getObjectPosition(EE_handle, base_handle)
    pass

def sysCall_cleanup():
    # do some clean-up here
    pass

# See the user manual or the available code snippets for additional callback functions and details

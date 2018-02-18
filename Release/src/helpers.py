import numpy as np

def negateDict(sensorState):    
    returnDict = {}
    for i,item in sensorState.items():
        if type(item)==dict:
            returnDict[i] = negateDict(item)
        else:
            returnDict[i] =-item
    return returnDict


def addDicts(DictA,DictB):
    for i,item in DictA.items():
        if type(item) == dict:
            addDicts(item,DictB[i])
        else:
            DictA[i] = item +DictB[i]
        
class Controller:
    def __init__(self):
        pass
    def update(self,droneState,clockState,target):
        pass
    def reset(self,droneState,clockState,target):
        pass
    def __call__(self):
        return np.array([0.,0.,0.])

class PhaseManager:
    def __init__(self,name):
        self.name = name
        self.status = False #false means inactive

    def setController(self,controller):
        """
        A controller is an object that takes upacked dicos as input,
        they are fed every loop the controller is active through the self.update() method
        The controller must have a self.reset() method called to reset their state
        The controller must generate a 3x1 vector of control inputs [vx,vy,vz] through __call__
        """
        self.controller = controller
    def toggle(self):
        self.status = not self.status
    
    def getController(self):
        return self.controller

class FlightManager(PhaseManager):
    def __init__(self,name,aboveGround,target,tol = 0.1,**kwargs):
        self.name = name
        self.target = np.array(target)
        self.count = 0
        self.tol = tol #position tolerance in meters
        self.newPhase = None
        self.status = False
        
    def update(self,droneState,clockState,**kwargs):
        #if we are steady around the target wait
        if all(np.abs(droneState['pos'] - self.target) < self.tol) and all(droneState['velLinear']<0.2):
            self.count += 1
        
        #if hover has been achieved for 1 second, land
        if self.count > kwargs['rate'] *1:
            print('phase must be changed because cout >20')
            self.controller.reset(clockState)
            self.status = False
            self.newPhase = 'safe'
        
        return 0

class safeMode(PhaseManager):
    def __init__(self,name,**kwargs):
        self.name = name
        self.count = 0
        self.status = True
        self.target = [0,0,0]
    def update(self,droneState,clockState,**kwargs):
        print(droneState)
        if droneState['pos'][2]>1: #count cycles above 1m
            self.count += 1
        else:
            self.controller.reset(droneState,clockState,[0,0,0])
            self.status = False
            self.newPhase = 'ramp'
        if self.count > 20:#send kill signal (will be passed back up)
            return 1
        
        
class MissionManager:
    """
    In charge of switching model states
    """
    def __init__(self, killFunc =None):
        initialPhase = safeMode('safe')
        initialPhase.setController(Controller())
        self.segments = {'safe' : initialPhase}
        self.currentState = 'safe'
        self.killfunc = killFunc
        
    def addSegment(self,phaseManager): #plan a new waypoint
        self.segments[phaseManager.name] = phaseManager
    def controller(self): #get current controller
        return self.segments[self.currentState].getController()
    def phase(self): #get phase manger for current segment
        return self.segments[self.currentState]
    def update(self,droneState,clockState,**kwargs): #update manager status (change phase)
        phase = self.segments[self.currentState]
        #check if the phase is still active
        if not self.segments[self.currentState].status: #change phase
            #phase change logic goes here
            if phase.newPhase == None:
                print('No phase to asign')
            else:
                name = phase.newPhase
                self.segments[name].status = True 
                self.currentState = name
                print('Changing phase')
            
        else: 
            status = phase.update(droneState,clockState,**kwargs)#update the phase 
            if status == 1:
                if self.killfunc == None:#If you forgot to define a kill function
                    raise Exception("Define a shudown function in __init__")
                else:
                    self.killfunc(0)#if not proceed
                
    def check(self):
        assert(self.segments[self.currentState].status)
def updateState(droneState,sensorState,clockState):
    #add sensor fusion here
    droneState['pos'] = sensorState['linearPose']
    droneState['pos'][2] =sensorState['height']
    droneState['quaternion'] = sensorState['quatPose']
    droneState['velLinear'] = sensorState['linearVel']
    droneState['velAngular'] = sensorState['angularVel']
    

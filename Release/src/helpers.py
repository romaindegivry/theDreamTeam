global droneState

def negateList(l,factor=1):
    if type(l)!=list:
        return -l
    r = [0]*len(l)
    for i,item in enumerate(l):
        if type(item) == list:
            r[i] = negateList(item)
        else:
            r[i] = - factor * l[i]
    return r

def addList(lA,lB):
    if type(lA)!=list:
        return lA+lB
    r = [0]*len(lA)
    for i,item in enumerate(lA):
        r[i] = addList(item,lB[i])

    return r
    

def negateDict(sensorState):    
    returnDict = {}
    for i,item in sensorState.items():
        if type(item)==dict:
            returnDict[i] = negateDict(item)
        else:
            returnDict[i] = negateList(item)
    return returnDict


def addDicts(DictA,DictB):
    for i,item in DictA.items():
        if type(item) == dict:
            addDicts(item,DictB[i])
        else:
            DictA[i] = addList(item,DictB[i])
            
           

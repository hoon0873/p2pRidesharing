import numpy as np
# Defining class of Driver and Passenger
class Driver:
    def __init__(self, origin, destin, etime, ltime, ptime, cap, cdev = 1, cdet = 3,val=100, rho=0,maxDev=2e9):
        self.ori = np.array(origin)
        self.des = np.array(destin)
        self.et = etime
        self.lt = ltime
        self.pt = ptime
        self.cdev = cdev
        self.cdet = cdet
        self.val = val
##        self.window = range(etime,ltime)
        self.cap = cap
        self.rho = rho
        self.maxDev = maxDev
    def __str__(self):
        return "Origin: "+str(self.ori)+"\nDestin: "+str(self.des)+"\nWindow,PT: "+str([self.et,self.lt,self.pt])+'\n'+\
               "cap: "+str(self.cap)+" cdev: "+str(self.cdev)+" cdet: "+str(self.cdet)+" val: "+str(self.val)+" rho "+str(self.rho)+'\n'

class Passenger:
    def __init__(self, origin, destin, etime, ltime,ptime, cdev = 1, cdet = 3, val =100, lamb=100,maxDev=2e9):
        self.ori = np.array(origin)
        self.des = np.array(destin)
        self.et = etime
        self.lt = ltime
        self.pt = ptime
        self.cdev = cdev
        self.cdet = cdet
        self.val = val
        self.lamb = lamb
        self.maxDev = maxDev
##        self.window = range(etime,ltime)
    def __str__(self):
        return "Origin: "+str(self.ori)+"\nDestin: "+str(self.des)+"\nWindow: "+str([self.et,self.lt,self.pt])+'\n'\
               "cdev: "+str(self.cdev)+" cdet: "+str(self.cdet)+" val: "+str(self.val)+" lambda: "+str(self.lamb)+'\n'
        
    def DIST(self):
        if int(abs(self.ori - self.des))==0: return 1
        else: return Distance(i.ori,i.des)

class Matching:
    def __init__(self, ):
        pass
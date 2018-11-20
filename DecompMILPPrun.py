import numpy as np
import networkx as nx
import pylab
import time
import random as rd
import gurobipy as grb
from Classes import Driver
from Classes import Passenger
from Preprocessing_4 import Preprocessing
from twoMILPPrun import MILP as mlp
import math

MUTE = 0 #set to 0 to MUTE
PRUN1 = 1 #set to 1 to do PRUNING
DECOMPOSITION = 1 #set to 1 to do decomposition
timeSwitch = 1
TIMELIMIT = 500

LAMBDA = 100
# Return distance between i's orgin to i's destination
def DIST(i):
    return Distance(i.ori,i.des)

def Distance(i,j):
    if type(i) != type(np.array([1])): i = np.array(i)
    if type(j) != type(np.array([1])): j = np.array(j)
##    if (i==j).all():
##        return 0.1
##    return int(np.linalg.norm(i-j))
    return math.ceil(np.linalg.norm(i-j))


def Decomp(drivers,reqs):
    BEGINTIME = time.clock()
    R = len(reqs)
    D = len(drivers)

    feasibleMat = np.zeros((D,R), dtype = int)

    for i in range(D):
        for j in range(R):
            t = drivers[i].et
            if drivers[i].et+Distance(drivers[i].ori,reqs[j].ori) < reqs[j].et and reqs[j].et+DIST(reqs[j])+Distance(reqs[j].des,drivers[i].des) <= drivers[i].lt:
                feasibleMat[i][j] = 1
            elif t+Distance(drivers[i].ori,reqs[j].ori) <= reqs[j].lt-DIST(reqs[j]) and  t+Distance(drivers[i].ori,reqs[j].ori)+DIST(reqs[j])+Distance(reqs[j].des,drivers[i].des) <= drivers[i].lt:
                feasibleMat[i][j] = 1

    objVal = 0

    if DECOMPOSITION != 0:
        Block,remainReq = Preprocessing(feasibleMat)


        objVal = LAMBDA*len(remainReq)
        
        for DRI, REQ in Block:
            if timeSwitch != 0 and time.clock() - BEGINTIME > TIMELIMIT:
                return objVal, ENDTIME-time.clock()
                
            driInd = [i for i in DRI]
            ds = [drivers[i] for i in DRI]
            rs = [reqs[j] for j in REQ]

            M,X,V,T,G,C = mlp(ds,rs)

            if M.status == 3:
                print("INPUT IS INFEASIBE")
                return 0,0

            objVal += V

            if MUTE != 0:
                for e in X:
                    for j in range(len(X[e])):
                        if X[e][j].x > 0:
                            print(e,driInd[j])
                print("\n")

    ENDTIME = time.clock()
    return objVal, ENDTIME-BEGINTIME

    print(feasibleMat)
    print(Block)
    


if __name__ == "__main__":
    R = 30
    D = 3
    reqs = []
    drivers = []
    PRECISION = 30
##    ##T = 150
    T = Distance((0,0), (0.7*PRECISION, 0.7*PRECISION))+PRECISION//2
##
##    
    BEGINTIME = time.clock()
##    print("DRIVERS")
    for i in range(D):
    ##    ori = (int(rd.uniform(0,1)*PRECISION),int(rd.uniform(0,1)*PRECISION))
    ##    des = (int(rd.uniform(0,1)*PRECISION),int(rd.uniform(0,1)*PRECISION))
        if i%2 == 0:
            ori = (int(rd.uniform(0,0.2)*PRECISION),int(rd.uniform(0,0.2)*PRECISION))
        if i%2 == 1:
            ori = (int(rd.uniform(0.8,1)*PRECISION),int(rd.uniform(0.8,1)*PRECISION))
        des = (int(rd.uniform(0.4,0.7)*PRECISION),int(rd.uniform(0.4,0.7)*PRECISION))
        
    ##    tim = int(rd.uniform(0,max(T/2-2-Distance(ori,des),1)))
    ####    delta = int(rd.uniform(0.8,1)*PRECISION)
    ##    delta = int(rd.uniform(0.5,1)*T)    
    ##    etim = min(tim+delta+Distance(ori,des),T-1)
        tim = 0
        etim = T
    ##    delta = int(rd.uniform(0,1)*PRECISION)
    ##    etim = min(tim+delta+Distance(ori,des),T-1)
        cap = 4
        drivers.append(Driver(ori,des,tim,etim,cap))
##        print(ori,des,tim,etim,cap)

##    print("REQUESTS")                   
    for i in range(R): 
    ##    ori = (int(rd.uniform(0,1)*PRECISION),int(rd.uniform(0,1)*PRECISION))
    ##    des = (int(rd.uniform(0,1)*PRECISION),int(rd.uniform(0,1)*PRECISION))
        if i%2 == 0:
            ori = (int(rd.uniform(0,0.2)*PRECISION),int(rd.uniform(0,0.2)*PRECISION))
        if i%2 == 1:
            ori = (int(rd.uniform(0.8,1)*PRECISION),int(rd.uniform(0.8,1)*PRECISION))
        des = (int(rd.uniform(0.4,0.7)*PRECISION),int(rd.uniform(0.4,0.7)*PRECISION))
        
        tim = int(rd.uniform(0,max(T/2-2-Distance(ori,des),1)))
        delta = int(rd.uniform(0,1)*PRECISION)
        etim = min(tim+delta+Distance(ori,des),T-1)
        ptim = tim+delta//2
    ##    tim = 0
    ##    etim = T-1
        reqs.append(Passenger(ori,des,tim,etim,ptim))
##        print(ori,des,tim,etim)


##    print("DRIVERS")
##    drivers.append(Driver((0,0),(7,0),0,20,4))
##    drivers.append(Driver((30,0),(17,0),20,40,4))
##    print("REQUESTS")
##    reqs.append(Passenger((1,0),(6,0),0,20,1))
##    reqs.append(Passenger((2,0),(5,0),0,20,2))
##    reqs.append(Passenger((3,0),(4,0),0,41,3))
##    reqs.append(Passenger((28,0),(19,0),13,35,24))


##    drivers = []
##    reqs =[]
##
##    drivers.append(Driver((0,0),(11,0),0,13,4))
##
##    reqs.append(Passenger((1,0),(10,0),0,15,5))
##    reqs.append(Passenger((2,0),(9,0),0,12,4))
##    reqs.append(Passenger((3,0),(8,0),0,12,3))
##    reqs.append(Passenger((4,0),(7,0),0,12,2))
##    reqs.append(Passenger((5,0),(6,0),0,12,1))
##


##    drivers = []
##    reqs =[]
##
    drivers = []
    reqs =[]

    drivers.append(Driver((3, 1),(26, 26),0,58,4))
    drivers.append(Driver((4, 4),(26, 28),0,58,4))
    drivers.append(Driver((4, 1),(24, 26),0,58,4))
    reqs.append(Passenger((0, 3),(26, 28),0,51,7))
    reqs.append(Passenger((2, 2),(25, 25),0,45,6))
    reqs.append(Passenger((0, 4),(29, 29),0,55,8))
    reqs.append(Passenger((3, 1),(25, 24),0,57,13))
    reqs.append(Passenger((0, 2),(24, 25),0,50,8))
    reqs.append(Passenger((2, 3),(28, 29),0,53,8))
    reqs.append(Passenger((0, 0),(27, 28),0,57,13))
    reqs.append(Passenger((5, 4),(25, 26),0,56,13))
    reqs.append(Passenger((5, 2),(24, 28),0,43,5))
    reqs.append(Passenger((3, 0),(27, 27),0,57,10))
    reqs.append(Passenger((1, 4),(26, 27),0,54,10))
    reqs.append(Passenger((1, 2),(29, 29),0,57,12))
    reqs.append(Passenger((4, 3),(25, 27),0,56,12))
    reqs.append(Passenger((4, 2),(25, 28),0,47,6))
    reqs.append(Passenger((4, 2),(24, 25),0,32,0))
    reqs.append(Passenger((5, 3),(26, 29),0,57,14))
    reqs.append(Passenger((4, 5),(26, 25),0,57,14))
    reqs.append(Passenger((3, 0),(29, 26),0,37,0))
    reqs.append(Passenger((1, 4),(28, 25),0,51,8))
    reqs.append(Passenger((3, 2),(26, 24),0,50,9))
    reqs.append(Passenger((5, 0),(25, 27),0,44,5))
    reqs.append(Passenger((1, 3),(28, 28),0,57,12))
    reqs.append(Passenger((3, 5),(24, 24),0,46,8))   
    
    v,t = Decomp(drivers,reqs)
    print(v,t)

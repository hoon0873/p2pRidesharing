import numpy as np
import networkx as nx
import pylab
import time
import random as rd
import gurobipy as grb
from Classes import Driver
from Classes import Passenger
from Preprocessing_4 import Preprocessing
from twoMILP import MILP as mlp
import math

MUTE = 0 #set to 0 to MUTE
PRUN1 = 0 #set to 1 to do PRUNING
DECOMPOSITION = 1 #set to 1 to do decomposition

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
            ds = [drivers[i] for i in DRI]
            rs = [reqs[j] for j in REQ]

            M,X,V,T,G,C = mlp(ds,rs)

            if M.status == 3:
                print("INPUT IS INFEASIBE")
                return 0,0

            objVal += V

    ENDTIME = time.clock()
    return objVal, ENDTIME-BEGINTIME

    print(feasibleMat)
    print(Block)
    


if __name__ == "__main__":
    R = 130
    D = 1000
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
####    DRIVERS
##    drivers.append(Driver( (4, 3) , (18, 13) , 0 , 26 , 4 ))
##    drivers.append(Driver( (24, 24) , (16, 14) , 0 , 26 , 4 ))
##    drivers.append(Driver( (25, 27) , (20, 14) , 0 , 26 , 4 ))
##    drivers.append(Driver( (29, 28) , (14, 14) , 0 , 26 , 4 ))
##    drivers.append(Driver( (29, 26) , (20, 16) , 0 , 26 , 4 ))
####    REQUESTS
##    reqs.append(Passenger( (2, 3) , (13, 20) , 0 , 25 , 14 ))
##    reqs.append(Passenger( (25, 25) , (20, 13) , 0 , 22 , 4 ))
##    reqs.append(Passenger( (4, 5) , (16, 13) , 0 , 16 , 1 ))
##    reqs.append(Passenger( (24, 28) , (18, 15) , 0 , 25 , 13 ))
##    reqs.append(Passenger( (5, 2) , (15, 16) , 0 , 25 , 6 ))
##    reqs.append(Passenger( (25, 25) , (18, 17) , 0 , 22 , 6 ))
##    reqs.append(Passenger( (1, 5) , (13, 15) , 0 , 22 , 3 ))
##    reqs.append(Passenger( (26, 25) , (12, 12) , 0 , 25 , 3 ))
##    reqs.append(Passenger( (5, 5) , (16, 14) , 0 , 25 , 12 ))
##    reqs.append(Passenger( (26, 28) , (13, 14) , 0 , 22 , 1 ))
##    reqs.append(Passenger( (5, 3) , (12, 12) , 0 , 23 , 6 ))
##    reqs.append(Passenger( (26, 29) , (12, 12) , 0 , 25 , 13 ))
##    reqs.append(Passenger( (3, 2) , (17, 15) , 0 , 25 , 10 ))
##    reqs.append(Passenger( (24, 29) , (14, 12) , 0 , 25 , 9 ))
##    reqs.append(Passenger( (4, 3) , (16, 16) , 0 , 25 , 5 ))
##    reqs.append(Passenger( (29, 25) , (12, 12) , 0 , 25 , 14 ))
##    reqs.append(Passenger( (0, 5) , (13, 20) , 0 , 25 , 3 ))
##    reqs.append(Passenger( (29, 28) , (16, 16) , 0 , 25 , 6 ))
##    reqs.append(Passenger( (3, 4) , (15, 14) , 0 , 22 , 3 ))
##    reqs.append(Passenger( (25, 25) , (14, 19) , 0 , 25 , 11 ))
##    reqs.append(Passenger( (4, 3) , (14, 12) , 0 , 25 , 8 ))
##    reqs.append(Passenger( (24, 25) , (19, 12) , 0 , 25 , 11 ))
##    reqs.append(Passenger( (5, 5) , (12, 16) , 0 , 25 , 6 ))
##    reqs.append(Passenger( (24, 24) , (17, 17) , 1 , 25 , 13 ))
##    reqs.append(Passenger( (1, 1) , (15, 12) , 0 , 25 , 4 ))
##    reqs.append(Passenger( (27, 29) , (20, 15) , 0 , 25 , 11 ))
##    reqs.append(Passenger( (4, 4) , (17, 18) , 0 , 24 , 2 ))
##    reqs.append(Passenger( (29, 29) , (19, 18) , 0 , 17 , 1 ))
##    reqs.append(Passenger( (4, 4) , (20, 16) , 0 , 25 , 8 ))
##    reqs.append(Passenger( (28, 24) , (19, 19) , 0 , 25 , 9 ))
##    reqs.append(Passenger( (0, 3) , (14, 16) , 0 , 25 , 4 ))
##    reqs.append(Passenger( (28, 26) , (16, 19) , 0 , 18 , 2 ))
##    reqs.append(Passenger( (4, 4) , (12, 17) , 0 , 15 , 0 ))
##    reqs.append(Passenger( (27, 28) , (18, 17) , 0 , 16 , 1 ))
##    reqs.append(Passenger( (2, 0) , (12, 15) , 0 , 25 , 13 ))
##    reqs.append(Passenger( (26, 26) , (18, 18) , 0 , 22 , 5 ))
##    reqs.append(Passenger( (3, 4) , (12, 17) , 0 , 19 , 2 ))
##    reqs.append(Passenger( (27, 28) , (15, 12) , 0 , 25 , 11 ))
##    reqs.append(Passenger( (3, 3) , (17, 14) , 0 , 25 , 5 ))
##    reqs.append(Passenger( (26, 25) , (12, 15) , 0 , 20 , 1 ))
##    reqs.append(Passenger( (5, 5) , (13, 15) , 0 , 16 , 2 ))
##    reqs.append(Passenger( (28, 24) , (20, 15) , 0 , 13 , 0 ))
##    reqs.append(Passenger( (5, 0) , (17, 13) , 0 , 25 , 8 ))
##    reqs.append(Passenger( (27, 28) , (18, 18) , 0 , 25 , 11 ))
##    reqs.append(Passenger( (1, 0) , (18, 13) , 0 , 25 , 5 ))
##    reqs.append(Passenger( (28, 24) , (20, 17) , 0 , 25 , 8 ))
##    reqs.append(Passenger( (4, 4) , (20, 17) , 0 , 25 , 3 ))


    
    v,t = Decomp(drivers,reqs)
    print(v,t)
    

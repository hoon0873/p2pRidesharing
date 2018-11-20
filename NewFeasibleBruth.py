import numpy as np
import networkx as nx
import pylab
import time
import random as rd
import gurobipy as grb
from Classes import Driver
from Classes import Passenger
import copy
import math

PRECISION = 10
MUTE = 0
LAMBDA = 0.2
LAMBDA1 = 0.2

M = 30

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


def travel(d, reqs, LB = None, LAMBDA = 0.2, UB = None):

    R = len(reqs)

    ind = [reqs[j].et for j in range(R)]
    ind = np.asarray(ind)
    ind = np.argsort(ind).tolist()
    Reqs = np.array([i for i in range(R)])
    Reqs = Reqs[ind]
    Reqs = Reqs.tolist()

    reqs = [reqs[j] for j in ind]

    seq = np.zeros((R*2), dtype=np.int)
    vis = np.zeros((R*2), dtype=np.int)
    cost = 1e9

    def bruteForceSearch(step,curTime,curLocation,curCost,curCap,cost,Rt):
##        if Rt == ['d',0]:
##            print(step,curTime,curLocation,curCost,curCap,cost,Rt)
        retRt = Rt[:]
        if curCost >= cost:
            return cost,retRt
        mincost = cost
        if step > R*2:
            mincost = min(mincost,
                          curCost+Distance(curLocation,d.des))
            return mincost,retRt+['d']
        for i in range(R):
            if vis[i] == 1 and curTime+Distance(curLocation,reqs[i].des)> reqs[i].lt:
                return 2e9, []
        if curTime+Distance(curLocation,d.des)>d.lt:
            return 2e0,[]
        for i in range(R):
            if vis[i] == -1: #already dropped off
                continue
            if vis[i] == 0:
                    
                vis[i]=1
                pickupTime = max(curTime+Distance(curLocation,reqs[i].ori),reqs[i].et)                
##                pickupTimes = max(curTime+Distance(curLocation,reqs[i].ori),reqs[i].et)
##                for pickupTime in range(pickupTimes,reqs[i].lt-DIST(reqs[i])+1):
                if pickupTime+DIST(reqs[i])> reqs[i].lt:
                    return 2e9,[]
                if curCap == 0:
                    continue
                newCost, newRt = bruteForceSearch(step+1,pickupTime,reqs[i].ori,
                                             curCost+Distance(curLocation,reqs[i].ori)+LAMBDA1*abs(reqs[i].pt-pickupTime)
                                             ,curCap-1,mincost,Rt+[i])
                if newCost < mincost:
                    retRt = newRt[:]
                    mincost = newCost
                vis[i]=0
            else:
                vis[i]=-1
                dropoffTime = curTime+Distance(curLocation,reqs[i].des)
##                dropoffTimes = curTime+Distance(curLocation,reqs[i].des)                
##                for dropoffTime in range(dropoffTimes,reqs[i].lt+1):
                newCost, newRt = bruteForceSearch(step+1,dropoffTime,reqs[i].des,
                                               curCost+Distance(curLocation,reqs[i].des)+LAMBDA1*max(0,dropoffTime-(reqs[i].pt+DIST(reqs[i])))
                                               ,curCap+1,mincost,Rt+[i])
                if newCost < mincost:
                    retRt = newRt[:]
                    mincost = newCost
                vis[i]=1
        return mincost, retRt

    cost,Rt= bruteForceSearch(1,d.et,d.ori,0,d.cap,int(2e9),['d'])
    
    if cost <1e9:
        return True, cost, Rt
    else:
        return False,0,None
                            


###################
if __name__ == "__main__":

##    testR = 10
##    oriD = (0,0)
##    desD = (0*PRECISION,1*PRECISION)
##    testT = 10*Distance(oriD,desD)
##    capD = 4
##    tD = 0
##    d = Driver(oriD,desD,tD,testT,capD)
##    reqs = []
##    for i in range(testR):
##        ori = (int(rd.uniform(0,1)*PRECISION),int(rd.uniform(0,1)*PRECISION))
##        des = (int(rd.uniform(0,1)*PRECISION),int(rd.uniform(0,1)*PRECISION))
##        tim = int(rd.uniform(0,max(testT-2-Distance(ori,des),1)))
##        delta = int(rd.uniform(0,1)*PRECISION)
##        etim = min(tim+delta+Distance(ori,des),testT-1)
##        ptim = tim+delta//2
##        reqs.append(Passenger(ori,des,tim,etim,ptim))
####        print(ori,des,tim,etim)
##
##
####    ori = (round(rd.uniform(0,1),PRECISION),round(rd.uniform(0,1),PRECISION))
####    des = (round(rd.uniform(0,1),PRECISION),round(rd.uniform(0,1),PRECISION))
####    tim = round(rd.uniform(0,max(testT-2-Distance(ori,des),1)),PRECISION)
####    delta = round(rd.uniform(0,1),PRECISION)
####    etim = max(tim+delta,testT-1)
####    reqs.append(Passenger(ori,des,tim,etim))
####    for i in range(testR):
####        ori = (rd.randint(0,10),rd.randint(0,10))
####        des = (rd.randint(0,10),rd.randint(0,10))
####    ##    tim = rd.randint(0,max(testT-2-Distance(ori,des),1))
####        tim = 0
####        delta = rd.randint(0,10)
####        etim = max(tim+Distance(ori,des)+delta,testT-1)
####        reqs.append(Passenger(ori,des,tim,etim))
####
####
####
####    for i in range(testR):
####        ori = (i,0)
####        des = (10,10-i)
####        tim = 0
####        etim = 20
####        reqs.append(Passenger(ori,des,tim,etim))
######
####    rr = Passenger((0,2),(0,6), 4, 67, 10)
####    r1 = Passenger((0,1),(0,4), 1, 67, 8)
####    reqs = [rr, r1]
####    reqs = [rr]
####        
####    print("DONE WITH INPUT")
####    r1 = Passenger(1,4,1,10)
####    r2 = Passenger(2,3,1,10)
####    r3 = Passenger(3,6,6,14)
####    r4 = Passenger(5,9,10,19)
####    reqs = [r1,r2,r3,r4]
####    reqs = [r1,r2]
##
##
##
##    b,c,m,x,W,Zd,Zo,Ye,Yl, Cost = travel(d,reqs)
##    if c != None:
##
####    print(Cost)
##        print(b)
##        print(c)
##
##
##    for i in range(len(x)):
##        for j in range(len(x[i])):
##            for t in x[i][j]:
##                if x[i][j][t].x > 0:
##                    print(i,j,t)


##    testR = 2
##    oriD = (0,0)
##    desD = (0,1*PRECISION)
##    testT = 10*Distance(oriD,desD)
##    capD = 4
##    tD = 0
##    d = Driver(oriD,desD,tD,testT,capD)
##    reqs = []
##
##
##    reqs.append(Passenger((0,1),(0,2),1,5,2))
##    reqs.append(Passenger((0,3),(0,4),1,20,2))
##


##    d = Driver((0, 0) ,(18, 0) ,0, 59, 4)
##    reqs = []
##    reqs.append(Passenger((1, 0), (19, 0) ,0, 41, 9))
##    reqs.append(Passenger((2, 0), (15, 0), 0, 36, 6))
##    reqs.append(Passenger((3, 0), (17, 0) ,4, 29, 5))
##    reqs.append(Passenger((7, 0), (20, 0) ,2 ,50, 14))
##
##
##    reqs = []
##    drivers = []
##    BIGNUMBER = 5
##
##    print("DRIVERS")
##    d = Driver((27, 27) ,(14, 17) ,0 ,59 ,4)
##
##    print("REQUESTS")
##    reqs.append(Passenger((27, 29) ,(18, 13) ,2 ,44 ,14))
####    reqs.append(Passenger((27, 24) ,(16, 16) ,8 ,41 ,18))
##    reqs.append(Passenger((25, 28) ,(15, 17) ,12 ,49 ,23))
##    reqs.append(Passenger((24, 28) ,(13, 15) ,0 ,37 ,10))
##    reqs.append(Passenger((27, 28), (16, 20) ,5 ,25 ,8))


    drivers = []
    reqs = []


##    DRIVERS
    drivers.append(Driver( (4, 0) , (19, 12) , 0 , 60 , 4 ))
    drivers.append(Driver( (27, 25) , (14, 19) , 0 , 60 , 4 ))
    drivers.append(Driver( (24, 25) , (13, 19) , 0 , 60 , 4 ))
    drivers.append(Driver( (27, 27) , (18, 13) , 0 , 60 , 4 ))
    drivers.append(Driver( (28, 24) , (15, 13) , 0 , 60 , 4 ))
##    REQUESTS
    reqs.append(Passenger( (5, 0) , (12, 20) , 0 , 41 , 9 ))
    reqs.append(Passenger( (26, 24) , (20, 20) , 6 , 17 , 7 ))
    reqs.append(Passenger( (0, 4) , (19, 20) , 2 , 34 , 5 ))
    reqs.append(Passenger( (24, 25) , (12, 14) , 7 , 36 , 13 ))
    reqs.append(Passenger( (2, 1) , (15, 13) , 0 , 30 , 6 ))


    

    d = drivers[4]
    indd = [1,3]
    reqss = [reqs[i] for i in indd]

    reqs = reqss
    
    b,v, Rt = travel(d,reqss)
##    b,v,m,x,W,Zd,Zo, Ye, Yl, Cost,timeW = travel(d,reqs)
    print(b,v)
##    print(x)
##    if x != None:

##    print(Cost)
##        print(b)
##        print(x)
##    for i in range(len(x)):
##        for j in range(len(x[i])):
##            for t in x[i][j]:
##                if x[i][j][t].x >0:
##                    print(i,j,t,x[i][j][t].x,Cost[i][j],timeW[j])

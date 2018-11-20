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


MUTE = 0
LAMBDA = 0.2
LAMBDA1 = 0.2
global BRUTECOUNT
global DPCOUNT

def DrawInstance(d,reqs):
    DG = nx.Graph()
    ORI = tuple(d.ori)
    DES = tuple(d.des)
    DG.add_node(ORI)
    DG.add_node(DES)
    DG.add_edge(ORI,DES)
    
    for i in reqs:
        ORI = tuple(i.ori)
        DES = tuple(i.des)
        DG.add_node(ORI)
        DG.add_node(DES)
        DG.add_edge(ORI,DES)

    pos={}
    for v in DG.nodes():
        pos[v] = (v[0],v[1])

    nx.draw(DG,pos,node_size=5)
    pylab.show()

def DistRt(d,reqs,Rt):
    N = len(reqs)+1
    VISIT = np.zeros(N, dtype=np.int)
    Dist = 0
    curLoc = (0,0)
    curTim = 0
    
    for i in Rt:
##        print(curTim,curLoc,Dist)
        if i == 'd' and VISIT[N-1] == 0:
            curLoc = d.ori
            curTim = d.et
            VISIT[N-1] = 1
        elif i == 'd' and VISIT[N-1] == 1:
            curTim = curTim + Distance(curLoc,d.des)
            Dist = Dist+Distance(curLoc,d.des)
            VISIT[N-1] = -1
        elif VISIT[i] == 0: #go to origin
            curTim = max(curTim+Distance(curLoc,reqs[i].ori),reqs[i].et)
            Dist = Dist + Distance(curLoc,reqs[i].ori)
            curLoc = reqs[i].ori
            VISIT[i] = 1
        elif VISIT[i] == 1: #go to destin
            curTim = curTim+Distance(curLoc,reqs[i].des)
            Dist = Dist + Distance(curLoc,reqs[i].des)
            curLoc = reqs[i].des
            VISIT[i] = -1

    return Dist,curTim
            
            

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
    global BRUTECOUNT
    global DPCOUNT
    R = len(reqs)
    
    ind = [reqs[j].et for j in range(R)]
    ind = np.asarray(ind)
    ind = np.argsort(ind).tolist()
##    print(ind)
    Reqs = np.array([i for i in range(R)])
    Reqs = Reqs[ind]
    Reqs = Reqs.tolist()

    reqs = [reqs[j] for j in ind]
    
    BRUTECOUNT = 0
    DPCOUNT = 0
##    invInd = np.zeros(len(reqs), dtype=np.int)
##    for i in range(len(ind)):
##        invInd[ind[i]] = i
##
##
##    print(invInd)

    if len(reqs)>=6 and MUTE!=0: DrawInstance(d,reqs)    

    seq = np.zeros((R*2), dtype=np.int)
    vis = np.zeros((R*2), dtype=np.int)
    cost = 1e9


    def TrueCost(TreeCost,Rt,disCost): #TripDP
        global DPCOUNT
        DPCOUNT += 1
##        print(Rt)
        N = len(Rt)
        timCost = 0
        curDist = 0
        VISIT = np.zeros(R+1,dtype=np.int)
        CostT = [{} for COUNT in range(N)]
        curPas = set()
        for v in range(N-1,-1,-1):
            if v == N-1:
                VISIT[R] = 1
                tBound = d.lt
                for t in range(disCost-curDist, tBound+1):
                    CostT[v][t] = 0 #LAMBDA1*abs(d.pt+DIST(d)-t)
                prevTBound = tBound
                prevPd = d.des
                prevP = d
                pvet = d.et
                continue
            elif v == 0:
                VISIT[R] = -1
                tBound = d.lt-DIST(d)
                p = d
                pd = d.ori
                rtCost = 2e9
                for t in range(disCost-curDist,tBound+1):
                    for tp in range(max(pvet,t+Distance(pd,prevPd)),prevTBound+1):
                        if CostT[1][tp] < rtCost:
##                            print(v+1,tp,CostT[1][tp])
                            rtCost = CostT[1][tp]
                timCost = rtCost
                break
            elif VISIT[Rt[v]] == 0: #destination
                VISIT[Rt[v]] = 1
                p = reqs[Rt[v]]
                curPas.add(p)
                pd = p.des
                ppt = p.pt+DIST(p)
                tBound = p.lt
                isDest = True
                pet = p.et+DIST(p)
            elif VISIT[Rt[v]] == 1: #origin
                VISIT[Rt[v]] = -1
                p = reqs[Rt[v]]
                pd = p.ori
                ppt = p.pt
                tBound = p.lt-DIST(p)
                isDest = False
                pet = p.et
            curDist = curDist+Distance(pd,prevPd)
            for t in range(max(disCost-curDist,pet), tBound+1):
                minC = 2e9
                for tp in range(max(t+Distance(pd,prevPd),pvet),prevTBound+1):
                    sumDetCost = 0
                    for rp in curPas:
                        sumDetCost += rp.cdet*(tp-(t+Distance(pd,prevPd)))
                    sumDetCost += d.cdet*1/2*(tp-(t+Distance(pd,prevPd)))
                    if CostT[v+1][tp]+sumDetCost < minC:
##                        print(v,t,v+1,tp,CostT[v+1][tp]+sumDetCost,sumDetCost,len(curPas))
                        minC = CostT[v+1][tp]+sumDetCost
                if isDest: CostT[v][t] = 0+minC
                else:
                    CostT[v][t] = p.cdev*abs(ppt-t)+minC
##                    print(p.cdev*abs(ppt-t), ppt, t, p.ori)
            if not isDest:
                curPas.remove(p)
                
            prevTBound = tBound
            prevPd = pd
            prevP = p
            pvet = pet
                
##        print(timCost)
        if MUTE !=0:
            for i in range(N):
                print(CostT[i])

            print(d,Rt,timCost,TreeCost+timCost)
        return TreeCost+timCost

##    print('s', 't', 'Loc', 'c', 'cap', 'GUB', 'RT')
    def bruteForceSearch(step,curTime,curLocation,curCost,curCap,cost,Rt,totalDist):
        global BRUTECOUNT
##        print(BRUTECOUNT)
        BRUTECOUNT+=1
##        print(totalDist, Rt)
##        if Rt == ['d',0,1]:
##            print(step,curTime,curLocation,curCost,curCap,cost,Rt)
##        print(step,curTime,curLocation,curCost,curCap,cost,Rt)
##        print(vis)
        retRt = Rt[:]
        if curCost >= cost:
            return cost,retRt
        mincost = cost
        if step > R*2: # We visited all requests go to driver's destination
##            print(step,curTime,curLocation,curCost,curCap,cost,Rt)
            curCost = curCost+d.cdet*Distance(curLocation,d.des)
            totalDist = totalDist+Distance(curLocation,d.des)
            retRt = retRt+['d']
            nowCost = TrueCost(curCost,retRt,totalDist)
            mincost = min(mincost,
                          nowCost)
            return mincost,retRt
        for i in range(R):
            if vis[i] == 1:
                if curTime+Distance(curLocation,reqs[i].des)> reqs[i].lt:
                    return 2e9, []
                if curCost+Distance(curLocation,reqs[i].des)>=cost:
                    return cost,retRt
        if curTime+Distance(curLocation,d.des)>d.lt:
            return 2e9,[]
        if curCost+Distance(curLocation,d.des) >= cost:
            return cost,retRt
        for i in range(R):
##            print(i,vis[i], step,curTime,curLocation,curCost,curCap,cost,Rt )
            if vis[i] == -1: #already dropped off
                continue
            if vis[i] == 0:
                vis[i]=1
                pickupTime = max(curTime+Distance(curLocation,reqs[i].ori),reqs[i].et)

                    
                if pickupTime+DIST(reqs[i])> reqs[i].lt and pickupTime+DIST(reqs[i])+Distance(reqs[i].des,d.des) > d.lt:
                    vis[i] = 0
                    return 2e9,[]
                if curCap == 0:
                    vis[i] = 0
                    continue
##                print("here origin",i)
                curDetCost = curCost
                for rp in range(R):
                    if vis[rp] == -1 or vis[rp] == 0:continue
                    if rp == i: continue
                    curDetCost += reqs[rp].cdet*Distance(curLocation,reqs[i].ori)
                curDetCost += d.cdet*1/2*Distance(curLocation,reqs[i].ori)
##                totalDist += Distance(curLocation,reqs[i].ori)
                newCost, newRt = bruteForceSearch(step+1,pickupTime,reqs[i].ori,
                                             curDetCost #+LAMBDA1*abs(reqs[i].pt-pickupTime)
                                             ,curCap-1,mincost,Rt+[i],totalDist + Distance(curLocation,reqs[i].ori))
                if newCost < mincost:
                    retRt = newRt[:]
                    mincost = newCost
                vis[i]=0
            else:
                vis[i]=-1
                dropoffTime = curTime+Distance(curLocation,reqs[i].des)
##                print("here destin",i)
                curDetCost = curCost
                for rp in range(R):
                    if rp == i: curDetCost += reqs[rp].cdet*Distance(curLocation,reqs[i].des)
                    if vis[rp] == -1 or vis[rp] == 0: continue
                    curDetCost += reqs[rp].cdet*Distance(curLocation,reqs[i].des)
                curDetCost += d.cdet*1/2*Distance(curLocation,reqs[i].des)
##                totalDist += Distance(curLocation,reqs[i].des)
                newCost, newRt = bruteForceSearch(step+1,dropoffTime,reqs[i].des,
                                               curDetCost #+LAMBDA1*max(0,dropoffTime-(reqs[i].pt+DIST(reqs[i])))
                                               ,curCap+1,mincost,Rt+[i], totalDist+Distance(curLocation,reqs[i].des))
                if newCost < mincost:
                    retRt = newRt[:]
                    mincost = newCost
                vis[i]=1
        return mincost, retRt

    cost,Rt= bruteForceSearch(1,d.et,d.ori,0,d.cap,int(2e9),['d'],0)

##
    if MUTE != 0:
        print("RECURSIVE COUNT: ",BRUTECOUNT)
        print("DP COUNT: ", DPCOUNT)
        print(cost)
    for i in range(len(Rt)):
        if Rt[i] == 'd':
            continue
        else:
            Rt[i] = ind[Rt[i]]
        
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

##    drivers.append(Driver((3, 2),(29, 26),0,58,4))
    drivers.append(Driver((5, 2),(24, 24),0,58,4))
##    drivers.append(Driver((4, 1),(29, 27),0,58,4))
    reqs.append(Passenger((2, 3),(8, 9),6,40,18))
    reqs.append(Passenger((5, 7),(10, 13),9,40,20))
    reqs.append(Passenger((10, 10),(15, 19),10,28,13))
    reqs.append(Passenger((18, 16),(21, 24),14,45,25))
    reqs.append(Passenger((20, 24),(28, 27),10,42,21))
    reqs.append(Passenger((2, 4),(6, 6),7,41,21))
    reqs.append(Passenger((7, 9),(14, 12),12,32,18))
    reqs.append(Passenger((10, 12),(18, 15),15,31,18))
    reqs.append(Passenger((19, 17),(20, 21),1,21,8))
    reqs.append(Passenger((22, 23),(25, 28),2,14,5))
    reqs.append(Passenger((1, 2),(8, 8),12,39,20))
    reqs.append(Passenger((9, 8),(13, 10),6,39,20))
    reqs.append(Passenger((12, 10),(17, 16),15,43,25))
    reqs.append(Passenger((15, 15),(24, 22),2,15,2))
    reqs.append(Passenger((23, 23),(28, 27),6,29,14))
    reqs.append(Passenger((3, 3),(6, 8),6,36,18))
    reqs.append(Passenger((5, 8),(11, 10),19,36,24))
    reqs.append(Passenger((12, 11),(18, 19),5,33,14))
    reqs.append(Passenger((18, 18),(21, 23),13,32,19))
    reqs.append(Passenger((20, 22),(27, 29),5,42,18))


    

    d = drivers[0]
    indd = [0,1,5,10,15,16]
    reqss = [reqs[i] for i in indd]


    drivers = []
    reqs = []

    
##    drivers.append(Driver((2, 1),(24, 25),0,58,4))
    drivers.append(Driver((3, 4),(28, 25),0,58,4))
##    drivers.append(Driver((2, 2),(27, 27),0,58,4))
    reqs.append(Passenger((0, 3),(8, 7),14,39,22))
    reqs.append(Passenger((9, 9),(13, 14),18,44,27))
    reqs.append(Passenger((13, 12),(17, 16),6,30,15))
    reqs.append(Passenger((18, 15),(20, 22),10,37,19))
    reqs.append(Passenger((20, 22),(29, 29),13,54,27))
    reqs.append(Passenger((4, 3),(6, 5),15,26,19))
    reqs.append(Passenger((8, 7),(11, 14),2,30,12))
    reqs.append(Passenger((13, 11),(17, 18),16,30,18))
    reqs.append(Passenger((19, 17),(21, 23),10,33,18))
    reqs.append(Passenger((22, 20),(28, 25),12,24,14))
    reqs.append(Passenger((3, 4),(6, 5),18,32,23))
    reqs.append(Passenger((6, 7),(11, 13),4,36,16))
    reqs.append(Passenger((12, 13),(18, 18),4,35,15))
    reqs.append(Passenger((16, 17),(21, 23),7,36,17))
    reqs.append(Passenger((21, 20),(25, 25),9,29,15))
    reqs.append(Passenger((1, 3),(5, 5),20,33,24))
    reqs.append(Passenger((7, 8),(14, 13),5,41,18))
    reqs.append(Passenger((12, 13),(16, 19),18,26,18))
    reqs.append(Passenger((16, 19),(23, 24),4,58,11))
    reqs.append(Passenger((23, 20),(25, 27),3,58,8))
    reqs.append(Passenger((1, 2),(6, 5),0,35,14))

    d = drivers[0]
    indd = [2,7,11,13,16,18]
    reqss = [reqs[i] for i in indd]





    
    reqs = reqss


    DrawInstance(d,reqs)
    
    b,v, Rt = travel(d,reqss)
##    b,v,m,x,W,Zd,Zo, Ye, Yl, Cost,timeW = travel(d,reqs)
    print(Rt)
    print(b,v)
##    print("SHOULD BE 52")
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

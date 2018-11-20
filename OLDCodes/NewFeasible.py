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
    V = set(range(2*len(reqs)+2))


    DriverOrigin = len(V)-2
    DriverDestin = len(V)-1
    SOURCE = len(V)
    SINK = len(V)+1
    N = SINK+1
    VO = set(range(0,R))
    VD = set(range(R,2*R))


    T = len(V)



    EPSILON = 1
##    TIMERANGE = set()
    TIMERANGE = range(T+1)
    InstTime = range(1,len(V))
    TIME = 0


    Cost = np.zeros([N,N])
    Time = np.zeros([N,N])
    for i in range(N):
        for j in range(N):
            if   i <   R: u = reqs[i].ori
            elif i < 2*R: u = reqs[i-R].des
            elif i ==2*R: u = d.ori
            else        : u = d.des
            
            if   j <   R: v = reqs[j].ori
            elif j < 2*R: v = reqs[j-R].des
            elif j ==2*R: v = d.ori
            else        : v = d.des
##            print(traCost)

            if i == SOURCE or j == SINK:
                Cost[i][j] = 0
                Time[i][j] = 1
            elif i == SINK or j == SOURCE:
                Cost[i][j] = 10
                Time[i][j] = 1
            elif np.array_equal(u,v):
                Cost[i][j] = 0
                if i%R == j%R: Time[i][j] = 1
                else: Time[i][j] = 1
            else:
                Cost[i][j] = Distance(u,v)
                Time[i][j] = Distance(u,v)
            
##            print(i,j,u,v,Cost[i][j])

    m = grb.Model("travel")

######### CREATE VARIABELS ####################################
    x = []
    for i in range(N):
        x.append([])
        for j in range(N):
            x[i].append([])
            x[i][j] = m.addVars(TIMERANGE, vtype=grb.GRB.BINARY)

    W = m.addVars(TIMERANGE, vtype=grb.GRB.CONTINUOUS, lb = 0)
    Zd = m.addVars(TIMERANGE, vtype=grb.GRB.CONTINUOUS, lb = 0) #how early we leave vertex 
    Zo = m.addVars(TIMERANGE, vtype=grb.GRB.CONTINUOUS, lb = 0) #how late we arrive at origin
    Ye = m.addVars(TIMERANGE, vtype=grb.GRB.CONTINUOUS, lb = 0) #Deviation Leftward
    Yl = m.addVars(TIMERANGE, vtype=grb.GRB.CONTINUOUS, lb = 0) #Deviation Rightward
    Ys = m.addVars(TIMERANGE, vtype=grb.GRB.CONTINUOUS, lb = 0) #Slack for Destin
########### CREATE COSNTRAINTS #################################
##    print(Time)

    def posT(v,t):
        for u in range(N):
            if u == SOURCE:
                if t == 0:
                    yield u
            elif u == SINK:
                continue
            elif t-Time[u][v] >= d.et:
                yield u
    def vNot(u):
        return [x for x in V if x!=u]

    def TimeRange(t=T, lb=-1,end = True):
        if end == False: return [x for x in InstTime if x<t and x>lb]
        return [x for x in InstTime if x<= t and x>lb]


    def retLoc(i):
        if   i <   R: u = reqs[i].ori
        elif i < 2*R: u = reqs[i-R].des
        elif i ==2*R: u = d.ori
        elif i ==2*R+1: u = d.des
        elif i ==2*R+2: u='SOURCE'
        else: u = "SINK"

        return u
    def retTim(i):
        if   i <   R: u = reqs[i].et
        elif i < 2*R: u = reqs[i-R].lt
        elif i ==2*R: u = d.et
        elif i ==2*R+1: u = d.lt
        elif i ==2*R+2: u = 0
        else: u = T

        return u

    def retInd(i):
        if   i <   R: u = i
        elif i < 2*R: u = i-R
        elif i <=2*R+1: u = 'd'
        elif i ==2*R+2: u='SOURCE'
        else: u = "SINK"

        return u

    def retWinSiz(i):
        if   i <   R: u = reqs[i]
        elif i < 2*R: u = reqs[i-R]
        elif i <=2*R+1: u = d
        elif i ==2*R+2: return 0
        else: return d.lt-d.et

        return u.lt-u.et

    def retPrefTim(i):
        if   i <   R: u = reqs[i].pt
        elif i < 2*R: u = reqs[i-R].pt+Distance(reqs[i-R].ori,reqs[i-R].des)
        elif i ==2*R: u = d.et
        elif i ==2*R+1: u = d.lt
        elif i ==2*R+2: u = 0
        else: u = T

        return u    
    
    timeW = np.zeros(N)
    WinSize = np.zeros(N)
    timeP = np.zeros(N)
    for i in range(N):
        timeW[i] = retTim(i)
        WinSize[i] = retWinSiz(i)
        timeP[i] = retPrefTim(i)

    
    for v in range(N):
        for t in TIMERANGE:
            if t==0: continue
                
            m.addConstr(grb.quicksum(x[u][v][t-1]
                                     for u in range(N))
                        == grb.quicksum(x[v][w][t]
                                       for w in range(N)))
        
##        m.addConstr(grb.quicksum(x[u][v][T-1]
##                                 for u in V)
##                    == grb.quicksum(x[v][w][T]
##                                    for w in range(N)))

        m.addConstr(x[SOURCE][DriverOrigin][0] == 1)
        m.addConstr(x[DriverDestin][SINK][T] == 1)

##    reqs = [rr, r1]
        # Origin before Destination
    for j in range(R):
        for t in TIMERANGE:
            m.addConstr(grb.quicksum(x[j][v][tau]
                                     for v in V
                                     for tau in range(t))
                        >= grb.quicksum(x[R+j][u][t]
                                        for u in V))

    for u in V:
        m.addConstr(grb.quicksum(x[u][v][t]
                                 for v in range(N)
                                 for t in TIMERANGE)
                    == 1)
    
    for t in TIMERANGE:

        # Capacity Contraint
        m.addConstr(grb.quicksum(x[u][v][tau]
                                 for u in VO
                                 for v in vNot(u)
                                 for tau in range(t))
                    -
                    grb.quicksum(x[u][v][tau]
                                 for u in VD
                                 for v in vNot(u)
                                 for tau in range(t))
                    <= d.cap)


        
        # For each time we are satisfying one edge
        m.addConstr(grb.quicksum(x[u][v][t]
                                 for u in range(N)
                                 for v in range(N))
                    == 1)


        for u in range(N-1):
            m.addConstr(x[u][u][t] == 0)

##        print(timeW)
        if t == 0: continue
        if t == T: continue


        # if u is destin we want Z on left, if origin we want Z on right
        m.addConstr(grb.quicksum(Cost[u][v]*x[u][v][tau]
                                 for u in V
                                 for v in V
                                 for tau in range(t+1))+
                    grb.quicksum(W[tau]
                                 for tau in range(t))
                    + Zd[t] + W[t]
                    ==
                    grb.quicksum(timeW[v]*x[u][v][t]
                                 for u in V
                                 for v in V)
                    + Zo[t])
        m.addConstr(Zd[t] <=
                    grb.quicksum((WinSize[v])*x[u][v][t]
                                 for u in V
                                 for v in VD.union({DriverDestin})))
        m.addConstr(Zo[t] <=
                    grb.quicksum((WinSize[v])*x[u][v][t]
                                 for u in V
                                 for v in VO.union({DriverOrigin})))

        m.addConstr(grb.quicksum(Cost[u][v]*x[u][v][tau]
                                 for u in V
                                 for v in V
                                 for tau in range(t+1))+
                    grb.quicksum(W[tau]
                                 for tau in range(t))
                    + Ye[t] + W[t] + Ys[t]
                    ==
                    grb.quicksum(timeP[v]*x[u][v][t]
                                 for u in V
                                 for v in V)
                    + Yl[t])

        m.addConstr(Ys[t] <=
                    grb.quicksum((WinSize[v])*x[u][v][t]
                                 for u in V
                                 for v in VD.union({DriverDestin})))

        # Add Lower Bound
        if LB != None:
            m.addConstr(grb.quicksum(Cost[u][v]*x[u][v][t]
                                for u in V
                                for v in V
                                for t in TIMERANGE)
                   + LAMBDA*grb.quicksum(Ye[t] + Yl[t]
                                  for t in TIMERANGE)
                        >= LB)


        if UB != None:
            m.addConstr(grb.quicksum(Cost[u][v]*x[u][v][t]
                                for u in V
                                for v in V
                                for t in TIMERANGE)
                   + LAMBDA*grb.quicksum(Ye[t] + Yl[t]
                                  for t in TIMERANGE)
                        <= UB)
##    print(WinSize)
    def NonWindow(j):
       for t in TimeRange(reqs[j].et, end=False):
           yield t


######### SET UNNECESSARY VARIABLES TO 0 #######
    for v in range(N):
        j = v%R
        
        for t in TIMERANGE:
            m.addConstr(x[SINK][v][t] == 0)
            m.addConstr(x[v][SOURCE][t] == 0)

        for t in range(T):
            m.addConstr(x[v][SINK][t] == 0)

        for t in range(1,T+1):
            m.addConstr(x[SOURCE][v][t] == 0)
            if v != DriverOrigin:
                m.addConstr(x[SOURCE][v][0] == 0)
                

            
    for t in TIMERANGE:
        m.addConstr(x[SOURCE][SINK][t] == 0)
    for u in V:
        m.addConstr(x[u][SINK][TIME]==0)
        for v in V:
            m.addConstr(x[u][v][TIME] == 0)

    def reqTime(u,t):
        if u < R:
            return reqs[u].et
            
        elif u < 2*R:
            return t
        else: return t

    m.setObjective(grb.quicksum(Cost[u][v]*x[u][v][t]
                                for u in V
                                for v in V
                                for t in TIMERANGE)
                   + LAMBDA*grb.quicksum(Ye[t] + Yl[t]
                                  for t in TIMERANGE),
                   grb.GRB.MINIMIZE)
    m.setParam('OutputFlag', MUTE) #Mute gurobi log

##    print("DONE SETTING UP")
    
    m.optimize()

    
##    if m.status == 3: return False,None,m,x,W,Zd,Zo,Ye,Yl,Cost
    if m.status == 3: return False, None, None

    if MUTE != 0:
        print('Cost:\t',sum(Cost[u][v]*x[u][v][t].x for u in V for v in V for t in TIMERANGE))
        print('yCost:\t', sum(LAMBDA*(Ye[t].x+Yl[t].x) for t in TIMERANGE))




    
##    for i in range(len(x)):
##        for j in range(len(x[i])):
##            COUNT = 0
##            for k in x[i][j].keys():
##                if x[i][j][k].x > 0:
##                    if i == j:
##                        COUNT += 1
##                    else: print('x[%d][%d][%d] = %f' %(i,j,k,x[i][j][k].x))
##            if COUNT > 0: print(COUNT)
##


    route = {}
    for i in range(len(x)):
        for j in range(len(x[i])):
            for t in x[i][j]:
                if x[i][j][t].x > 0:
                    route[t] = i
##    print(route)

    retRt = []
    for i in range(d.lt):
        if i in route:
            if len(retRt) == 0:
                retRt.append(route[i])
            elif retRt[-1]!=route[i]:
                retRt.append(route[i])

    if MUTE != 0: print(retRt)
##    print([retLoc(i) for i in retRt])
    
    retRt = [retInd(i) for i in retRt]

    
    if MUTE != 0: print(retRt)



    if MUTE != 0:
        for t in range(len(Ye)):
            if Ye[t].x >0:
                print('ye', t, Ye[t].x)
            if Yl[t].x >0:
                print('yl', t, Yl[t].x)
        for t in range(len(W)):
            if W[t].x >0:
                print(t, W[t].x)
    if retRt[0] == 'SOURCE':
        retRt = retRt[1:]


    
##    return True,m.ObjVal,m,x,W,Zd,Zo, Ye, Yl, Cost,timeW
    return True,m.ObjVal, retRt



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

    
    
    ##DRIVERS 5
    drivers.append(Driver( (2, 3) , (3, 29) , 0 , 58 , 4 ))
    drivers.append(Driver( (3, 17) , (19, 8) , 0 , 58 , 4 ))
    drivers.append(Driver( (19, 10) , (6, 23) , 0 , 58 , 4 ))
    drivers.append(Driver( (4, 6) , (12, 15) , 0 , 58 , 4 ))
    drivers.append(Driver( (6, 2) , (8, 16) , 0 , 58 , 4 ))
    ##REQUESTS 20
    reqs.append(Passenger( (9, 13) , (28, 11) , 3 , 42 , 12 ))
    reqs.append(Passenger( (12, 16) , (20, 12) , 12 , 35 , 19 ))
    reqs.append(Passenger( (24, 7) , (3, 0) , 2 , 26 , 2 ))
    reqs.append(Passenger( (25, 20) , (15, 6) , 7 , 45 , 17 ))
    reqs.append(Passenger( (8, 28) , (20, 25) , 0 , 21 , 4 ))
    reqs.append(Passenger( (13, 18) , (2, 17) , 13 , 51 , 26 ))
    reqs.append(Passenger( (2, 13) , (7, 23) , 1 , 33 , 11 ))
    reqs.append(Passenger( (12, 16) , (17, 3) , 0 , 28 , 7 ))
    reqs.append(Passenger( (19, 28) , (25, 1) , 0 , 46 , 9 ))
    reqs.append(Passenger( (14, 8) , (25, 25) , 1 , 22 , 1 ))
    reqs.append(Passenger( (16, 20) , (29, 19) , 7 , 46 , 19 ))
    reqs.append(Passenger( (29, 29) , (1, 2) , 0 , 57 , 13 ))
    reqs.append(Passenger( (13, 24) , (12, 17) , 12 , 31 , 17 ))
    reqs.append(Passenger( (12, 14) , (25, 10) , 7 , 35 , 14 ))
    reqs.append(Passenger( (19, 23) , (24, 16) , 14 , 30 , 17 ))
    reqs.append(Passenger( (6, 5) , (12, 6) , 9 , 28 , 15 ))
    reqs.append(Passenger( (16, 9) , (9, 2) , 15 , 46 , 25 ))
    reqs.append(Passenger( (10, 2) , (18, 6) , 7 , 26 , 12 ))
    reqs.append(Passenger( (21, 26) , (29, 16) , 9 , 22 , 9 ))
    reqs.append(Passenger( (7, 14) , (23, 29) , 0 , 39 , 8 ))


    d = drivers[4]
    indd = [15,16,17]



    
    reqss = [reqs[i] for i in indd]
    reqs = reqss
    
    b,v,Rt = travel(d,reqss)
    print(Rt)
##    b,v,m,x,W,Zd,Zo, Ye, Yl, Cost,timeW = travel(d,reqs)
    print(b,v)
##    print(x)
##    if x != None:

##    for i in range(len(W)):
##        print(W[i].x)
##        print(Ye[i].x)
##        print(Yl[i].x)
##        print("\n")

##    print(Cost)
##        print(b)
##        print(x)
##    for i in range(len(x)):
##        for j in range(len(x[i])):
##            for t in x[i][j]:
##                if x[i][j][t].x >0:
##                    print(i,j,t,x[i][j][t].x,Cost[i][j],timeW[j])

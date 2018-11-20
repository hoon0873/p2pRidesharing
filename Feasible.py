import numpy as np
import networkx as nx
import pylab
import time
import random as rd
import gurobipy as grb
from Classes import Driver
from Classes import Passenger
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


def travel(d, reqs, LB = None, LABMDA = 0.2):

    T = d.lt

    R = len(reqs)
    V = set(range(2*len(reqs)+2))


    DriverOrigin = len(V)-2
    DriverDestin = len(V)-1
    SOURCE = len(V)
    SINK = len(V)+1
    N = SINK+1

    VO = set(range(0,R))
    VD = set(range(R,2*R))



    EPSILON = 1
    TIMERANGE = set()
    InstTime = set()
    TIMERANGE.add(d.et-1)
    TIMERANGE.add(T+1)
    ADDT = d.et
    while ADDT < T+1:
        TIMERANGE.add(ADDT)
        InstTime.add(ADDT)
        ADDT += EPSILON
        ADDT = int(ADDT)
##        print(ADDT)
##        time.sleep(0.05)
    
##    TIMERANGE = range(d.et-1,T+1)
    TIME = d.et


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
                Cost[i][j] = d.cdet*Distance(u,v)
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
    y = []
    for i in range(N):
        y.append({})
        for t in TIMERANGE:
            y[i][t] = m.addVars(TIMERANGE, vtype=grb.GRB.BINARY)

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

    def retInd(i):
        if   i <   R: u = i
        elif i < 2*R: u = i-R
        elif i <=2*R+1: u = 'd'
        elif i ==2*R+2: u='SOURCE'
        else: u = "SINK"

        return u

    def retPrefTim(i):
        if   i <   R: u = reqs[i].pt
        elif i < 2*R: u = reqs[i-R].pt+Distance(reqs[i-R].ori,reqs[i-R].des)
        elif i ==2*R: u = d.et
        elif i ==2*R+1: u = d.lt
        elif i ==2*R+2: u = 0
        else: u = T

        return u    

    timeP = np.zeros(N)
    for i in range(N):
        timeP[i] = retPrefTim(i)        


##    print('TIME TESTING', Time[5][4])
    
    for v in range(N):
        for t in InstTime:
            if t>=T: continue
                
            m.addConstr(grb.quicksum(x[u][v][int(t-Time[u][v])]
                                     for u in posT(v,t))
                        == grb.quicksum(x[v][w][t]
                                       for w in V))
        
        m.addConstr(grb.quicksum(x[u][v][int(T-Time[u][v])]
                                 for u in posT(v,T))
                    == grb.quicksum(x[v][w][T]
                                    for w in range(N)))

    m.addConstr(x[SOURCE][DriverOrigin][TIME-1] == 1)
    m.addConstr(x[DriverDestin][SINK][T] == 1)



    for j in range(R):
        for t in TimeRange(t=reqs[j].lt, lb=max(reqs[j].et,TIME)):
            
##            print(j, tau)
##                print('R x[%d][%d][%d]' %(traR+j, u,tau))
##            time.sleep(1)
##                print(TIME)
##                print(traT)
##                print(j,tau)
            m.addConstr(grb.quicksum(x[j][v][tau]
                                     for v in V
                                     for tau in TimeRange(t, end=False))
                        >= grb.quicksum(x[R+j][u][t]
                                        for u in V))
##            for v in range(traV):
##                for t in range(TIME, tau):
##                    print('L x[%d][%d][%d]' %(j,v,t))
##            print('\n')
##            for u in range(traV):
##                print('R x[%d][%d][%d]' %(traR+j, u,tau))
##            time.sleep(1)
##                print(TIME)
##                print(traT)
##                print(j,tau)
            m.addConstr(grb.quicksum(x[j][v][tau]
                                     for v in V
                                     for tau in TimeRange(t, end=False))
                        >= grb.quicksum(x[R+j][u][t]
                                        for u in V))

            
    for t in InstTime:
##        print('\n')
##        print(t)
##        for u in traOrigin:
##            for v in range(traV):
##                for tau in range(TIME, t+1):
##                    print('+ x[%d][%d][%d]' %(u,v,tau))
##        print('\n')
##        for u in traDestin:
##            for v in range(traV):
##                for tau in range(TIME, t+1):
##                    print('+ x[%d][%d][%d]' %(u,v,tau))
##        time.sleep(1)

        
        m.addConstr(grb.quicksum(x[u][v][tau]
                                 for u in VO
                                 for v in vNot(u)
                                 for tau in TimeRange(t))
                    -
                    grb.quicksum(x[u][v][tau]
                                 for u in VD
                                 for v in vNot(u)
                                 for tau in TimeRange(t))
                    <= d.cap)
        
##        m.addConstr(grb.quicksum(y[u][tau]
##                                 for u in VO
##                                 for tau in TimeRange(t))
##                    -
##                    grb.quicksum(y[u][tau]
##                                 for u in VD
##                                 for tau in TimeRange(t))
##                    <= d.cap)        

    
    for s in VO:
        m.addConstr(grb.quicksum(x[s][v][t]
                                 for v in vNot(s)
                                 for t in TimeRange(lb= max(reqs[s].et-1,TIME), t=reqs[s].lt))
                    >= 1)
    for s in VD:
        m.addConstr(grb.quicksum(x[v][s][t]
                                 for v in vNot(s)
                                 for t in TimeRange(lb= max(reqs[s-R].et-1,TIME), t = reqs[s-R].lt))
                    >= 1)


    for j in range(R):
        for t in TimeRange(t=reqs[j].lt, lb=max(reqs[j].et,TIME)):
            for tp in TimeRange(t=reqs[j].lt,lb=max(reqs[j].et,TIME)):
                m.addConstr(y[j][t][tp] >=
                            grb.quicksum(x[j][v][t] for v in range(N))
                            +grb.quicksum(x[u][j+R][tp] for u in range(N))
                            -1
                            )
    m.addConstr(grb.quicksum(y[j][t][tp] for j in range(R) for t in TimeRange(t=reqs[j].lt, lb=max(reqs[j].et,TIME))
                             for tp in TimeRange(t=reqs[j].lt, lb=max(reqs[j].et,TIME)))==1)
##    for u in V:
##        for t in TIMERANGE:
##            m.addConstr(y[u][t] <=
##                        grb.quicksum(x[u][v][t]
##                                     for v in range(N)))
######    for v in VD:
######        for t in TIMERANGE:
######            m.addConstr(y[v][t] <=
######                        grb.quicksum(x[u][v][t]
######                                     for u in range(N)))
####            
##    for v in VO:
##        m.addConstr(grb.quicksum(y[v][t] for t in TimeRange(lb= max(reqs[v].et-1,TIME), t = reqs[v].lt))
##                     == 1)
##    for v in VD:
##        m.addConstr(grb.quicksum(y[v][t] for t in TimeRange(lb= max(reqs[v-R].et-1,TIME), t = reqs[v-R].lt))
##                     == 1)
                        
######        print(s)
######        for t in TimeRange(lb= max(reqs[s-R].et,TIME), t = reqs[s-R].lt):
######            print(t)

####### constraint on no cycle if there's flow out u at time t, there's no flow in u.
####    for u in range(V):
####        for t in traTIMERANGE:
####            m.addConstr(grb.quicksum(x[u][v][t]
####                                     for v in [x for x in range(N) if x!=u])
####                        +
####                        grb.quicksum(x[v][u][t]
####                                     for v in [x for x in range(N) if x!=u])
####                        <= 2)
        
    def NonWindow(j):
       for t in TimeRange(reqs[j].et, end=False):
           yield t


####### SET UNNECESSARY VARIABLES TO 0 #######
    for v in range(N):
        j = v%R
        
        for t in TIMERANGE:
            m.addConstr(x[SINK][v][t] == 0)
            m.addConstr(x[v][SOURCE][t] == 0)

        for t in range(TIME-1,T):
            m.addConstr(x[v][SINK][t] == 0)

        for t in TimeRange(T):
            m.addConstr(x[SOURCE][v][t] == 0)
            if v != DriverOrigin:
                m.addConstr(x[SOURCE][v][TIME-1] == 0)
                
        if v>=2*R: continue
        
        for t in NonWindow(j):
            for u in range(N):
                m.addConstr(x[v][u][t] == 0)

        for t in TimeRange(lb=reqs[j].lt):
            for u in vNot(v):
                m.addConstr(x[v][u][t] == 0)
        m.addConstr(x[v][v][T+1] == 0)


        m.addConstr(grb.quicksum(x[v][u][t] for u in vNot(v) for t in TIMERANGE) == 1)



        


            
    for t in TIMERANGE:
        m.addConstr(x[SOURCE][SINK][t] == 0)
    for u in V:
        m.addConstr(x[u][SINK][TIME-1]==0)
        for v in V:
            m.addConstr(x[u][v][TIME-1] == 0)

    def reqTim(u,t):
        if u < R:
            return reqs[u].et
            
        elif u < 2*R:
            return t
        else: return t

    m.setObjective(grb.quicksum(Cost[u][v]*x[u][v][t]
                                for u in V
                                for v in V
                                for t in TIMERANGE)
                   +
                   grb.quicksum(d.cdet*(abs(timeP[0]-t)*x[0][u][t])
                                for u in vNot(v)
                                for t in TIMERANGE)
                   +
                   grb.quicksum((reqs[j].cdet*(tp-t)+reqs[j].cdev*(abs(reqs[j].pt-t)))*y[j][t][tp]
                                for j in range(R)
                                for t in TimeRange(t=reqs[j].lt, lb=max(reqs[j].et,TIME))
                                for tp in TimeRange(t=reqs[j].lt, lb=max(reqs[j].et,TIME)))
                   ,grb.GRB.MINIMIZE)


    m.setParam('OutputFlag', MUTE) #Mute gurobi log

##    print("DONE SETTING UP")
    
    m.optimize()

    if m.status == 3: return False,None, None

    if MUTE != 0:
        print('COST:\t'+str(sum(Cost[u][v]*x[u][v][t].x for u in V for v in V for t in TIMERANGE)))
        print('YCOST:\t'+str(sum(LAMBDA*(abs(timeP[v]-t)*x[v][u][t].x)
                                    for v in VO.union(VD)
                                    for u in vNot(v)
                                    for t in TIMERANGE)))

    

##    for i in range(len(x)):
##        for j in range(len(x[i])):
##            COUNT = 0
##            for k in x[i][j].keys():
##                if x[i][j][k].x > 0:
##                    if i == j:
##                        COUNT += 1
##                    else: print('x[%d][%d][%d] = %f' %(i,j,k,x[i][j][k].x))
##            if COUNT > 0: print(COUNT)



    route = {}
    for i in range(len(x)):
        for j in range(len(x[i])):
            for t in x[i][j]:
                if x[i][j][t].x > 0:
                    route[t] = i
                    if i != j and MUTE!= 0: print(i,j,t,x[i][j][t].x, Cost[i][j])
    if MUTE != 0: print('\n\n')
    if MUTE != 0:
        for v in V:
            for u in vNot(v):
                for t in TIMERANGE:
                    if x[v][u][t].x >0:
                        print(v,u,t,x[v][u][t].x,abs(timeP[v]-t),timeP[v])

##    for i in V:
##        print(i)
####    if MUTE != 0:
####        for i in range(len(y)):
####            for j in y[i]:
####                if y[i][j].x >0:
####                    print(i,j,y[i][j].x)
####                    print('\t', timeP[i], abs(timeP[i]-j), LAMBDA*abs(timeP[i]-j)*y[i][j].x)
    

##    print(route)

    retRt = []
    for i in range(d.lt):
        if i in route:
            if len(retRt) == 0:
                retRt.append(route[i])
            elif retRt[-1]!=route[i]:
                retRt.append(route[i])

    

    
    retRt = [retInd(i) for i in retRt]
    if MUTE != 0:   print(retRt)
    
##    return m,m.ObjVal,x,y,Cost
    return True, m.ObjVal,retRt
















if __name__ == "__main__":

##    testR = 2
##    oriD = (0,0)
##    desD = (0,1*PRECISION)
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
##        print(ori,des,tim,etim)



##    ori = (round(rd.uniform(0,1),PRECISION),round(rd.uniform(0,1),PRECISION))
##    des = (round(rd.uniform(0,1),PRECISION),round(rd.uniform(0,1),PRECISION))
##    tim = round(rd.uniform(0,max(testT-2-Distance(ori,des),1)),PRECISION)
##    delta = round(rd.uniform(0,1),PRECISION)
##    etim = max(tim+delta,testT-1)
##    reqs.append(Passenger(ori,des,tim,etim))
##    for i in range(testR):
##        ori = (rd.randint(0,10),rd.randint(0,10))
##        des = (rd.randint(0,10),rd.randint(0,10))
##    ##    tim = rd.randint(0,max(testT-2-Distance(ori,des),1))
##        tim = 0
##        delta = rd.randint(0,10)
##        etim = max(tim+Distance(ori,des)+delta,testT-1)
##        reqs.append(Passenger(ori,des,tim,etim))
##
##
##
##    for i in range(testR):
##        ori = (i,0)
##        des = (10,10-i)
##        tim = 0
##        etim = 20
##        reqs.append(Passenger(ori,des,tim,etim))
##
##    rr = Passenger((0,0),(0,4), 1, 67)
##    reqs = [rr]
##        
##    print("DONE WITH INPUT")
##    r1 = Passenger(1,4,1,10)
##    r2 = Passenger(2,3,1,10)
##    r3 = Passenger(3,6,6,14)
##    r4 = Passenger(5,9,10,19)
##    reqs = [r1,r2,r3,r4]


##    testR = 2
##    oriD = (0,0)
##    desD = (0,1*PRECISION)
##    testT = 20
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

##    d = Driver((0, 0) ,(18, 0) ,0, 59, 4)
##    reqs = []
##    reqs.append(Passenger((1, 0), (19, 0) ,0, 41, 9))
##    reqs.append(Passenger((2, 0), (15, 0), 0, 36, 6))
##    reqs.append(Passenger((3, 0), (17, 0) ,4, 29, 5))
##    reqs.append(Passenger((7, 0), (20, 0) ,2 ,50, 14))


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

    d = Driver((26, 29) ,(20, 15) ,0 ,59 ,4)

    reqs = []
    reqs.append(Passenger((25, 24), (15, 19) ,10 ,38 ,18))
    reqs.append(Passenger((25, 27), (15, 19) ,3 ,29 ,10))
    reqs.append(Passenger((25, 27) ,(16, 14) ,4 ,42 ,15))
    reqs.append(Passenger((24, 27) ,(15, 17) ,9 ,31 ,13))
    
    
##    b,obj,x,y,Cost = travel(d,reqs)
    b,obj,Rt = travel(d,reqs)
    if obj != None:

##    print(Cost)
        print(b)
        print(obj)


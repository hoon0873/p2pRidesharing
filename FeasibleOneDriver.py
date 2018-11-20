import numpy as np
import networkx as nx
import pylab
import time
import random as rd
import gurobipy as grb
from Classes import Driver
from Classes import Passenger

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
    return int(np.linalg.norm(i-j))


def travel(d, reqs, LB = None, LABMDA = 0.2, RHO = 1):

    T = d.lt



    R = len(reqs)
    V = set(range(2*len(reqs)+2))
    
    c = np.ones(R)*100 # Penalty of not satisfying request
    
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
    y = []
    for i in range(N):
        y.append([])
        y[i] = m.addVars(TIMERANGE, vtype=grb.GRB.BINARY)

    z = m.addVars(R, vtype=grb.GRB.BINARY)

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

    
    for v in range(N):
        for t in InstTime:
            if t>=T: continue
##            print('\n')
##            print(v,t)
##            for u in posT(v,t):
####                print(Time[u][v])
####                print(t-Time[u][v],PRECISION)
####                etp = round(t-Time[u][v],PRECISION)
##                print('x[%d][%d][%f]' %(u,v,t-Time[u][v]))
####                print(etp)
####                print(x[u][v][round(etp,2)])
##            print('\n')
##            for w in V:
##                print('x[%d][%d][%f]' %(v,w,t))
##            time.sleep(0.3)

##            print(v,t)
##            for u in posT(v,t):
##                print('\n')
##                print(u)
##                print(t-Time[u][v])
##                print(x[u][v])
                
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
                    grb.quicksum(x[v][u][tau]
                                 for u in VD
                                 for v in vNot(u)
                                 for tau in TimeRange(t))
                    <= d.cap)

    
##    for s in VO:
##        m.addConstr(grb.quicksum(x[s][v][t]
##                                 for v in vNot(s)
##                                 for t in TimeRange(lb= max(reqs[s].et,TIME), t=reqs[s].lt))
##                    == 1)
##    for s in VD:
##        m.addConstr(grb.quicksum(x[v][s][t]
##                                 for v in vNot(s)
##                                 for t in TimeRange(lb= max(reqs[s-R].et,TIME), t = reqs[s-R].lt))
##                    == 1)


    for v in V:
        for t in TIMERANGE:
            m.addConstr(y[v][t] <=
                        grb.quicksum(x[u][v][t]
                                     for u in range(N)))
    for v in VO:
        m.addConstr(z[v] + grb.quicksum(y[v][t] for t in TimeRange(lb= max(reqs[v].et-1,TIME), t = reqs[v].lt))
                     == 1)
    for v in VD:
        m.addConstr(z[v-R] + grb.quicksum(y[v][t] for t in TimeRange(lb= max(reqs[v-R].et-1,TIME), t = reqs[v-R].lt))
                     == 1)
                        
##        print(s)
##        for t in TimeRange(lb= max(reqs[s-R].et,TIME), t = reqs[s-R].lt):
##            print(t)

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

    m.setObjective(RHO*(grb.quicksum(Cost[u][v]*x[u][v][t]
                                for u in V
                                for v in V
                                for t in TIMERANGE)
                   +
                   grb.quicksum(LAMBDA*(abs(timeP[v]-t)*y[v][t])
                                for v in V
                                for t in TIMERANGE))
                   + grb.quicksum(c[r]*z[r] for r in range(R))
                   ,grb.GRB.MINIMIZE)
    m.setParam('OutputFlag', MUTE) #Mute gurobi log

##    print("DONE SETTING UP")
    
    m.optimize()



    
    if m.status == 3:
        print("HERE")
        return False,None
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
##    print(route)

    retRt = []
    for i in range(d.lt):
        if i in route:
            if len(retRt) == 0:
                retRt.append(route[i])
            elif retRt[-1]!=route[i]:
                retRt.append(route[i])

##    print(retRt)

    
    retRt = [retInd(i) for i in retRt]
##    print(retRt)
    
##    return m,m.ObjVal,x,y,Cost
    return True, m.ObjVal,x,y,z,m

if __name__ == "__main__":

    testR = 10
    oriD = (0,0)
    desD = (1*PRECISION,1*PRECISION)
    testT = 10*Distance(oriD,desD)
    capD = 4
    tD = 0
    d = Driver(oriD,desD,tD,testT,capD)
    reqs = []
    for i in range(testR):
        ori = (int(rd.uniform(0,1)*PRECISION),int(rd.uniform(0,1)*PRECISION))
        des = (int(rd.uniform(0,1)*PRECISION),int(rd.uniform(0,1)*PRECISION))
        tim = int(rd.uniform(0,max(testT-2-Distance(ori,des),1)))
        delta = int(rd.uniform(0,1)*PRECISION)
        etim = min(tim+delta+Distance(ori,des),testT-1)
        ptim = tim+delta//2
        reqs.append(Passenger(ori,des,tim,etim,ptim))
        print(ori,des,tim,etim,ptim)


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




    b,c,x,y,z,m = travel(d,reqs)
    if c != None:

##    print(Cost)
        print(b)
        print(c)

    setReq = set()
    for i in range(len(x)):
        for j in range(len(x[i])):
            for t in x[i][j]:
                if x[i][j][t].x > 0:
                    print(i,j,t)
                    setReq.add(i)

    print(setReq)
    print([y[i].x for i in y])




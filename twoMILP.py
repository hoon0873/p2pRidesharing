import numpy as np
import networkx as nx
import pylab
import time
import random as rd
import gurobipy as grb
from Classes import Driver
from Classes import Passenger
import math

MUTE = 0 #set to 0 to MUTE
PRUN1 = 0 #set to 1 to do PRUNING


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

def MILP(drivers, reqs):
    R = len(reqs)
    D = len(drivers)
##    T = min(drivers[i].et for i in range(D))
    T = 0
    T = max(drivers[i].lt for i in range(D)) - T
    BEGINTIME = time.clock()


    inpCur = []
    P = []
    BDO = [set() for i in range(D)]     #set of edges starting at driver's origin
    BRO = [set() for i in range(R)]     #set of edges starting at rider's origin
    BRD = [set() for i in range(R)]     #set of edges starting at rider's destination
    EDD = [set() for i in range(D)]     #set of edges ending at driver's destination
    ERD = [set() for i in range(R)]     #set of edges ending at rider's destination
    VDO = set()                         #set of vertices of driver's origin
    VDD = set()                         #set of vertices of driver's destination
     
    G = nx.DiGraph() 
    # NODE = driver/req index ; origin/des ; time

    # Take index and od indicator and output its location
    def retLocation(index, od):
        if type(index) == type('d'):
            ind = int(index[1:])
            if od == 0:
                return drivers[ind].ori
            else:
                return drivers[ind].des
        else:
            if od == 0:
                return reqs[index].ori
            else:
                return reqs[index].des
    def retPt(index,od):
        if type(index) == type('d'):
            ind = int(index[1:])
            if od == 0:
                return drivers[ind].et
            else:
                return drivers[ind].lt
        else:
            if od == 0:
                return reqs[index].pt
            else:
                return reqs[index].pt+DIST(reqs[index])

    def DistOfEdge(e):
        i1, od1, t1 = e[0]
        i2, od2, t2 = e[1]

        return Distance(retLocation(i1,od1),retLocation(i2,od2))



    # ADDING EDGES
    for i in range(D):
        dIndex = 'd'+str(i)
        for t in range(drivers[i].et,drivers[i].lt-DIST(drivers[i])+1):
            G.add_node((dIndex, 0,t))                   # add origin
            G.add_node((dIndex, 1,t+DIST(drivers[i])))  # add destin
            VDO.add((dIndex, 0,t))
            VDD.add((dIndex, 1,t+DIST(drivers[i])))


            # we only need edge from driver's origin to req's origin
            for j in range(R):
                if t+Distance(retLocation(dIndex,0),reqs[j].ori) < reqs[j].et:
                    G.add_edge((dIndex,0,t),(j,0,t+Distance(retLocation(dIndex,0),reqs[j].ori)))
                    BDO[i].add(((dIndex,0,t),(j,0,t+Distance(retLocation(dIndex,0),reqs[j].ori))))
                    G.add_edge((j,0,t+Distance(retLocation(dIndex,0),reqs[j].ori)),(j,0,reqs[j].et))
                elif t+Distance(retLocation(dIndex,0),reqs[j].ori) <= reqs[j].lt-DIST(reqs[j]):
                    G.add_edge((dIndex,0,t),(j,0,math.ceil(t+Distance(retLocation(dIndex,0),reqs[j].ori))))
                    BDO[i].add(((dIndex,0,t),(j,0,math.ceil(t+Distance(retLocation(dIndex,0),reqs[j].ori)))))

            # and driver's origin to driver's destin
            G.add_edge((dIndex,0,t),(dIndex,1,t+DIST(drivers[i])))
            BDO[i].add(((dIndex,0,t),(dIndex,1,t+DIST(drivers[i]))))
            EDD[i].add(((dIndex,0,t),(dIndex,1,t+DIST(drivers[i]))))
                

    for j in range(R):
        for t in range(reqs[j].et,reqs[j].lt-DIST(reqs[j])+1):
            G.add_node((j,0,t))
            G.add_node((j,1,t+DIST(reqs[j])))
            if t != reqs[j].lt-DIST(reqs[j]):
                G.add_edge((j,0,t),(j,0,t+1))
                G.add_edge((j,1,t+DIST(reqs[j])),(j,1,t+DIST(reqs[j])+1))
                
            # add req's origin to req's destin
            G.add_edge((j,0,t),(j,1,t+DIST(reqs[j])))
            BRO[j].add(((j,0,t),(j,1,t+DIST(reqs[j]))))
            ERD[j].add(((j,0,t),(j,1,t+DIST(reqs[j]))))
            for k in range(R):
                if j == k: continue
                # add j's origin to k's origin
                if (retLocation(j,0) == retLocation(k,0)).all() and j>k:
                    True
                elif  t+Distance(retLocation(j,0),reqs[k].ori) < reqs[k].et:
                    G.add_edge((j,0,t),(k,0,t+Distance(retLocation(j,0),reqs[k].ori)))
                    BRO[j].add(((j,0,t),(k,0,t+Distance(retLocation(j,0),reqs[k].ori))))
                    G.add_edge((k,0,t+Distance(retLocation(j,0),reqs[k].ori)),(k,0,reqs[k].et))
##                    print('1: ',(k,0,t+Distance(retLocation(j,0),reqs[k].ori)),(k,0,reqs[k].et))
                elif t+Distance(retLocation(j,0),reqs[k].ori) <= reqs[k].lt-DIST(reqs[k]):
                    G.add_edge((j,0,t),(k,0,math.ceil(t+Distance(retLocation(j,0),reqs[k].ori))))
                    BRO[j].add(((j,0,t),(k,0,math.ceil(t+Distance(retLocation(j,0),reqs[k].ori)))))

                # add j's origin to k's destin
                if (retLocation(j,0) == retLocation(k,1)).all() and j>k:
                    True
                elif t+Distance(retLocation(j,0),reqs[k].des) < reqs[k].et+DIST(reqs[k]):
                    True
##                    G.add_edge((j,0,t),(k,1,t+Distance(retLocation(j,0),reqs[k].des)))
##                    BRO[j].add(((j,0,t),(k,1,t+Distance(retLocation(j,0),reqs[k].des))))
##                    ERD[k].add(((j,0,t),(k,1,t+Distance(retLocation(j,0),reqs[k].des))))
##                    G.add_edge((k,1,t+Distance(retLocation(j,0),reqs[k].des)),(k,1,reqs[k].et+DIST(reqs[k])))

                elif t+Distance(retLocation(j,0),reqs[k].des) <= reqs[k].lt:
                    G.add_edge((j,0,t),(k,1,math.ceil(t+Distance(retLocation(j,0),reqs[k].des))))
                    BRO[j].add(((j,0,t),(k,1,math.ceil(t+Distance(retLocation(j,0),reqs[k].des)))))
                    ERD[k].add(((j,0,t),(k,1,math.ceil(t+Distance(retLocation(j,0),reqs[k].des)))))


                tt = t+DIST(reqs[j])
                # add j's destin to k's origin
                if (retLocation(j,1) == retLocation(k,0)).all() and j>k:
                    True
                elif  tt+Distance(retLocation(j,1),reqs[k].ori) < reqs[k].et:
                    G.add_edge((j,1,tt),(k,0,tt+Distance(retLocation(j,1),reqs[k].ori)))
                    BRD[j].add(((j,1,tt),(k,0,tt+Distance(retLocation(j,1),reqs[k].ori))))
                    G.add_edge((k,0,tt+Distance(retLocation(j,1),reqs[k].ori)),(k,0,reqs[k].et))

                elif tt+Distance(retLocation(j,1),reqs[k].ori) <= reqs[k].lt-DIST(reqs[k]):
                    G.add_edge((j,1,tt),(k,0,math.ceil(tt+Distance(retLocation(j,1),reqs[k].ori))))
                    BRD[j].add(((j,1,tt),(k,0,math.ceil(tt+Distance(retLocation(j,1),reqs[k].ori)))))
                    
                # add j's destin to k's destin  
                if (retLocation(j,1) == retLocation(k,1)).all() and j>k:
                    True         
                elif tt+Distance(retLocation(j,1),reqs[k].des) < reqs[k].et+DIST(reqs[k]):
                    True
##                    G.add_edge((j,1,t),(k,1,tt+Distance(retLocation(j,0),reqs[k].des)))
##                    BRD[j].add(((j,1,t),(k,1,tt+Distance(retLocation(j,0),reqs[k].des))))
##                    ERD[k].add(((j,1,t),(k,1,tt+Distance(retLocation(j,0),reqs[k].des))))
##                    G.add_edge((k,1,t+Distance(retLocation(j,1),reqs[k].des)),(k,1,reqs[k].et+DIST(reqs[k])))

                elif tt+Distance(retLocation(j,1),reqs[k].des) <= reqs[k].lt:
                    G.add_edge((j,1,tt),(k,1,math.ceil(tt+Distance(retLocation(j,1),reqs[k].des))))
                    BRD[j].add(((j,1,tt),(k,1,math.ceil(tt+Distance(retLocation(j,1),reqs[k].des)))))
                    ERD[k].add(((j,1,tt),(k,1,math.ceil(tt+Distance(retLocation(j,1),reqs[k].des)))))

#### Also, it means we can reduce some edges.
##  We don’t need edges that goes to a destination but arrives before  (
##  t_earliest time at origin of that rider + min dist from origin to destination)

            # add edge j's destin to driver's destin
            for i in range(D):
                tt = t+DIST(reqs[j])
                dIndex = 'd'+str(i)
                if tt+Distance(retLocation(j,1),drivers[i].des) < drivers[i].et+DIST(drivers[i]):
                    G.add_edge((j,1,tt),(dIndex,1,tt+Distance(retLocation(j,1),drivers[i].des)))
                    BRD[j].add(((j,1,tt),(dIndex,1,tt+Distance(retLocation(j,1),drivers[i].des))))
                    EDD[i].add(((j,1,tt),(dIndex,1,tt+Distance(retLocation(j,1),drivers[i].des))))
                    VDD.add((dIndex,1,tt+Distance(retLocation(j,1),drivers[i].des)))
                    G.add_edge((dIndex,1,t+Distance(retLocation(j,1),drivers[i].des)),(dIndex,1,drivers[i].et+DIST(drivers[i])))
                elif tt+Distance(retLocation(j,1),drivers[i].des) <= drivers[i].lt:
                    G.add_edge((j,1,tt),(dIndex,1,math.ceil(tt+Distance(retLocation(j,1),drivers[i].des))))
                    BRD[j].add(((j,1,tt),(dIndex,1,math.ceil(tt+Distance(retLocation(j,1),drivers[i].des)))))
                    EDD[i].add(((j,1,tt),(dIndex,1,math.ceil(tt+Distance(retLocation(j,1),drivers[i].des)))))
                    VDD.add((dIndex,1,math.ceil(tt+Distance(retLocation(j,1),drivers[i].des))))


    BD = set()
    ED = set()
    for i in range(D):
        BD = BD.union(BDO[i])
        ED = ED.union(EDD[i])
        
    RO = set()
    RD = set()
    EERD = set()
    for j in range(R):
        RO = RO.union(BRO[j])
        RD = RD.union(BRD[j])
        EERD = EERD.union(ERD[j])

    
    if MUTE != 0: print("\nConstruction Time: %f\n\n" %(time.clock()-BEGINTIME))

    #DEFINING COST FUNCITION
    c = {}
    LAMBDA1 = 0.2
    for e in G.edges():
        if e in BD:
            c[e] = DistOfEdge(e)
        elif e in RD and e in EERD:
            c[e] = DistOfEdge(e) + LAMBDA1*max(0, e[1][2]-retPt(e[1][0],e[1][1]))
        elif e in RD:
            c[e] = DistOfEdge(e)
        elif e[0][0] == e[1][0] and e[0][1] == e[1][1]:
            c[e] = DistOfEdge(e)
        elif e in EERD: # origin to destin [RELU function]
            c[e] = DistOfEdge(e) + LAMBDA1*abs(retPt(e[0][0],e[0][1])-e[0][2]) + LAMBDA1*max(0, e[1][2]-retPt(e[1][0],e[1][1]))
        else:   #origin to origin
            c[e] = DistOfEdge(e) + LAMBDA1*abs(retPt(e[0][0],e[0][1])-e[0][2])

    m = grb.Model("COPYMILP")


    y = {}
    for e in G.edges():
        y[e] = m.addVars(D, vtype=grb.GRB.BINARY, lb=0, ub=1)

    z = m.addVars(R, vtype = grb.GRB.BINARY, lb=0, ub=1)


### ADD CONSTRAINTS ###########################
    
    def OrinTim(t,i):   # return set of request origin edge start less than time t
        for e in RO:
            if e[0][2] <= t: yield e
    def DesTim(t,i):    # return set of request destination edge less than time t
        for e in RD:
            if e[0][2] <= t: yield e
    def OriRTim(t,i):   # return set of request j's origin edge less than time t
        for e in BRO[i]:
            if e[0][2] <= t: yield e
    def DesRTim(t,i):   # return set of request j's destin edge less than time t
        for e in BRD[i]:
            if e[0][2] <= t: yield e
        


    for i in range(D):
        # FLOW CONSERVATION CONSTRAINT
        for v in G.nodes():
            if v in VDD: continue
            if v in VDO: continue
            m.addConstr(grb.quicksum(y[e][i] for e in G.in_edges(v))
                        ==
                        grb.quicksum(y[e][i] for e in G.out_edges(v))
                        )
        # Driver need to be satisfied
        m.addConstr(grb.quicksum(y[e][i] for e in BDO[i])
                    == 1)
        m.addConstr(grb.quicksum(y[e][i] for e in EDD[i])
                    == 1)

    for j in range(R):
        m.addConstr(z[j]+grb.quicksum(y[e][i] for e in BRO[j] for i in range(D)) == 1)
        for i in range(D):
            m.addConstr(grb.quicksum(y[e][i] for e in BRO[j]) == grb.quicksum(y[e][i] for e in BRD[j]))

    
        
    for t in range(T):
        # CAPACITY CONSTRAINT
        for i in range(D):
            m.addConstr(grb.quicksum(y[e][i] for e in OrinTim(t,i)) - grb.quicksum(y[e][i] for e in DesTim(t,i))
                        <= drivers[i].cap)
        # ORIGIN BEFORE DESTINATION
            for j in range(R):
                m.addConstr(grb.quicksum(y[e][i] for e in OriRTim(t,j)) - grb.quicksum(y[e][i] for e in DesRTim(t,j))
                            >= 0)

    for i in range(D):
        for j in range(D):
            if i == j: continue
            for e in BDO[j]:
                m.addConstr(y[e][i] == 0)
            for e in EDD[j]:
                m.addConstr(y[e][i] == 0)
            
    


    m.setObjective(grb.quicksum(c[e]*y[e][i] for e in G.edges() for i in range(D)) + LAMBDA*grb.quicksum(z[j] for j in range(R)),
                   grb.GRB.MINIMIZE)


    m.setParam('OutputFlag', MUTE)
    m.optimize()


    if m.status == 3:
        print("INPUT IS INFEASIBE")
##        retVal = 0
##        for i in range(D):
##            retVal += Distance()
        return m,[],0,0,None,None

    ENDTIME = time.clock()
    if MUTE != 0: print(ENDTIME - BEGINTIME)

##    for i in range(len(x)):
##        for j in range(len(x[i])):
##            if x[i][j].x > 0:
##                print(i,j,'\t',P[i],'\t',P[j])

    if MUTE != 0:
        for e in y:
            for j in range(len(y[e])):
                if y[e][j].x>0:
                    print(e,j,c[e])
##                    print(e,end=', ')

        for i in range(len(z)):
            if z[i].x>0:
                print(i)

##    
##    return m,x, m.ObjVal, ENDTIME-BEGINTIME 

    return m,y,m.ObjVal,ENDTIME-BEGINTIME,G,c


##    return   m,v,x,y,z, P,c 




    
    print(len(P))
##    print(BRO,BRD,BDO,BDD,RD,RO,EDD)

if __name__ == "__main__":
    R = 10
    D = 10
    reqs = []
    drivers = []
    PRECISION = 30
##    ##T = 150
    T = Distance((0,0), (0.7*PRECISION, 0.7*PRECISION))+PRECISION//2
##
##    
##    BEGINTIME = time.clock()
##    print("DRIVERS")
##    for i in range(D):
##    ##    ori = (int(rd.uniform(0,1)*PRECISION),int(rd.uniform(0,1)*PRECISION))
##    ##    des = (int(rd.uniform(0,1)*PRECISION),int(rd.uniform(0,1)*PRECISION))
##        if i%2 == 0:
##            ori = (int(rd.uniform(0,0.2)*PRECISION),int(rd.uniform(0,0.2)*PRECISION))
##        if i%2 == 1:
##            ori = (int(rd.uniform(0.8,1)*PRECISION),int(rd.uniform(0.8,1)*PRECISION))
##        des = (int(rd.uniform(0.4,0.7)*PRECISION),int(rd.uniform(0.4,0.7)*PRECISION))
##        
##    ##    tim = int(rd.uniform(0,max(T/2-2-Distance(ori,des),1)))
##    ####    delta = int(rd.uniform(0.8,1)*PRECISION)
##    ##    delta = int(rd.uniform(0.5,1)*T)    
##    ##    etim = min(tim+delta+Distance(ori,des),T-1)
##        tim = 0
##        etim = T
##    ##    delta = int(rd.uniform(0,1)*PRECISION)
##    ##    etim = min(tim+delta+Distance(ori,des),T-1)
##        cap = 4
##        drivers.append(Driver(ori,des,tim,etim,cap))
##        print(ori,des,tim,etim,cap)
##
##    print("REQUESTS")                   
##    for i in range(R): 
##    ##    ori = (int(rd.uniform(0,1)*PRECISION),int(rd.uniform(0,1)*PRECISION))
##    ##    des = (int(rd.uniform(0,1)*PRECISION),int(rd.uniform(0,1)*PRECISION))
##        if i%2 == 0:
##            ori = (int(rd.uniform(0,0.2)*PRECISION),int(rd.uniform(0,0.2)*PRECISION))
##        if i%2 == 1:
##            ori = (int(rd.uniform(0.8,1)*PRECISION),int(rd.uniform(0.8,1)*PRECISION))
##        des = (int(rd.uniform(0.4,0.7)*PRECISION),int(rd.uniform(0.4,0.7)*PRECISION))
##        
##        tim = int(rd.uniform(0,max(T/2-2-Distance(ori,des),1)))
##        delta = int(rd.uniform(0,1)*PRECISION)
##        etim = min(tim+delta+Distance(ori,des),T-1)
##        ptim = tim+delta//2
##    ##    tim = 0
##    ##    etim = T-1
##        reqs.append(Passenger(ori,des,tim,etim,ptim))
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


    drivers = []
    reqs =[]

    ####DRIVERS
    drivers.append(Driver( (1, 4) , (18, 12) , 0 , 59 , 4 ))
    drivers.append(Driver( (29, 24) , (17, 20) , 0 , 59 , 4 ))
    drivers.append(Driver( (27, 27) , (20, 15) , 0 , 59 , 4 ))
    drivers.append(Driver( (28, 26) , (15, 15) , 0 , 59 , 4 ))
    drivers.append(Driver( (24, 25) , (20, 16) , 0 , 59 , 4 ))
    ####REQUESTS
    reqs.append(Passenger( (0, 0) , (20, 13) , 2 , 54 , 16 ))
    reqs.append(Passenger( (29, 24) , (19, 13) , 2 , 44 , 16 ))
    reqs.append(Passenger( (5, 2) , (16, 19) , 3 , 30 , 6 ))
    reqs.append(Passenger( (26, 29) , (14, 13) , 6 , 37 , 11 ))
    
    m,x,v,t,G,c = MILP(drivers,reqs)

import numpy as np
import networkx as nx
import pylab
import time
import random as rd
import gurobipy as grb
from Classes import Driver
from Classes import Passenger
from Preprocessing_4 import Preprocessing
import math

MUTE = 0 #set to 0 to MUTE
PRUN1 = 0 #set to 1 to do PRUNING
DECOMPOSITION = 1 #set to 1 to do decomposition
TIMELIMIT = 1500
timeSwitch = 0 # set to 1 to do TIMELIMIT

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

    def DeleteEdges(EDGE,i):
        if set(G.out_edges(EDGE[0])).intersection(infeaEdge[i]) == set(G.in_edges(EDGE[0])):
            infeaEdge[i] = infeaEdge[i].union(set(G.in_edges(EDGE[0])))
            for e in G.in_edges(EDGE[0]):
                DeleteEdges(e,i)
        
##    feaEdge = [set() for i in range(D)]
    infeaEdge = [set() for i in range(D)]
    feasibleMat = np.zeros((D,R), dtype = int)

    # ADDING EDGES
    for i in range(D):
        dIndex = 'd'+str(i)
        for t in range(drivers[i].et,drivers[i].lt-DIST(drivers[i])+1):
            G.add_node((dIndex, 0,t))                   # add origin
            G.add_node((dIndex, 1,t+DIST(drivers[i])))  # add destin
            VDO.add((dIndex, 0,t))
            VDD.add((dIndex, 1,t+DIST(drivers[i])))
            
            # and driver's origin to driver's destin
            EDGE = ((dIndex,0,t),(dIndex,1,t+DIST(drivers[i])))
            G.add_edge(EDGE[0],EDGE[1])
            BDO[i].add(EDGE)
            EDD[i].add(EDGE)
##            feaEdge[i].add(EDGE)


        # we only need edge from driver's origin to req's origin
        for j in range(R):
            for t in range(drivers[i].et,drivers[i].lt-DIST(drivers[i])+1):
                if t+Distance(retLocation(dIndex,0),reqs[j].ori) < reqs[j].et and reqs[j].et+DIST(reqs[j])+Distance(reqs[j].des,retLocation(dIndex,1)) <= drivers[i].lt:
                    EDGE = ((dIndex,0,t),(j,0,t+Distance(retLocation(dIndex,0),reqs[j].ori)))
                    G.add_edge(EDGE[0],EDGE[1])
                    BDO[i].add(EDGE)
##                    feaEdge[i].add(EDGE)
                    G.add_edge((j,0,t+Distance(retLocation(dIndex,0),reqs[j].ori)),(j,0,reqs[j].et))
##                    feaEdge[i].add(((j,0,t+Distance(retLocation(dIndex,0),reqs[j].ori)),(j,0,reqs[j].et)))
                    if t == drivers[i].et:
                        feasibleMat[i][j] = 1
                elif t+Distance(retLocation(dIndex,0),reqs[j].ori) <= reqs[j].lt-DIST(reqs[j]) and  t+Distance(retLocation(dIndex,0),reqs[j].ori)+DIST(reqs[j])+Distance(reqs[j].des,retLocation(dIndex,1)) <= drivers[i].lt:
                    EDGE = ((dIndex,0,t),(j,0,math.ceil(t+Distance(retLocation(dIndex,0),reqs[j].ori))))
                    G.add_edge(EDGE[0],EDGE[1])
                    BDO[i].add(EDGE)
##                    feaEdge[i].add(EDGE)
                    if t == drivers[i].et:
                        feasibleMat[i][j] = 1
                else:   # we don't have to check rest of t
                    break
                                         


    if MUTE != 0: print(feasibleMat)                


        
    
    for j in range(R):
        for t in range(reqs[j].et,reqs[j].lt-DIST(reqs[j])+1):
            G.add_node((j,0,t))
            G.add_node((j,1,t+DIST(reqs[j])))
            if t != reqs[j].lt-DIST(reqs[j]):
                G.add_edge((j,0,t),(j,0,t+1))
                G.add_edge((j,1,t+DIST(reqs[j])),(j,1,t+DIST(reqs[j])+1))
            
            if len(G.in_edges((j,0,t))) ==0:
                continue
            # add req's origin to req's destin
            EDGE = ((j,0,t),(j,1,t+DIST(reqs[j])))
            G.add_edge(EDGE[0],EDGE[1])
            BRO[j].add(EDGE)
            ERD[j].add(EDGE)

            # add edge j's destin to driver's destin
            for i in range(D):
                tt = t+DIST(reqs[j])
                dIndex = 'd'+str(i)
                if tt+Distance(retLocation(j,1),drivers[i].des) < drivers[i].et+DIST(drivers[i]):
                    pass
                elif tt+Distance(retLocation(j,1),drivers[i].des) <= drivers[i].lt:
                    EDGE = ((j,1,tt),(dIndex,1,math.ceil(tt+Distance(retLocation(j,1),drivers[i].des))))
                    G.add_edge(EDGE[0],EDGE[1])
                    BRD[j].add(EDGE)
                    EDD[i].add(EDGE)
                    VDD.add((dIndex,1,math.ceil(tt+Distance(retLocation(j,1),drivers[i].des))))

        # ADD edge betweem 2 reqs            
        for k in range(R):
            if j == k: continue
            
            # add j's origin to k's origin
            for t in range(reqs[j].et,reqs[j].lt-DIST(reqs[j])+1):
##                if len(G.in_edges((j,0,t))) ==0:
##                    continue
                if (retLocation(j,0) == retLocation(k,0)).all() and j>k:
                    pass
                elif t+Distance(retLocation(j,0),reqs[k].ori) < reqs[k].et and (reqs[k].et+DIST(reqs[k])+Distance(reqs[k].des,reqs[j].des) <= reqs[j].lt or reqs[k].et+Distance(reqs[k].ori,reqs[j].des)+Distance(reqs[j].des,reqs[k].des) <= reqs[k].lt):           
                    EDGE = ((j,0,t),(k,0,t+Distance(retLocation(j,0),reqs[k].ori)))
                    G.add_edge(EDGE[0],EDGE[1])
                    BRO[j].add(EDGE)
                    G.add_edge((k,0,t+Distance(retLocation(j,0),reqs[k].ori)),(k,0,reqs[k].et))

                    for i in range(D):
                        TIME = min(reqs[k].et+DIST(reqs[k])+Distance(reqs[k].des,reqs[j].des)+Distance(reqs[j].des,drivers[i].des),reqs[k].et+Distance(reqs[k].ori,reqs[j].des)+Distance(reqs[j].des,reqs[k].des)+Distance(reqs[k].des,drivers[i].des))
                        if TIME > drivers[i].lt:
                            infeaEdge[i].add(EDGE)
                            DeleteEdges(EDGE,i)
                elif t+Distance(retLocation(j,0),reqs[k].ori) <= reqs[k].lt-DIST(reqs[k]) and (t+Distance(retLocation(j,0),reqs[k].ori)+DIST(reqs[k])+Distance(reqs[k].des,reqs[j].des) <= reqs[j].lt or t+Distance(retLocation(j,0),reqs[k].ori)+Distance(reqs[k].ori,reqs[j].des)+Distance(reqs[j].des,reqs[k].des) <= reqs[k].lt):
                    EDGE = ((j,0,t),(k,0,math.ceil(t+Distance(retLocation(j,0),reqs[k].ori))))
                    G.add_edge(EDGE[0],EDGE[1])
                    BRO[j].add(EDGE)

                    for i in range(D):
                        TIME = min(t+Distance(retLocation(j,0),reqs[k].ori)+DIST(reqs[k])+Distance(reqs[k].des,reqs[j].des)+Distance(reqs[j].des,drivers[i].des),t+Distance(retLocation(j,0),reqs[k].ori)+Distance(reqs[k].ori,reqs[j].des)+Distance(reqs[j].des,reqs[k].des)+Distance(reqs[k].des,drivers[i].des))
                        if TIME > drivers[i].lt:
                            infeaEdge[i].add(EDGE)
                            DeleteEdges(EDGE,i)
                else: break

            
            # add j's origin to k's destin
            for t in range(reqs[j].et,reqs[j].lt-DIST(reqs[j])+1):
##                if len(G.in_edges((j,0,t))) ==0:
##                    pass
                if (retLocation(j,0) == retLocation(k,1)).all() and j>k:
                    pass
                elif t+Distance(retLocation(j,0),reqs[k].des) < reqs[k].et+DIST(reqs[k]):
                    pass
                elif t+Distance(retLocation(j,0),reqs[k].des) <= reqs[k].lt and t+Distance(retLocation(j,0),reqs[k].des)+Distance(reqs[k].des,reqs[j].des) <= reqs[j].lt:
                    EDGE = ((j,0,t),(k,1,math.ceil(t+Distance(retLocation(j,0),reqs[k].des))))
                    G.add_edge(EDGE[0],EDGE[1])
                    BRO[j].add(EDGE)
                    ERD[k].add(EDGE)

                    for i in range(D):
                        TIME = t+Distance(retLocation(j,0),reqs[k].des)+Distance(reqs[k].des,reqs[j].des)+Distance(reqs[j].des,drivers[i].des)
                        if TIME > drivers[i].lt:
                            infeaEdge[i].add(EDGE)
                            DeleteEdges(EDGE,i)
                else: break


                
            # add j's destin to k's origin
            for t in range(reqs[j].et,reqs[j].lt-DIST(reqs[j])+1):
##                if len(G.in_edges((j,0,t))) ==0:
##                    continue
                tt = t+DIST(reqs[j])
                if (retLocation(j,1) == retLocation(k,0)).all() and j>k:
                    pass
                elif  tt+Distance(retLocation(j,1),reqs[k].ori) < reqs[k].et:
                    EDGE = ((j,1,tt),(k,0,tt+Distance(retLocation(j,1),reqs[k].ori)))
                    G.add_edge(EDGE[0],EDGE[1])
                    BRD[j].add(EDGE)
                    G.add_edge((k,0,tt+Distance(retLocation(j,1),reqs[k].ori)),(k,0,reqs[k].et))

                    for i in range(D):
                        TIME = reqs[k].et+DIST(reqs[k])+Distance(reqs[k].des,drivers[i].des)
                        if TIME > drivers[i].lt:
                            infeaEdge[i].add(EDGE)
                            DeleteEdges(EDGE,i)

                elif tt+Distance(retLocation(j,1),reqs[k].ori) <= reqs[k].lt-DIST(reqs[k]):
                    EDGE = ((j,1,tt),(k,0,math.ceil(tt+Distance(retLocation(j,1),reqs[k].ori))))
                    G.add_edge(EDGE[0],EDGE[1])
                    BRD[j].add(EDGE)

                    for i in range(D):
                        TIME = tt+Distance(retLocation(j,1),reqs[k].ori)+DIST(reqs[k])+Distance(reqs[k].des,drivers[i].des)
                        if TIME > drivers[i].lt:
                            infeaEdge[i].add(EDGE)
                            DeleteEdges(EDGE,i)
                                        
                else: break

                    
            # add j's destin to k's destin  
            for t in range(reqs[j].et,reqs[j].lt-DIST(reqs[j])+1):  
##                if len(G.in_edges((j,0,t))) ==0:
##                    continue
                tt = t+DIST(reqs[j])                  
                if (retLocation(j,1) == retLocation(k,1)).all() and j>k:
                    pass         
                elif tt+Distance(retLocation(j,1),reqs[k].des) < reqs[k].et+DIST(reqs[k]):
                    pass

                elif tt+Distance(retLocation(j,1),reqs[k].des) <= reqs[k].lt:
                    EDGE = ((j,1,tt),(k,1,math.ceil(tt+Distance(retLocation(j,1),reqs[k].des))))
                    G.add_edge(EDGE[0],EDGE[1])
                    BRD[j].add(EDGE)
                    ERD[k].add(EDGE)

                    for i in range(D):
                        TIME = tt+Distance(retLocation(j,1),reqs[k].des)+Distance(reqs[k].des,drivers[i].des)
                        if TIME > drivers[i].lt:
                            infeaEdge[i].add(EDGE)
                            DeleteEdges(EDGE,i)
                else: break

##    for j in range(R):
##        for t in range(reqs[j].et,reqs[j].lt-DIST(reqs[j])+1):
##            if len(G.in_edges((j,0,t))) == 0:
##                print(j,t)
##                G.remove_node((j,0,t))
##            else:
##                break
            

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
        for e in infeaEdge[i]:
            m.addConstr(y[e][i] == 0)
        
    


    m.setObjective(grb.quicksum(c[e]*y[e][i] for e in G.edges() for i in range(D)) + LAMBDA*grb.quicksum(z[j] for j in range(R)),
                   grb.GRB.MINIMIZE)


    m.setParam('OutputFlag', MUTE)
    if timeSwitch!= 0: m.setParam('TimeLimit', TIMELIMIT)
    m.optimize()

    if m.status == 3:
        print("INPUT IS INFEASIBE")
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
##                    print(e in infeaEdge[j])

        for i in range(len(z)):
            if z[i].x>0:
                print(i)

##    
##    return m,x, m.ObjVal, ENDTIME-BEGINTIME 

    return m,y,m.ObjVal,ENDTIME-BEGINTIME,G,infeaEdge


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

####    DRIVERS
####    drivers.append(Driver( (4, 3) , (18, 13) , 0 , 26 , 4 ))
####    drivers.append(Driver( (24, 24) , (16, 14) , 0 , 26 , 4 ))
####    drivers.append(Driver( (25, 27) , (20, 14) , 0 , 26 , 4 ))
####    drivers.append(Driver( (29, 28) , (14, 14) , 0 , 26 , 4 ))
####    drivers.append(Driver( (29, 26) , (20, 16) , 0 , 26 , 4 ))
####    REQUESTS
####    reqs.append(Passenger( (2, 3) , (13, 20) , 0 , 25 , 14 ))
####    reqs.append(Passenger( (25, 25) , (20, 13) , 0 , 22 , 4 ))
####    reqs.append(Passenger( (4, 5) , (16, 13) , 0 , 16 , 1 ))
####    reqs.append(Passenger( (24, 28) , (18, 15) , 0 , 25 , 13 ))
####    reqs.append(Passenger( (5, 2) , (15, 16) , 0 , 25 , 6 ))
####    reqs.append(Passenger( (25, 25) , (18, 17) , 0 , 22 , 6 ))
####    reqs.append(Passenger( (1, 5) , (13, 15) , 0 , 22 , 3 ))
####    reqs.append(Passenger( (26, 25) , (12, 12) , 0 , 25 , 3 ))
####    reqs.append(Passenger( (5, 5) , (16, 14) , 0 , 25 , 12 ))
####    reqs.append(Passenger( (26, 28) , (13, 14) , 0 , 22 , 1 ))
####    reqs.append(Passenger( (5, 3) , (12, 12) , 0 , 23 , 6 ))
####    reqs.append(Passenger( (26, 29) , (12, 12) , 0 , 25 , 13 ))
####    reqs.append(Passenger( (3, 2) , (17, 15) , 0 , 25 , 10 ))
####    reqs.append(Passenger( (24, 29) , (14, 12) , 0 , 25 , 9 ))
####    reqs.append(Passenger( (4, 3) , (16, 16) , 0 , 25 , 5 ))
####    reqs.append(Passenger( (29, 25) , (12, 12) , 0 , 25 , 14 ))
####    reqs.append(Passenger( (0, 5) , (13, 20) , 0 , 25 , 3 ))
####    reqs.append(Passenger( (29, 28) , (16, 16) , 0 , 25 , 6 ))
####    reqs.append(Passenger( (3, 4) , (15, 14) , 0 , 22 , 3 ))
####    reqs.append(Passenger( (25, 25) , (14, 19) , 0 , 25 , 11 ))
####    reqs.append(Passenger( (4, 3) , (14, 12) , 0 , 25 , 8 ))
####    reqs.append(Passenger( (24, 25) , (19, 12) , 0 , 25 , 11 ))
####    reqs.append(Passenger( (5, 5) , (12, 16) , 0 , 25 , 6 ))
####    reqs.append(Passenger( (24, 24) , (17, 17) , 1 , 25 , 13 ))
####    reqs.append(Passenger( (1, 1) , (15, 12) , 0 , 25 , 4 ))
####    reqs.append(Passenger( (27, 29) , (20, 15) , 0 , 25 , 11 ))
####    reqs.append(Passenger( (4, 4) , (17, 18) , 0 , 24 , 2 ))
####    reqs.append(Passenger( (29, 29) , (19, 18) , 0 , 17 , 1 ))
####    reqs.append(Passenger( (4, 4) , (20, 16) , 0 , 25 , 8 ))
####    reqs.append(Passenger( (28, 24) , (19, 19) , 0 , 25 , 9 ))
####    reqs.append(Passenger( (0, 3) , (14, 16) , 0 , 25 , 4 ))
####    reqs.append(Passenger( (28, 26) , (16, 19) , 0 , 18 , 2 ))
####    reqs.append(Passenger( (4, 4) , (12, 17) , 0 , 15 , 0 ))
####    reqs.append(Passenger( (27, 28) , (18, 17) , 0 , 16 , 1 ))
####    reqs.append(Passenger( (2, 0) , (12, 15) , 0 , 25 , 13 ))
####    reqs.append(Passenger( (26, 26) , (18, 18) , 0 , 22 , 5 ))
####    reqs.append(Passenger( (3, 4) , (12, 17) , 0 , 19 , 2 ))
####    reqs.append(Passenger( (27, 28) , (15, 12) , 0 , 25 , 11 ))
####    reqs.append(Passenger( (3, 3) , (17, 14) , 0 , 25 , 5 ))
####    reqs.append(Passenger( (26, 25) , (12, 15) , 0 , 20 , 1 ))
####    reqs.append(Passenger( (5, 5) , (13, 15) , 0 , 16 , 2 ))
####    reqs.append(Passenger( (28, 24) , (20, 15) , 0 , 13 , 0 ))
####    reqs.append(Passenger( (5, 0) , (17, 13) , 0 , 25 , 8 ))
####    reqs.append(Passenger( (27, 28) , (18, 18) , 0 , 25 , 11 ))
####    reqs.append(Passenger( (1, 0) , (18, 13) , 0 , 25 , 5 ))
####    reqs.append(Passenger( (28, 24) , (20, 17) , 0 , 25 , 8 ))
####    reqs.append(Passenger( (4, 4) , (20, 17) , 0 , 25 , 3 ))


    drivers = []
    reqs =[]

    ##DRIVERS
    drivers.append(Driver( (5, 5) , (13, 17) , 0 , 59 , 4 ))
    drivers.append(Driver( (25, 27) , (12, 13) , 0 , 59 , 4 ))
    drivers.append(Driver( (29, 28) , (12, 15) , 0 , 59 , 4 ))
    drivers.append(Driver( (24, 26) , (17, 19) , 0 , 59 , 4 ))
    drivers.append(Driver( (26, 28) , (20, 15) , 0 , 59 , 4 ))
    drivers.append(Driver( (25, 27) , (15, 18) , 0 , 59 , 4 ))
    ##REQUESTS
    reqs.append(Passenger( (4, 0) , (19, 12) , 4 , 42 , 13 ))
    reqs.append(Passenger( (24, 28) , (19, 13) , 5 , 46 , 18 ))
    reqs.append(Passenger( (0, 5) , (12, 17) , 7 , 24 , 7 ))
    reqs.append(Passenger( (25, 27) , (17, 17) , 1 , 33 , 11 ))
    reqs.append(Passenger( (0, 0) , (15, 16) , 3 , 31 , 6 ))
    reqs.append(Passenger( (25, 24) , (12, 15) , 5 , 28 , 9 ))
    reqs.append(Passenger( (0, 4) , (19, 19) , 2 , 35 , 6 ))
    reqs.append(Passenger( (24, 25) , (14, 14) , 12 , 53 , 25 ))
    reqs.append(Passenger( (0, 5) , (16, 12) , 7 , 39 , 14 ))
    reqs.append(Passenger( (29, 25) , (13, 17) , 7 , 25 , 7 ))
    reqs.append(Passenger( (0, 5) , (15, 12) , 0 , 16 , 0 ))
    reqs.append(Passenger( (28, 27) , (18, 20) , 6 , 40 , 17 ))
    reqs.append(Passenger( (2, 4) , (14, 13) , 3 , 24 , 6 ))
    reqs.append(Passenger( (27, 26) , (15, 13) , 9 , 38 , 15 ))    


    
    m,x,v,t,G,c = MILP(drivers,reqs)
    print(v,t)

import numpy as np
import networkx as nx
import pylab
import time
import random as rd
import gurobipy as grb
from Classes import Driver
from Classes import Passenger
import math



LAMBDA = 100
# Return distance between i's orgin to i's destination
def DIST(i):
    return Distance(i.ori,i.des)

def Distance(i,j):
    if type(i) != type(np.array([1])): i = np.array(i)
    if type(j) != type(np.array([1])): j = np.array(j)
##    if (i==j).all():
##        return 0.1
    return int(np.linalg.norm(i-j))

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
##    BDD = [set() for i in range(D)]     #set of edges starting at driver's destination ### THIS SHOULD BE EMPTY
    BRO = [set() for i in range(R)]     #set of edges starting at rider's origin
    BRD = [set() for i in range(R)]     #set of edges starting at rider's destination
    EDD = [set() for i in range(D)]     #set of edges ending at driver's destination 
##    RD = [set() for i in range(D)]      #set of edges ending at rider's destination
##    RO = [set() for i in range(D)]      #set of edges starting at rider's origin
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

            for k in range(R):
                if j == k: continue
                # add j's origin to k's origin
                if  t+Distance(retLocation(j,0),reqs[k].ori) < reqs[k].et:
                    G.add_edge((j,0,t),(k,0,t+Distance(retLocation(j,0),reqs[k].ori)))
                    BRO[j].add(((j,0,t),(k,0,t+Distance(retLocation(j,0),reqs[k].ori))))
                    G.add_edge((k,0,t+Distance(retLocation(j,0),reqs[k].ori)),(k,0,reqs[k].et))
                elif t+Distance(retLocation(j,0),reqs[k].ori) <= reqs[k].lt-DIST(reqs[k]):
                    G.add_edge((j,0,t),(k,0,math.ceil(t+Distance(retLocation(j,0),reqs[k].ori))))
                    BRO[j].add(((j,0,t),(k,0,math.ceil(t+Distance(retLocation(j,0),reqs[k].ori)))))

                # add j's origin to k's destin
                if t+Distance(retLocation(j,0),reqs[k].des) < reqs[k].et+DIST(reqs[k]):
                    G.add_edge((j,0,t),(k,1,t+Distance(retLocation(j,0),reqs[k].des)))
                    BRO[j].add(((j,0,t),(k,1,t+Distance(retLocation(j,0),reqs[k].des))))
                    G.add_edge((k,1,t+Distance(retLocation(j,0),reqs[k].des)),(k,1,reqs[k].et+DIST(reqs[k])))
                elif t+Distance(retLocation(j,0),reqs[k].des) <= reqs[k].lt:
                    G.add_edge((j,0,t),(k,1,math.ceil(t+Distance(retLocation(j,0),reqs[k].des))))
                    BRO[j].add(((j,0,t),(k,1,math.ceil(t+Distance(retLocation(j,0),reqs[k].des)))))

                # add j's destin to k's origin
                if  t+Distance(retLocation(j,1),reqs[k].ori) < reqs[k].et:
                    G.add_edge((j,1,t),(k,0,t+Distance(retLocation(j,1),reqs[k].ori)))
                    BRD[j].add(((j,1,t),(k,0,t+Distance(retLocation(j,1),reqs[k].ori))))
                    G.add_edge((k,0,t+Distance(retLocation(j,1),reqs[k].ori)),(k,0,reqs[k].et))
                elif t+Distance(retLocation(j,1),reqs[k].ori) <= reqs[k].lt-DIST(reqs[k]):
                    G.add_edge((j,1,t),(k,0,math.ceil(t+Distance(retLocation(j,0),reqs[k].ori))))
                    BRD[j].add(((j,1,t),(k,0,math.ceil(t+Distance(retLocation(j,0),reqs[k].ori)))))
                    
                # add j's destin to k's destin           
                if t+Distance(retLocation(j,1),reqs[k].des) < reqs[k].et+DIST(reqs[k]):
                    G.add_edge((j,1,t),(k,1,t+Distance(retLocation(j,0),reqs[k].des)))
                    BRD[j].add(((j,1,t),(k,1,t+Distance(retLocation(j,0),reqs[k].des))))
                    G.add_edge((k,1,t+Distance(retLocation(j,0),reqs[k].des)),(k,1,reqs[k].et+DIST(reqs[k])))
                elif t+Distance(retLocation(j,1),reqs[k].des) <= reqs[k].lt:
                    G.add_edge((j,1,t),(k,1,math.ceil(t+Distance(retLocation(j,1),reqs[k].des))))
                    BRD[j].add(((j,1,t),(k,1,math.ceil(t+Distance(retLocation(j,1),reqs[k].des)))))

            # add edge j's destin to driver's destin
            for i in range(D):
                dIndex = 'd'+str(i)
                if t+Distance(retLocation(j,1),drivers[i].des) < drivers[i].et+DIST(drivers[i]):
                    G.add_edge((j,1,t),(dIndex,1,t+Distance(retLocation(j,1),drivers[i].des)))
                    BRD[j].add(((j,1,t),(dIndex,1,t+Distance(retLocation(j,1),drivers[i].des))))
                    EDD[i].add(((j,1,t),(dIndex,1,t+Distance(retLocation(j,1),drivers[i].des))))
                    VDD.add((dIndex,1,t+Distance(retLocation(j,1),drivers[i].des)))
                    G.add_edge((dIndex,1,t+Distance(retLocation(j,1),drivers[i].des)),(dIndex,1,drivers[i].et+DIST(drivers[i])))
                elif t+Distance(retLocation(j,1),drivers[i].des) < drivers[i].lt:
                    G.add_edge((j,1,t),(dIndex,1,math.ceil(t+Distance(retLocation(j,1),drivers[i].des))))
                    BRD[j].add(((j,1,t),(dIndex,1,math.ceil(t+Distance(retLocation(j,1),drivers[i].des)))))
                    EDD[i].add(((j,1,t),(dIndex,1,math.ceil(t+Distance(retLocation(j,1),drivers[i].des)))))
                    VDD.add((dIndex,1,math.ceil(t+Distance(retLocation(j,1),drivers[i].des))))



    

##    # Add edges for each driver separately for now.
##    for i in range(D):
##        inpCur = [] #contain (x,y),et,lt,pt,index,driv/req,ori/des
##        inpCur.append([drivers[i].ori, drivers[i].et, drivers[i].lt-DIST(drivers[i]), drivers[i].et, i,0,0])
##        for j in range(R):
##            if drivers[i].et+Distance(drivers[i].ori,reqs[j].ori)+Distance(reqs[j].ori,reqs[j].des) <= reqs[j].lt and drivers[i].et+Distance(drivers[i].ori,reqs[j].ori)+Distance(reqs[j].ori,reqs[j].des)+Distance(reqs[j].des,drivers[i].des) <= drivers[i].lt:
##                inpCur.append([reqs[j].ori,reqs[j].et,reqs[j].lt-DIST(reqs[j]),reqs[j].pt, j,1,0])
##                inpCur.append([reqs[j].des,reqs[j].et+DIST(reqs[j]),reqs[j].lt,reqs[j].pt+DIST(reqs[j]),j,1,1])
##        inpCur.append([drivers[i].des, drivers[i].et+DIST(drivers[i]),drivers[i].lt,drivers[i].lt,i,0,1])
##        
##        print(inpCur)
##        
# P contain (x1,y1) (x2,y2) t index driver/req ori/des pt
##% each P contain (x1, y1, x2, y2, t, index, driver/req,
##% ori/des, preferred time, which network does it belong to?
                
##        for ii in range(len(inpCur)):
##            for jj in range(len(inpCur)):
##                if ii == jj: continue
##                if jj == 0 : continue
##                if ii == len(inpCur)-1 : continue
##                G.add_node((i,inpCur[jj][5],inpCur[jj][6],inpCur[jj][7],inpCur[jj][2]))
##                
##                for tt in range(inpCur[ii][1],inpCur[ii][2]+1):
##                    
##                    if Distance(inpCur[ii][0],inpCur[jj][0])+tt > inpCur[jj][2]:
##                        continue
##                    if (inpCur[ii][0] == inpCur[jj][0]).all() and ii > jj:
##                        continue
##                    G.add_node((i,inpCur[ii][5],inpCur[ii][6],inpCur[ii][7],tt))
##                    G.add_node((i,inpCur[jj][5],inpCur[jj][6],inpCur[jj][7],tt+Distance(inpCur[ii][0],inpCur[jj][0])))
##                    G.add_edge((i,inpCur[ii][5],inpCur[ii][6],inpCur[ii][7],tt),(i,inpCur[jj][5],inpCur[jj][6],inpCur[jj][7],tt+Distance(inpCur[ii][0],inpCur[jj][0])))
##                    if tt+Distance(inpCur[ii][0],inpCur[jj][0]) < inpCur[jj][1]:
##                          G.add_edge((i,inpCur[jj][5],inpCur[jj][6],inpCur[jj][7],tt+Distance(inpCur[ii][0],inpCur[jj][0])),(i,inpCur[jj][5],inpCur[jj][6],inpCur[jj][7],inpCur[jj][1]))
##                          
##                    P.append([inpCur[ii][0], inpCur[jj][0], tt, inpCur[ii][4], inpCur[ii][5],inpCur[ii][6],inpCur[ii][3],i])
##                    if jj == len(inpCur)-1: EDD[i].add(((i,inpCur[ii][5],inpCur[ii][6],inpCur[ii][7],tt),(i,inpCur[jj][5],inpCur[jj][6],inpCur[jj][7],tt+Distance(inpCur[ii][0],inpCur[jj][0]))))
##                    if P[-1][4] == 0 and P[-1][5] == 0: BDO[P[-1][3]].add(((i,inpCur[ii][5],inpCur[ii][6],inpCur[ii][7],tt),(i,inpCur[jj][5],inpCur[jj][6],inpCur[jj][7],tt+Distance(inpCur[ii][0],inpCur[jj][0]))))
##                    elif P[-1][4] == 0 and P[-1][5] == 1: BDD[P[-1][3]].add(((i,inpCur[ii][5],inpCur[ii][6],inpCur[ii][7],tt),(i,inpCur[jj][5],inpCur[jj][6],inpCur[jj][7],tt+Distance(inpCur[ii][0],inpCur[jj][0]))))
##                    elif P[-1][4] == 1 and P[-1][5] == 0:
##                        BRO[P[-1][3]].add(((i,inpCur[ii][5],inpCur[ii][6],inpCur[ii][7],tt),(i,inpCur[jj][5],inpCur[jj][6],inpCur[jj][7],tt+Distance(inpCur[ii][0],inpCur[jj][0]))))
##                        RO[i].add(((i,inpCur[ii][5],inpCur[ii][6],inpCur[ii][7],tt),(i,inpCur[jj][5],inpCur[jj][6],inpCur[jj][7],tt+Distance(inpCur[ii][0],inpCur[jj][0]))))
##                    elif P[-1][4] == 1 and P[-1][5] == 1:
##                        BRD[P[-1][3]].add(((i,inpCur[ii][5],inpCur[ii][6],inpCur[ii][7],tt),(i,inpCur[jj][5],inpCur[jj][6],inpCur[jj][7],tt+Distance(inpCur[ii][0],inpCur[jj][0]))))
##                        RD[i].add(((i,inpCur[ii][5],inpCur[ii][6],inpCur[ii][7],tt),(i,inpCur[jj][5],inpCur[jj][6],inpCur[jj][7],tt+Distance(inpCur[ii][0],inpCur[jj][0]))))
##        
        
##                    if jj == len(inpCur)-1:
##                        EDD[i].add(len(P)-1)

    BD = set()
    ED = set()
    for i in range(D):
        BD = BD.union(BDO[i])
        ED = ED.union(EDD[i])
        
    RO = set()
    RD = set()
    for j in range(R):
        RO = RO.union(BRO[j])
        RD = RD.union(BRD[j])


    print("\nConstruction Time: %f\n\n" %(time.clock()-BEGINTIME))

        
##    c = [0 for i in range(len(P))]
##    LAMBDA1 = 0.2
##    for p in range(len(P)):
##        if p in BD:
##            c[p] = Distance(P[p][0],P[p][1])
##        else:
##            c[p] = Distance(P[p][0],P[p][1])+LAMBDA1*abs(P[p][2]-P[p][6])


    c = {}
    LAMBDA1 = 0.2
    for e in G.edges():
        if e in BD:
            c[e] = DistOfEdge(e)
        else:
            c[e] = DistOfEdge(e) + LAMBDA1*abs(retPt(e[0][0],e[0][1])-e[0][2])

    m = grb.Model("COPYMILP")


##    feasiblePQ = []
##    for i in range(len(P)):
##        feasiblePQ.append(set())
##        for j in range(len(P)):
##            if P[i][7] != P[j][7]:
##                continue
##            elif P[i][3] == P[j][3] and P[i][4] == P[j][4] and P[i][5] == P[j][5]:
##                continue
##            elif (P[i][1] != P[j][0]).any() or P[i][2] + Distance(P[i][1],P[j][0]) > P[j][2]:
##                continue
##            else:
##                feasiblePQ[i].add(j)

    
##    x = []
##    for i in range(len(P)):
##        x.append([])
####        x[i] = m.addVars(feasiblePQ[i], vtype=grb.GRB.BINARY, lb=0, ub=1)
##        x[i] = m.addVars(len(P), vtype = grb.GRB.BINARY, lb=0, ub=1)
            
##    y = m.addVars(len(P), vtype = grb.GRB.BINARY, lb=0, ub=1)
####    y = []
####    for i in range(len(P)):
####        y.append([])
####        y[i] = m.addVars(D, vtype = grb.GRB.BINARY, lb=0, ub=1)

    y = {}
    for e in G.edges():
        y[e] = m.addVars(D, vtype=grb.GRB.BINARY, lb=0, ub=1)

    z = m.addVars(R, vtype = grb.GRB.BINARY, lb=0, ub=1)


### ADD CONSTRAINTS ###########################

    def feaPQ(q):
        for p in range(len(P)):
            if q in feasiblePQ[p]:
                yield p
    def OrinTim(t,i):
        for p in RO[i]:
            if P[p][2] <= t: yield p
    def DesTim(t,i):
        for p in RD[i]:
            if P[p][2] <= t: yield p
    def OriRTim(t,i):
        for p in BRO[i]:
            if P[p][2] <= t: yield p
    def DesRTim(t,i):
        for p in BRD[i]:
            if P[p][2] <= t: yield p        
        


##    for q in range(len(P)):
##        if q not in [BDO[i] for i in range(D)]:
##            m.addConstr(grb.quicksum(x[p][q] for p in feaPQ(q)) == y[q])
##        if q not in [EDD[i] for i in range(D)]:    
##            m.addConstr(grb.quicksum(x[q][p] for p in feasiblePQ[q]) == y[q])
##

    for i in range(D):
        # FLOW CONSERVATION CONSTRAINT
        for v in G.nodes():
            if v in VDD: continue
            if v in VDO: continue
            m.addConstr(grb.quicksum(y[e][i] for e in G.in_edges(v))
                        ==
                        grb.quicksum(y[e][i] for e in G.out_edges(v))
                        )

        m.addConstr(grb.quicksum(y[e][i] for e in BDO[i])
                    == 1)
        m.addConstr(grb.quicksum(y[e][i] for e in EDD[i])
                    == 1)
        
                    

    

    
##    for q in range(len(P)):
##        if q not in BD:
##            m.addConstr(grb.quicksum(x[p][q] for p in range(len(P))) == y[q])
##        if q not in ED:    
##            m.addConstr(grb.quicksum(x[q][p] for p in range(len(P))) == y[q])
##


##    for j in range(R):
##        m.addConstr(z[j] + grb.quicksum(y[p] for p in BRO[j]) == 1)
##        m.addConstr(grb.quicksum(y[p] for p in BRO[j]) == grb.quicksum(y[p] for p in BRD[j]))
##        
##

##    for t in range(T):
##        for i in range(D):
##            print(t,i)
##            print(sum(S[p] for p in OrinTim(t,i)))
##            print(sum(S[p] for p in DesTim(t,i)))
##            print('\n')
                

    #CAPACITY CONSTRAINTS
    for t in range(T):
        for i in range(D):
            m.addConstr(grb.quicksum(y[p] for p in OrinTim(t,i)) - grb.quicksum(y[p] for p in DesTim(t,i)) <= drivers[i].cap)
        for j in range(R): #Ensure we go to origin before destination
            m.addConstr(grb.quicksum(y[p] for p in OriRTim(t,j)) - grb.quicksum(y[p] for p in DesRTim(t,j)) >= 0)
    

    for i in range(D):
        m.addConstr(grb.quicksum(y[p] for p in BDO[i]) == 1)
        m.addConstr(grb.quicksum(y[p] for p in EDD[i]) == 1)

###### P contain (x1,y1) (x2,y2) t index driver/req ori/des pt DRIVER
######% each P contain (x1, y1, x2, y2, t, index, driver/req,
######% ori/des, preferred time, which network does it belong to?
    CNT  = 0
    for i in range(len(P)):
        for j in range(len(P)):
            if P[i][7] != P[j][7]:
                CNT += 1
                m.addConstr(x[i][j] == 0)
            elif P[i][3] == P[j][3] and P[i][4] == P[j][4] and P[i][5] == P[j][5]:
                CNT += 1
                m.addConstr(x[i][j] == 0)
            elif (P[i][1] !=  P[j][0]).any():
                CNT +=1
                m.addConstr(x[i][j] == 0)
            elif P[i][2] + Distance(P[i][0],P[i][1]) > P[j][2]:
                m.addConstr(x[i][j] == 0)

##    for i in range(len(P)):     # esnure no cycle
##        m.addConstr(grb.quicksum(x[i][q] for q in range(len(P))) <= 1)
    # OBJECTIVE
    m.setObjective(grb.quicksum(c[p]*y[p] for p in range(len(P)))+LAMBDA*grb.quicksum(z[j] for j in range(R)),
                   grb.GRB.MINIMIZE)

    print(len(y))
    print(len(x))

    m.optimize()

    ENDTIME = time.clock()
    print(ENDTIME - BEGINTIME)

    for i in range(len(x)):
        for j in range(len(x[i])):
            if x[i][j].x > 0:
                print(i,j,'\t',P[i],'\t',P[j])

    for i in range(len(y)):
        if y[i].x>0:
            print(i,P[i],c[i])

    for i in range(len(z)):
        if z[i].x>0:
            print(i)

##    
##    return m,x, m.ObjVal, ENDTIME-BEGINTIME 

    return m,x,m.ObjVal,ENDTIME-BEGINTIME,P,c


##    return   m,v,x,y,z, P,c 




    
    print(len(P))
##    print(BRO,BRD,BDO,BDD,RD,RO,EDD)

if __name__ == "__main__":
##    R = 30
##    D = 3
    reqs = []
    drivers = []
    PRECISION = 30
##    ##T = 150
##    T = Distance((0,0), (0.7*PRECISION, 0.7*PRECISION))+PRECISION//2
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


    drivers = []
    reqs =[]
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

    drivers.append(Driver((2,0),(11,0),0,30,4))

    reqs.append(Passenger((0,0),(10,0),0,20,5))
    reqs.append(Passenger((1,0),(9,0),3,20,4))
    reqs.append(Passenger((3,0),(8,0),2,20,3))
    reqs.append(Passenger((4,0),(7,0),5,25,2))
    reqs.append(Passenger((5,0),(6,0),14,15,14))



    
    m,x,v,t,P,c = MILP(drivers,reqs)

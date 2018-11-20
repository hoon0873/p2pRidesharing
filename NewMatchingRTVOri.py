import numpy as np
import networkx as nx
import pylab
import time
import random as rd
import gurobipy as grb
from Classes import Driver
from Classes import Passenger
##from NewFeasibleBruth import travel
from NewFeasibleBruthWTime import travel
from itertools import combinations as COMB
from NewFeasibleBruth import Distance
import math
from Preprocessing_3 import Preprocessing
import FeasibleOneDriver as OneDMILP

LAMBDA = 1
BIGNUMBER = 5
MUTE = 0 # Set to 0 to Mute
LBPRUN = 0 # Set to 1 to do LB Pruning
GreedySWITCH = 0 # Set to 1 to do Greedy
PREPROCESSING = 0 # Set to 1 to do PREPROCESSING [Decomposition]
CONTINUOUS = 0
TimeSwitch = 0 # set to 1 to do timelimit
TimeLIMIT = 3600

# take sequence of tuples (locations) and output sum of distances
def GreedySeq(seq):
    retVal = 0
    prev = seq[0]
    for i in seq:
        retVal += Distance(prev,i)
        prev = i
    return retVal

# iSeq is sequence of request by index in rList
def Greedy(rList, iSeq, cap = math.inf):
    TIME = rList[iSeq[0]].et
    LOCATION = rList[iSeq[0]].ori
    reqCh = {0}
    cost = 0
    
    for i in iSeq[1:]:
##        print(i, reqCh,cost)
        if i not in reqCh: # We are going to the origin
            TIME = max(rList[i].et, TIME+Distance(LOCATION,rList[i].ori))
            cost += Distance(LOCATION,rList[i].ori)
            LOCATION = rList[i].ori
            if TIME > rList[i].lt: return False, math.inf
            reqCh.add(i)
        elif i in reqCh: #We are going to the destination
##            if reqCh[-1]==i: TIME = TIME +max(1,Distance(LOCATION,rList[i].des)
            TIME = TIME+Distance(LOCATION, rList[i].des)
            cost += Distance(LOCATION, rList[i].des)
            LOCATION = rList[i].des
            if TIME > rList[i].lt: return False, math.inf
            reqCh.remove(i)

        if len(reqCh) > cap: return False, math.inf # Capacity constraint
    return True, cost



def MatchingRTV(drivers,reqs, RHO =None):
    R = len(reqs)
    D = len(drivers)
    T = min(drivers[i].et for i in range(D))
    T = max(drivers[i].lt for i in range(D)) - T
    BEGINTIME = time.clock()
    COST = []
    SCHEDULE = []
    for i in range(D):
        COST.append({})
        SCHEDULE.append({})

    
    c = np.ones(R)*100 # Not Satisfied COST SHOULD change to LAMBDA
##    lambdaCost = np.ones(R)*1000 # Not satisfied Cost
    
    # RV Graph
    FeasibleMat = np.zeros((D,R), dtype=int)

    RV = nx.Graph()
    for j in range(R):
        J = reqs[j]    
        for jp in range(R):
            if j == jp: continue


            JP = reqs[jp]
##            if (min(J.lt,JP.lt) < max(J.et,JP.et)): continue #their window doesn't intersect
            
            ETIM = min(J.et, JP.et)
            LTIM = max(J.lt, JP.lt)
            J1 = J
            J2 = JP
            
            ETIM = J.et
########            if J.et < JP.et:
########                ETIM = J.et
########                J1 = J
########                J2 = JP
########            else:
########                ETIM = JP.et
########                J1 = JP
########                J2 = J

        # J1 - J2 - J2 - J1
            prevCost = float('inf')
            cost = 0
            Curtim = ETIM
            Curtim = max(Curtim+Distance(J1.ori, J2.ori), J2.et)
            cost += Distance(J1.ori, J2.ori)
            Curtim = Curtim + Distance(J2.ori, J2.des)
            cost += Distance(J2.ori, J2.des)
            if Curtim <= J2.lt:
                Curtim = Curtim + Distance(J2.des,J1.des)
                cost += Distance(J2.des, J1.des)
                if Curtim <= J1.lt:
                    prevCost = cost
                    RV.add_edge(j,jp,weight=prevCost)
    ##                print("HERE")
                    continue
                
    ##        print(Curtim, J1.lt)
        # J1 - J2 - J1 - J2
            cost = 0
            Curtim = ETIM
            Curtim = max(ETIM+Distance(J1.ori, J2.ori), J2.et)
            cost = Distance(J1.ori, J2.ori)
            Curtim = Curtim + Distance(J2.ori, J1.des)
            cost += Distance(J2.ori, J1.des)
            if Curtim <= J1.lt:
                Curtim = Curtim + Distance(J1.des, J2.des)
                cost += Distance(J1.des, J2.des)
                if Curtim <= J2.lt and cost <= prevCost:
                    RV.add_edge(j,jp,weight=cost)
    ##                print("HERE")
                    continue
    ##        print(Curtim, J2.lt)
        # J1 - J1 - J2 - J2
            cost = 0
            Curtim = ETIM
            Curtim = ETIM+Distance(J1.ori, J1.des)
            cost = Distance(J1.ori, J1.des)
            if Curtim <= J1.lt:
                Curtim = max(Curtim + Distance(J1.des, J2.ori),J2.et)
                cost += Distance(J1.des, J2.ori)
                Curtim = Curtim + Distance(J2.ori, J2.des)
                cost += Distance(J2.ori, J2.des)
                if Curtim <= J2.lt and cost <= prevCost:
                    RV.add_edge(j,jp,weight=cost)
    ##                print("HERE")
                    continue
    ##        print(Curtim, J2.lt)

                
                
##            VirtualDriver = Driver(J.ori, J.des, ETIM, LTIM, 2)
##            bol, cost, pvRT = travel(VirtualDriver, [J,JP])
##            if bol == True:
##                RV.add_edge(j,jp,weight=cost)
##            else:
##                VirtualDriver = Driver(J.ori, JP.des, ETIM, LTIM, 2)
##                bol, cost, pvRT  = travel(VirtualDriver, [J,JP])
##                if bol == True:
##                    RV.add_edge(j,jp,weight=cost)
##    
##
##            VirtualDriver = Driver(JP.ori, JP.des, ETIM, LTIM, 2)        
##            bol, cost, pvRT  = travel(VirtualDriver, [J,JP])
##            if bol == True:
##                RV.add_edge(j,jp,weight=cost)
##            else:
##                VirtualDriver = Driver(JP.ori, J.des, ETIM, LTIM, 2)
##                bol, cost, pvRT  = travel(VirtualDriver, [J,JP])
##                if bol == True:
##                    RV.add_edge(j,jp,weight=cost)

        for i in range(D):
            if J.lt < drivers[i].et or J.et > drivers[i].lt: continue
            bol,cost,route = travel(drivers[i],[J],RHO=RHO)
            if bol == True:
                FeasibleMat[i][j] = 1
                RV.add_edge('d'+str(i), j, weight=cost)
                COST[i][tuple({j})] = cost                
                for schInd in range(len(route)):
                    if route[schInd][0] == 'd':
                        continue
                    route[schInd] = (j,route[schInd][1])
                SCHEDULE[i][tuple({j})] = route
##            cost = 0
##            Curtim = drivers[i].et
##            Curtim = max(Curtim+Distance(drivers[i].ori, J.ori), J.et)
##            cost += Distance(drivers[i].ori, J.ori)
##            Curtim = Curtim + Distance(J.ori,J.des)
##            cost += Distance(J.ori,J.des)
##            if Curtim > J.lt: continue
##            Curtim = Curtim + Distance(J.des,drivers[i].des)
##            cost += Distance(J.des,drivers[i].des)
##            if Curtim > drivers[i].lt: continue
##            
##            else:
##                FeasibleMat[i][j] = 1
##                RV.add_edge('d'+str(i), j, weight=cost)
##                COST[i][tuple({j})] = cost
                


    if MUTE != 0: print(FeasibleMat)

##    print(FeasibleMat)

############ RUN WEILONG"S ALGORITHM FOR DECOMPOSING SINGLE DRIVER ###################
##
    OneDObjVal = 0
##
##    if PREPROCESSING != 0:
##        newMat, SingleD, delD, delR = Preprocessing(FeasibleMat)
##
##        if MUTE != 0:
##            print(newMat)
##            print(SingleD)
##            print(delD)
##            print(delR)
##
##        OneDObjVal = 0
##        for singD in SingleD:
##            bol,dval,dx,dy,dz,dm = OneDMILP.travel(drivers[singD[0]], [reqs[i] for i in singD[1]],RHO=RHO)
##                
##            OneDObjVal += dval
##
##        if MUTE != 0: print(OneDObjVal)
##
##        print("Decomposed Objective Value: " + str(OneDObjVal))
##        if MUTE != 0: print("Decomposition Time: %f" %(time.clock()-BEGINTIME))
##        
##        for i in delD:
##            RV.remove_node('d'+str(i))
##        for j in delR:
##            c[j] = 0
##            RV.remove_node(j)
##        
    
######################################################################################

    # RTV Graph
    ##Trip = []
##    print(RV.edges())
    RTV = nx.DiGraph()
    
    for i in range(D):
        feaReq = set()
        if MUTE != 0: print("Driver %d starting Time: %f" %(i, time.clock()-BEGINTIME))
        Trip = []
        for UNNECESSARY in range(2):
            Trip.append(set())

        DRIVER = 'd'+str(i)

        if MUTE != 0: print( time.clock()-BEGINTIME)

        for r in range(R):
            RTV.add_node(r)
        RTV.add_node(DRIVER)

        for d,r in RV.edges(DRIVER):
            Tr = {r}
            feaReq.add(r)
            Tr = tuple(Tr)
            Trip[0].add(Tr)
            RTV.add_edge(r,Tr)
            RTV.add_edge(Tr,DRIVER)


        
        if LBPRUN != 0: TripCost = {}
        for T1 in Trip[0]:
            for T2 in Trip[0]:
                r1 = T1[0]
                r2 = T2[0]

                if r1 == r2: continue
                if (r1,r2) not in RV.edges(): continue

                if (r1,r2) in Trip[1]: continue
                if (r2,r1) in Trip[1]: continue
                reqList = [reqs[r1],reqs[r2]]
                reqIndex = [r1,r2]
                bol, cost, route = travel(drivers[i], reqList,RHO=RHO)
                totalBol = bol
                pvCost = cost


################# INITIAL GREEDY  TRIP OF SIZE 1 ##############################################################                  
##                totalBol = False
##                pvCost = float("inf")
##                pvRt = None
##                bol, cost = Greedy([drivers[i]]+reqList, [0,1,1,2,2,0], cap = drivers[i].cap)
##                if bol == True and cost < pvCost:
##                    pvCost = cost
##                    totalBol = True
##                    pvRt = [0,1,1,2,2,0]
####                    print("PVCOST 1:  " + str(pvCost))
##                bol, cost = Greedy([drivers[i]]+reqList, [0,1,2,1,2,0], cap = drivers[i].cap)
##                if bol == True and cost < pvCost:
##                    pvCost = cost
##                    totalBol = True
##                    pvRt = [0,1,2,1,2,0]
####                    print("PVCOST 2:  " + str(pvCost))
##                bol, cost = Greedy([drivers[i]]+reqList, [0,1,2,2,1,0], cap = drivers[i].cap)
##                if bol == True and cost < pvCost:
##                    pvCost = cost
##                    totalBol = True
##                    pvRt = [0,1,2,2,1,0]
####                    print("PVCOST 3:  " + str(pvCost))
##                bol, cost = Greedy([drivers[i]]+reqList, [0,2,2,1,1,0], cap = drivers[i].cap)
##                if bol == True and cost < pvCost:
##                    pvCost = cost
##                    totalBol = True
##                    pvRt = [0,2,2,1,1,0]
####                    print("PVCOST 4:  " + str(pvCost))
##                bol, cost = Greedy([drivers[i]]+reqList, [0,2,1,2,1,0], cap = drivers[i].cap)
##                if bol == True and cost < pvCost:
##                    pvCost = cost
##                    totalBol = True
##                    pvRt = [0,2,1,2,1,0]
####                    print("PVCOST 5:  " + str(pvCost))
##                bol, cost = Greedy([drivers[i]]+reqList, [0,2,1,1,2,0], cap = drivers[i].cap)
##                if bol == True and cost < pvCost:
##                    pvCost = cost
##                    totalBol = True
##                    pvRt = [0,2,1,1,2,0]
####                    print("PVCOST 6:  " + str(pvCost))
####            
####                print("PVCOST:  " + str(pvCost))
                
                if totalBol == True:
                    Tr = tuple(sorted((r1,r2)))
                    Trip[1].add(Tr)
                    RTV.add_edge(r1,Tr)
                    RTV.add_edge(r2,Tr)
                    RTV.add_edge(Tr,DRIVER)
                    COST[i][Tr] = pvCost
                    for schInd in range(len(route)):
                        if route[schInd][0] == 'd':
                            continue
                        route[schInd] = (reqIndex[route[schInd][0]],route[schInd][1])
                    SCHEDULE[i][Tr] = route
##                    if LBPRUN!=0:
##                        pvRt = [i-1 if i!=0 else 'd' for i in pvRt]
##                        TripCost[Tr] = (pvCost,pvRt)

                    
        if MUTE != 0: print("Finishing pair %f" %(time.clock()-BEGINTIME))



#################### TRIP OF SIZE 2 to R ############################################

        
        for TSize in range(2,R):

            if TimeSwitch != 0 and time.clock()-BEGINTIME > TimeLIMIT:
                break
            
            Trip.append(set())
            if LBPRUN  != 0:
                PrevTripCost= TripCost
                TripCost = {}
            
            if MUTE != 0: print("Tsize: %d SIZE OF TRIPS %d;\t%f" %(TSize, len(Trip[TSize-1]), time.clock()-BEGINTIME))
            if len(Trip[TSize-1]) == 0: break
            COUNT = 0
            for T1 in Trip[TSize-1]:
                if TimeSwitch != 0 and time.clock()-BEGINTIME > TimeLIMIT:
                    break
##                for T2 in Trip[TSize-1]:
##                    TUT = set(T1).union(set(T2))
                for req in feaReq:
                    TUT = set(T1).union({req})
                    if len(TUT) != TSize+1: continue
                    if tuple(sorted(TUT)) in Trip[TSize]: continue
                    reqList = sorted(list(TUT))


                    SKIP = 0
                    for rr in T1:
                        TUT.remove(rr)
                        if tuple(sorted(TUT)) not in Trip[TSize-1]:
                            TUT.add(rr)
                            SKIP = -1
                            break
                        TUT.add(rr)
                    if SKIP == -1:
                        SKIP = 0
                        continue
                    if SKIP == 0:
                        reqList = sorted(list(TUT))                        

                        COUNT+=1

                        reqL = [reqs[JS] for JS in reqList]
                    
                        if LBPRUN != 0:
                            TUT = tuple(sorted(TUT))
                            if T1 in PrevTripCost:
                                LB,ROUTE = PrevTripCost[T1]
                            else:
                                LB = 0
                                ROUTE = None

                            grdCost = None
                            if ROUTE != None and GreedySWITCH != 0:
                                ROUTE = [i if i!='d' else len(ROUTE)//2 for i in ROUTE]
                                pvCost = float('inf')
                                pvRt = None
                                totalBol = False
                                reqL1 = [reqs[HRK] for HRK in T1]
                                for rtI in range(len(ROUTE)+1):
                                    for rtJ in range(rtI,len(ROUTE)+1):
                                        curROUTE = ROUTE[:rtI]+[len(reqL1)]+ROUTE[rtI:rtJ]+[len(reqL1)]+ROUTE[rtJ:]
    ##                                    print(ROUTE, curROUTE)
                                        bol,cost = Greedy(reqL1 + [reqs[req]] + [drivers[i]], curROUTE, cap=drivers[i].cap)
                                        if bol == True and cost < pvCost:
                                            pvCost = cost
                                            totalBol = True
                                            pvRt = curROUTE
                                grdCost = pvCost
                            
                            bol,cost,route = travel(drivers[i], reqL, LB = LB, UB = grdCost,RHO=RHO)

                            if MUTE == 3: print('COST:\t', cost)
                            
                        else:
                            if TSize >= 5 and MUTE !=0: print(reqList)
                            bol, cost, route = travel(drivers[i], reqL,RHO=RHO)

                        


                        if bol == True:
                            Tr = tuple(sorted(TUT))
                            Trip[TSize].add(Tr)
                            for r in Tr:
                                RTV.add_edge(r,Tr)
                            RTV.add_edge(Tr,DRIVER)
                            COST[i][Tr] = cost
                            for schInd in range(len(route)):
                                if route[schInd][0] == 'd':
                                    continue
                                route[schInd] = (reqList[route[schInd][0]],route[schInd][1])
                            SCHEDULE[i][Tr] = route
                            if LBPRUN != 0: TripCost[TUT] = (cost,route)
            if MUTE != 0: print('COUNT:\t',COUNT)

    if MUTE != 0: print("FINISH PREPROCESSING OPTIMIZING %f" %(time.clock()-BEGINTIME))

    ## ADD EMPTY SET
    for i in range(D):
        dIndex = 'd'+str(i)
        RTV.add_edge('EMPTY'+str(i), dIndex)
        COST[i]['EMPTY'+str(i)] = drivers[i].cdet*Distance(drivers[i].ori, drivers[i].des)
        SCHEDULE[i]['EMPTY'+str(i)] = [('d',drivers[i].pt),('d',drivers[i].pt+Distance(drivers[i].ori, drivers[i].des))]
##        COST[i]['EMPTY'+str(i)] = 0 # By convension we only care about detour and deviation cost

        
                                          
    #### START THE MATCHING MILP ####
    m = grb.Model("MATCHING")
    x = []
    for i in range(D):
        x.append({})

    # SET variables for each edge in RTV graph
    for a,b in RTV.edges():
        if type(a) != type('a') and type(b) != type('b'): continue
        elif b[0] == 'd':
            v = int(b[1:])
            if CONTINUOUS == 0: x[v][a] = m.addVar(vtype=grb.GRB.BINARY)
            else: x[v][a] = m.addVar(vtype = 'c', lb= 0, ub = 1)

    # SET variable y for each request
    if CONTINUOUS == 0: y = m.addVars(R, vtype = grb.GRB.BINARY)
    else: y = m.addVars(R, vtype = 'c', lb=0, ub=1)

    # constraint: at most one trip is matched for each driver
    for v in range(D):
        m.addConstr(grb.quicksum(x[v][b] for b in RTV.predecessors('d'+str(v)))
                    == 1)

    # exactly one variable (either trip or y) for each request is satisfeid
    for j in range(R):
        m.addConstr(grb.quicksum(x[int(v[1:])][trip]
                                 for trip in RTV.neighbors(j)
                                 for v in RTV.neighbors(tuple(trip)))
                    + y[j] == 1)

    # OBJECTIVE FUNCTION
    m.setObjective(grb.quicksum(COST[d][S]*x[d][S]
                            for d in range(D)
                            for S in RTV.predecessors('d'+str(d)))
               + grb.quicksum(reqs[j].lamb*y[j]
                              for j in range(R)),
               grb.GRB.MINIMIZE)
                             
####    # OBJECTIVE FUNCTION
####    m.setObjective(grb.quicksum(COST[d][S]*x[d][S]
####                            for d in range(D)
####                            for S in RTV.predecessors('d'+str(d)))
####               + grb.quicksum(lambdaCost[j]*y[j]
####                              for j in range(R)),
####               grb.GRB.MINIMIZE)

    if MUTE != 0: print("START MATCHING %f" %(time.clock()-BEGINTIME))
    MILPSTARTTIME = time.clock()
    m.setParam('OutputFlag', MUTE)
    m.optimize()

    ENDTIME = time.clock()
    if MUTE != 0: print("Matching Time %f" %(ENDTIME - MILPSTARTTIME))
    print("RTV3\t Total Time %f" %(ENDTIME - BEGINTIME))


    AUD = np.zeros(D) # altruistic Utility
    BUD = np.zeros(D) # Base utility
    UR = np.zeros(R)
    UDVSIT = -np.ones(D)
    URVSIT = -np.ones(R)
    for i in range(D):
        for s in x[i]:
            if x[i][s].x > 0:
                print(i,s,COST[i][s],x[i][s].x)
                print('  ', SCHEDULE[i][s])
                for (agnt, ti) in SCHEDULE[i][s]:
                    if agnt == 'd' and UDVSIT[i] ==-1:
                        AUD[i] = drivers[i].val
                        BUD[i] = drivers[i].val
                        AUD[i] -= drivers[i].cdev*abs(ti-drivers[i].pt)
                        BUD[i] -= drivers[i].cdev*abs(ti-drivers[i].pt)
                        UDVSIT[i] = ti
                    elif agnt == 'd':
                        AUD[i] -= drivers[i].cdet*(ti-UDVSIT[i])
                        BUD[i] -= drivers[i].cdet*(ti-UDVSIT[i])
                    elif URVSIT[agnt] == -1:
                        UR[agnt] = reqs[agnt].val
                        UR[agnt] -= reqs[agnt].cdev*abs(ti-reqs[agnt].pt)
                        URVSIT[agnt] = ti
                    else:
                        UR[agnt] -= reqs[agnt].cdet*(ti-URVSIT[agnt])
                        AUD[i] += drivers[i].rho*UR[agnt]

    objSW = 0
    objEff = 0

    for i in range(D):
        objSW += AUD[i]
        objEff += BUD[i]
        print('Utility of ', i,': ',UD[i])
    for j in range(R):
        objSW += UR[j]
        objEff += UR[j]
        print('Utility of ',j,': ',UR[j])




    
    return m,x,OneDObjVal+m.ObjVal,ENDTIME-BEGINTIME
##    return m,x,OneDObjVal+m.ObjVal,COST
##    return m,x,m.ObjVal,ENDTIME-BEGINTIME

if __name__ == "__main__":

    R = 30
    D = 15
    reqs = []
    drivers = []
    PRECISION = 30
    ##T = 150
    T = Distance((0,0), (0.7*PRECISION, 0.7*PRECISION))+PRECISION//2

    
    BEGINTIME = time.clock()
    print("DRIVERS")
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

    print("REQUESTS")                   
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



##    reqs = []
##    drivers = []
##
##    BIGNUMBER = 5


##    print("DRIVERS")
##    drivers.append(Driver((0,0),(7,0),0,20,4))
##    drivers.append(Driver((30,0),(17,0),20,40,4))
####    drivers.append(Driver((0, 1),(15, 20), 0 ,59 ,4))
####    drivers.append(Driver((26, 29) ,(20, 15) ,0 ,59 ,4))
##
##    print("REQUESTS")
##    reqs.append(Passenger((1,0),(6,0),0,20,1))
####    reqs.append(Passenger((26,0),(18,0),14,30,16))
##    reqs.append(Passenger((2,0),(5,0),0,20,2))
##    reqs.append(Passenger((3,0),(4,0),0,41,3))
####    reqs.append(Passenger((27,0),(20,0),2,39,10))
####    reqs.append(Passenger((3,0),(17,0),4,29,5))
##    reqs.append(Passenger((28,0),(19,0),13,35,24))
##    reqs.append(Passenger((2,0),(19,0),2,50,14))
##    reqs.append(Passenger((26,0),(14,0),5,28,7))
    
    
                
##    reqs.append(Passenger((2, 5), (14, 12) ,4 ,27 ,9))
##    reqs.append(Passenger((25, 24), (15, 19) ,10 ,38 ,18))
##    reqs.append(Passenger((5, 2), (20, 15) ,6 ,35 ,11))
##    reqs.append(Passenger((25, 29) ,(12, 20) ,4 ,22 ,5))
##    reqs.append(Passenger((5, 5), (16, 20), 7 ,51 ,20))
##    reqs.append(Passenger((26, 27) ,(14, 13) ,2 ,41 ,12))
##    reqs.append(Passenger((4, 4), (15, 18) ,2 ,34 ,9))
##    reqs.append(Passenger((25, 27), (15, 19) ,3 ,29 ,10))
##    reqs.append(Passenger((3, 1), (13, 18) ,4 ,48 ,16))
##    reqs.append(Passenger((25, 27) ,(16, 14) ,4 ,42 ,15))
##    reqs.append(Passenger((4, 2), (14, 19) ,6 ,32 ,9))
##    reqs.append(Passenger((29, 25), (12, 17) ,8 ,52 ,21))
##
##
##    drivers = []
##    reqs = []
##
##    
####    drivers.append(Driver((2, 1),(24, 25),0,58,4))
##    drivers.append(Driver((3, 4),(28, 25),0,58,4))
####    drivers.append(Driver((2, 2),(27, 27),0,58,4))
##    reqs.append(Passenger((0, 3),(8, 7),14,39,22))
##    reqs.append(Passenger((9, 9),(13, 14),18,44,27))
##    reqs.append(Passenger((13, 12),(17, 16),6,30,15))
##    reqs.append(Passenger((18, 15),(20, 22),10,37,19))
##    reqs.append(Passenger((20, 22),(29, 29),13,54,27))
##    reqs.append(Passenger((4, 3),(6, 5),15,26,19))
##    reqs.append(Passenger((8, 7),(11, 14),2,30,12))
##    reqs.append(Passenger((13, 11),(17, 18),16,30,18))
##    reqs.append(Passenger((19, 17),(21, 23),10,33,18))
##    reqs.append(Passenger((22, 20),(28, 25),12,24,14))
##    reqs.append(Passenger((3, 4),(6, 5),18,32,23))
##    reqs.append(Passenger((6, 7),(11, 13),4,36,16))
##    reqs.append(Passenger((12, 13),(18, 18),4,35,15))
##    reqs.append(Passenger((16, 17),(21, 23),7,36,17))
##    reqs.append(Passenger((21, 20),(25, 25),9,29,15))
##    reqs.append(Passenger((1, 3),(5, 5),20,33,24))
##    reqs.append(Passenger((7, 8),(14, 13),5,41,18))
##    reqs.append(Passenger((12, 13),(16, 19),18,26,18))
##    reqs.append(Passenger((16, 19),(23, 24),4,58,11))
##    reqs.append(Passenger((23, 20),(25, 27),3,58,8))
##    reqs.append(Passenger((1, 2),(6, 5),0,35,14))
##
##
##
############# HARDER INPUT ###########################
##    drivers = []
##    reqs = []
##
####    drivers.append(Driver((3, 2),(29, 26),0,58,4))
##    drivers.append(Driver((5, 2),(24, 24),0,58,4))
####    drivers.append(Driver((4, 1),(29, 27),0,58,4))
##    reqs.append(Passenger((2, 3),(8, 9),6,40,18))
##    reqs.append(Passenger((5, 7),(10, 13),9,40,20))
##    reqs.append(Passenger((10, 10),(15, 19),10,28,13))
##    reqs.append(Passenger((18, 16),(21, 24),14,45,25))
##    reqs.append(Passenger((20, 24),(28, 27),10,42,21))
##    reqs.append(Passenger((2, 4),(6, 6),7,41,21))
##    reqs.append(Passenger((7, 9),(14, 12),12,32,18))
##    reqs.append(Passenger((10, 12),(18, 15),15,31,18))
##    reqs.append(Passenger((19, 17),(20, 21),1,21,8))
##    reqs.append(Passenger((22, 23),(25, 28),2,14,5))
##    reqs.append(Passenger((1, 2),(8, 8),12,39,20))
##    reqs.append(Passenger((9, 8),(13, 10),6,39,20))
##    reqs.append(Passenger((12, 10),(17, 16),15,43,25))
##    reqs.append(Passenger((15, 15),(24, 22),2,15,2))
##    reqs.append(Passenger((23, 23),(28, 27),6,29,14))
##    reqs.append(Passenger((3, 3),(6, 8),6,36,18))
##    reqs.append(Passenger((5, 8),(11, 10),19,36,24))
##    reqs.append(Passenger((12, 11),(18, 19),5,33,14))
##    reqs.append(Passenger((18, 18),(21, 23),13,32,19))
##    reqs.append(Passenger((20, 22),(27, 29),5,42,18))
##






##    m,x,val,runtime = nrtv.MatchingRTV(drivers,reqs)
##
##    m1,x1,val1,runtime1 = rtv.MatchingRTV(drivers,reqs)
##    reqList = [reqs[0],reqs[2]]
##    bol, cost = Greedy([drivers[0]]+reqList, [0,1,2,1,2,0], cap = drivers[0].cap)
##    print(cost)
##    print(bol)
    
    
    m,x,v,t = MatchingRTV(drivers,reqs)
    print(v)

    

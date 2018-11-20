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
from Feasible import travel as travelMILP
from itertools import combinations as COMB
from NewFeasibleBruth import Distance
import math
from Preprocessing_3 import Preprocessing
import FeasibleOneDriver as OneDMILP

LAMBDA = 1
BIGNUMBER = 5
MUTE = 0 # Set to 0 to Mute
LBPRUN = 1 # Set to 1 to do LB Pruning
RVPRUN = 1 # Set to 1 to do RV Pruning
GreedySWITCH = 0 # Set to 1 to do Greedy
PREPROCESSING = 0 # Set to 1 to do PREPROCESSING [Decomposition]
SIMDPRUN = 1 # set to 1 to do similar Drivers pruning
CONTINUOUS = 0
TimeSwitch = 1 # set to 1 to do timelimit
TimeLIMIT = 60*60*2


TIMEMUTE = 1 # Set 0 to mute; 1 to see high level timeline; 2 to detailed timeline; 


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
##            if reqCh[-1]==i: TIME = TIME +ax(1,Distance(LOCATION,rList[i].des)
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

    if TIMEMUTE >=1: print("\nSTARTING MATCHINGRTV %f\n" %(time.clock()-BEGINTIME))

    # given two schedules, create more likely schedule
    def computeUBroutes(RT1, RT2, r1, r2, MUTEWITHINFUNCTION = 0):
        if  MUTEWITHINFUNCTION ==1 :
            print('RT1 = ',RT1)
            print('RT2 = ', RT2)
            print('r1 = ',r1,'\nr2 = ',r2)
            
        THRESHOLD = 10
        UBRS = [[]]
        i1= 0
        i2 = 0
        i = 0
        visited = np.zeros(R+2)
        while(i < (len(RT1))):
            if MUTEWITHINFUNCTION == 1:
                print("\n")
                print(UBRS[0])
                print(i,i1,i2)
                print(RT1[i1][0], RT2[i2][0])
####            print(RT1[i1+1][0], RT2[i2+1][0])
####            print(RT1[i1][0],RT2[i2][0])
####            print(UBRS)
            
            if i == 0:
                for UBR in UBRS:
                    UBR.append('d')
                i1+=1
                i2+=1
                i+=1
                continue
            elif RT1[i1][0] == 'd':
                while(RT2[i2][0] != 'd'):
                    for UBR in UBRS:
                        if visited[RT2[i2][0]]<=1:
                            UBR.append(RT2[i2][0])
                    i2+=1
                if RT2[i2][0] == 'd':
                    for UBR in UBRS:                     
                            UBR.append(RT1[i1][0])
                    i2+=1
                    i1+=1
                    break
            elif RT2[i2][0] == 'd':
                while(RT1[i1][0] != 'd'):
                    for UBR in UBRS:
                        if visited[RT1[i1][0]] <=1:   
                            UBR.append(RT1[i1][0])
                    i1+=1
                if RT1[i1][0] == 'd':
                    for UBR in UBRS:
                        UBR.append(RT1[i1][0])
                    i1+=1
                    i2+=1
                    break               
            while(visited[RT1[i1][0]]) > 1:
                if MUTEWITHINFUNCTION == 1:print('IN THE VISITED -1')
                i1+=1
                if RT1[i1][0] == 'd': break
            while(visited[RT2[i2][0]]) > 1:
                if MUTEWITHINFUNCTION == 1:print('IN THE VISITED -2')
                i2+=1
                if RT2[i2][0] == 'd': break
            if RT1[i1][0] == 'd' or RT2[i2][0] =='d': continue
            if RT1[i1][0] == RT2[i2][0]:
                if MUTEWITHINFUNCTION == 1:print('EQUAL')
                for UBR in UBRS:
                    UBR.append(RT1[i1][0])
                visited[RT1[i1][0]]+=1
                i1+=1
                i2+=1
                continue
            elif RT1[i1][0] == RT2[i2+1][0] and RT1[i1+1][0] == RT2[i2][0]:
                if MUTEWITHINFUNCTION == 1:print('DIAG EQUAL')
                if RT1[i1][1] <= RT2[i2][1]:
                    for UBR in UBRS:
                        UBR.append(RT1[i1][0])
                        UBR.append(RT2[i2][0])
                    visited[RT1[i1][0]]+=1
                    visited[RT2[i2][0]]+=1                
                    i1+=2
                    i2+=2
                else:
                    for UBR in UBRS:
                        UBR.append(RT2[i2][0])
                        UBR.append(RT1[i1][0])
                    visited[RT1[i1][0]]+=1
                    visited[RT2[i2][0]]+=1 
                    i1+=2
                    i2+=2
                continue
            elif RT1[i1][0] == RT2[i2+1][0] and RT2[i2][0] == r1:
                if MUTEWITHINFUNCTION == 1:print('-1')
                for UBR in UBRS:
                    UBR.append(RT2[i2][0])
                    UBR.append(RT1[i1][0])
                visited[RT1[i1][0]]+=1
                visited[RT2[i2][0]]+=1 
                i1+=1
                i2+=2
                continue
            elif RT1[i1+1][0] == RT2[i2][0] and RT1[i1][0] == r2:
                if MUTEWITHINFUNCTION == 1:print('-2')
                for UBR in UBRS:
                    UBR.append(RT1[i1][0])
                    UBR.append(RT2[i2][0])
                visited[RT1[i1][0]]+=1
                visited[RT2[i2][0]]+=1 
                i1+=2
                i2+=1
                continue
            elif RT1[i1][0] == RT2[i2+1][0]:
                if MUTEWITHINFUNCTION == 1: print('-3')
                for UBR in UBRS:
                    UBR.append(RT2[i2][0])
                    UBR.append(RT1[i1][0])
                visited[RT1[i1][0]]+=1
                visited[RT2[i2][0]]+=1 
                i1+=1
                i2+=2
                continue
            elif RT1[i1+1][0] == RT2[i2][0]:
                if MUTEWITHINFUNCTION == 1: print('-4')
                for UBR in UBRS:
                    UBR.append(RT1[i1][0])
                    UBR.append(RT2[i2][0])
                visited[RT1[i1][0]]+=1
                visited[RT2[i2][0]]+=1 
                i1+=2
                i2+=1
                continue
            elif len(UBRS) <= THRESHOLD:
                UBR2S = []
                for UBR in UBRS:
                    UBR2 = UBR.copy()
                    if visited[RT1[i1][0]] <= 1:
                        UBR.append(RT1[i1][0])
                    if visited[RT2[i2][0]] <= 1:
                        UBR.append(RT2[i2][0])
                        UBR2.append(RT2[i2][0])
                    if visited[RT1[i1][0]] <= 1:
                        UBR2.append(RT1[i1][0])
                    UBR2S.append(UBR2)
                UBRS = UBRS+UBR2S
                visited[RT1[i1][0]]+=1
                visited[RT2[i2][0]]+=1
                i1+=1
                i2+=1
            else:
                if MUTEWITHINFUNCTION == 1: print("HERE AT THE END")
                if RT1[i1][1] <= RT2[i2][1]:
                    for UBR in UBRS:
                        UBR.append(RT1[i1][0])
                        UBR.append(RT2[i2][0])
                    visited[RT1[i1][0]]+=1
                    visited[RT2[i2][0]]+=1                
                    i1+=1
                    i2+=1
                else:
                    for UBR in UBRS:
                        UBR.append(RT2[i2][0])
                        UBR.append(RT1[i1][0])
                    visited[RT1[i1][0]]+=1
                    visited[RT2[i2][0]]+=1 
                    i1+=1
                    i2+=1
                continue
                
            i = min(i1,i2)
        for UBR in UBRS:
            if len(UBR) != len(RT1)+2:
                print("\n\nERROR\n")
                print(RT1,RT2,r1,r2)
                print(UBR)
                print("\n\n")
                time.sleep(2)
        return UBRS
        


##    RT1 = [('d', 0), (0, 4), (3, 8), (1, 9), (3, 24), (1, 27), (0, 30), ('d', 35)] 
##    RT2 = [('d', 3), (0, 7), (1, 10), (2, 12), (2, 28), (0, 29), (1, 32), ('d', 39)]
##    r1 = 2
##    r2 = 3
##
##    UBRS = computeUBroutes(RT1,RT2,r1,r2,1)
##    print(UBRS[0])

 
##['d', 0, 1, 2, 2, 3, 1, 0, 2, 3, 'd']

    
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
                
        for i in range(D):
            d = drivers[i]
            if RVPRUN == 1:
                if J.lt < drivers[i].et or J.et > drivers[i].lt: continue
                cost = 0
                Curtim = d.et
                Curtim = max(Curtim+Distance(d.ori,J.ori),J.et)
                Curtim = Curtim + Distance(J.ori,J.des)
                if Curtim >= J.lt: continue
                if Curtim+Distance(J.des,d.des) >= d.lt: continue

                # v_d + rho_d*(v_r - mincost_r) - mincost_d -minDevCost_{dr}< v_d - d_cdet*dist_d
                if d.cdet*(Distance(d.ori,d.des))+d.rho*(J.val-J.cdet*Distance(J.ori,J.des))\
                   -d.cdet*(Distance(J.ori,J.des))\
                   -d.cdet*(Distance(d.ori,J.ori)+Distance(J.des,d.des))\
                   -min(d.cdev,J.cdev*d.rho)*abs(J.pt-d.pt-Distance(d.ori,J.ori))< -1e-5: #driver cannot be IR
                    continue
                
            if TIMEMUTE >=2: print('   ---- before travel', time.clock()-BEGINTIME)
            bol,cost,route = travel(drivers[i],[J],RHO=RHO)
            if TIMEMUTE >=2: print('\t',bol,cost,route,i,j)
            if TIMEMUTE >=2: print('   ---- after travel', time.clock()-BEGINTIME)
##            if TIMEMUTE !=0: print('\tbefore travel MILP', time.clock()-BEGINTIME)
##            bol2,cost2,route2 = travelMILP(drivers[i],[J])
##            if TIMEMUTE !=0: print('\tafter travel MILP', time.clock()-BEGINTIME)
##            print('\t',bol,bol2,cost,cost2)
            if bol == True:
                FeasibleMat[i][j] = 1
                RV.add_edge('d'+str(i), j, weight=cost)
                COST[i][tuple({j})] = cost                
                for schInd in range(len(route)):
                    if route[schInd][0] == 'd':
                        continue
                    route[schInd] = (j,route[schInd][1])
                SCHEDULE[i][tuple({j})] = route
##                print(i,j)
##                print(cost)
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
    if TIMEMUTE >=1: print("Finished RV Time: %f" %(time.clock()-BEGINTIME))
##    print(FeasibleMat)

######################################################333#############################
    ### ORDER MATTER ###
    DEPSILON = 5
    SimilarD = {}
    for d1 in range(D):
        for d2 in range(d1):
            if d1==d2: continue
            D1 = drivers[d1]
            D2 = drivers[d2]

            if Distance(D1.ori,D2.ori) <= DEPSILON and Distance(D1.des,D2.des) <=DEPSILON:
                if d2 in SimilarD:
                    SimilarD[d2].add(d1)
                else:
                    SimilarD[d2] = set([d1])
    print(SimilarD)
                


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
    Trips = []
    RTV = nx.DiGraph()
    for i in range(D):
        Trips.append([])
        COUNT=0
        feaReq = set()
        if TIMEMUTE >=1: print("\n\nRTV: Driver %d starting Time: %f" %(i, time.clock()-BEGINTIME))
        Trip = Trips[i]
        for UNNECESSARY in range(2):
            Trip.append(set())

        DRIVER = 'd'+str(i)
        if TIMEMUTE >=1: print(" Number of feasibe requests: %d" %(len(RV.edges(DRIVER))))

##        if TIMEMUTE >=1: print('StartRTV: ', time.clock()-BEGINTIME)

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


        if TIMEMUTE >=1: print("  Now computeing Tsize 2:")
        if LBPRUN != 0: TripCost = {}
        for T1 in Trip[0]:
            for T2 in Trip[0]:
##                print("ERERERER")
                r1 = T1[0]
                r2 = T2[0]

                if r1 == r2: continue
                if (r1,r2) not in RV.edges(): continue

                if (r1,r2) in Trip[1]: continue
                if (r2,r1) in Trip[1]: continue
                reqList = [reqs[r1],reqs[r2]]
                reqIndex = [r1,r2]
                INFEASIBLEUB = min(COST[i][tuple({r1})]+reqs[r2].lamb,COST[i][tuple({r2})]+reqs[r1].lamb)
##                print(reqList)
                if TIMEMUTE >=2: print('\tbefore travel', time.clock()-BEGINTIME)
                COUNT+=1
                bol, cost, route = travel(drivers[i], reqList,RHO=RHO,INFEASIBLEUB=INFEASIBLEUB)
                if TIMEMUTE >=2: print('\tafter travel', time.clock()-BEGINTIME)
##                if TIMEMUTE >=2: print('\tbefore travel MILP', time.clock()-BEGINTIME)
##                bol2,cost2,route2 = travelMILP(drivers[i],reqList)
##                if TIMEMUTE >=2: print('\tafter travel MILP', time.clock()-BEGINTIME)
##                print('\t',bol,bol2,cost,cost2)
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
                    if LBPRUN != 0: TripCost[Tr] = (pvCost,route)
                    
######                    print(i,Tr)
######                    print(pvCost)
######                    if LBPRUN!=0:
######                        pvRt = [i-1 if i!=0 else 'd' for i in pvRt]
######                        TripCost[Tr] = (pvCost,pvRt)

                    
        if TIMEMUTE >=1: print("  Finished pair (2) \t\t%f" %(time.clock()-BEGINTIME))



#################### TRIP OF SIZE 2 to R ############################################
    
        for TSize in range(2,R):
            INFEASIBELTRIP = set()
            if TimeSwitch != 0 and time.clock()-BEGINTIME > TimeLIMIT:
                break
            
            Trip.append(set())
            if LBPRUN  != 0:
                PrevTripCost= TripCost
                if TIMEMUTE >=2: print("    PREV TRIP LENGTH", len(PrevTripCost)) # Dict of (SET : cost, Schedule)
                TripCost = {}
            if TIMEMUTE >=1: print("  There %d cadidate trips" %(COUNT))
            if TIMEMUTE >=1: print("  There are %d trips of size %d\t%f" %(len(Trip[TSize-1]), TSize, time.clock()-BEGINTIME))
            if TIMEMUTE >=1: print("\n  Now Computing Tsize %d:" %(TSize+1))
            if TIMEMUTE >=3: print("TRIP LIST: ",len(Trip[TSize-1]))
            if len(Trip[TSize-1]) == 0: break
            COUNT = 0
            TravelCOUNT = 0
            for T1 in Trip[TSize-1]:
                
                if COUNT == 0 and TIMEMUTE >=2:
                    print('\t\t',T1)
                    
                if TimeSwitch != 0 and time.clock()-BEGINTIME > TimeLIMIT:
                    break
##                for T2 in Trip[TSize-1]:
##                    TUT = set(T1).union(set(T2))
                for req in feaReq:
                    if TIMEMUTE >=2: print('\tbefore clique', time.clock()-BEGINTIME)                    
                    TUT = set(T1).union({req})
                    if len(TUT) != TSize+1: continue
                    if tuple(sorted(TUT)) in Trip[TSize]: continue
                    if tuple(sorted(TUT)) in INFEASIBELTRIP: continue
                    if TIMEMUTE>=2: print(tuple(sorted(TUT)), Trip[TSize])
                    reqList = sorted(list(TUT))
                                
                    invReqList = {}
                    for ct in range(len(reqList)):
                        invReqList[reqList[ct]] = ct


                    ALLT1SETS = []
                    INFEASIBLEUB = 2e9
                    SKIP = 0
                    if LBPRUN !=0 and T1 in PrevTripCost:
                        T1LB, T1RT = PrevTripCost[tuple(sorted(T1))]
                        INFEASIBELUB = min(INFEASIBLEUB,T1LB+reqs[req].lamb)

                        if len(T1RT[0]) == 1: T1RTT = [ (rqNum,0) if rqNum == 'd' else (invReqList[rqNum],0) for rqNum in T1RT]
                        else: T1RTT = [(rqNum,tim) if rqNum == 'd' else (invReqList[rqNum],tim) for rqNum,tim in T1RT]
                        ALLT1SETS.append(T1RTT)
                        
                        
                    for rr in T1:
                        TUT.remove(rr)
                        if tuple(sorted(TUT)) not in Trip[TSize-1]:
                            TUT.add(rr)
                            SKIP = -1
                            break
                        if LBPRUN != 0:
                            if tuple(sorted(TUT)) in PrevTripCost:
                                TUTLB,TUTRT = PrevTripCost[tuple(sorted(TUT))]
                                INFEASIBLEUB = min(INFEASIBLEUB,TUTLB+reqs[rr].lamb)
                                
                                if len(TUTRT[0]) == 1: TUTRTT = [ (rqNum,0) if rqNum == 'd' else (invReqList[rqNum],0) for rqNum in TUTRT]
                                else: TUTRTT = [(rqNum,tim) if rqNum == 'd' else (invReqList[rqNum],tim) for rqNum,tim in TUTRT]
                                ALLT1SETS.append(TUTRTT)
                                
                        TUT.add(rr)
                    if SKIP == -1:
                        INFEASIBELTRIP.add(tuple(sorted(TUT)))
##                        if TIMEMUTE >=1: print('\t',TUT)
                        SKIP = 0
                        if TIMEMUTE >=2: print('\tafter clique',time.clock()-BEGINTIME)
                        continue
                    
                    if SKIP == 0:
                        INFEASIBELTRIP.add(tuple(sorted(TUT))) ## ALREADY CONSIDERED NOW NO NEED TO CONSIDER
                        reqList = sorted(list(TUT))                        
                        if TIMEMUTE>=2: print('passed: ',tuple(sorted(TUT)))
                        COUNT+=1

##                        invReqList = {}
##                        for ct in range(len(reqList)):
##                            invReqList[reqList[ct]] = ct
                        
                        reqL = [reqs[JS] for JS in reqList]
                        UBRs = None
                        if SIMDPRUN != 0:
                            if i in SimilarD:
                                UBRs = []
                                for d1 in SimilarD[i]:
                                    if tuple(sorted(TUT)) in COST[d1]:
                                        simDcost = COST[d1][tuple(sorted(TUT))]
                                        simDsch = SCHEDULE[d1][tuple(sorted(TUT))]
                                        if len(simDsch[0]) == 1: SIMDRT = [ (rqNum,0) if rqNum == 'd' else (invReqList[rqNum],0) for rqNum in simDsch]
                                        else: SIMDRT = [ (rqNum,tim) if rqNum == 'd' else (invReqList[rqNum],tim) for rqNum in simDsch]
                                        UBRs += SIMDRT
                            else:
                                UBRs = None
                                    
                        if LBPRUN != 0:
                            TUT = tuple(sorted(TUT))
                            T2 = None
                            if T1 in PrevTripCost:
                                LB,ROUTE1 = PrevTripCost[T1]
##                                LB += reqs[req].lamb
##                                INFEASIBLEUB = LB+reqs[req].lamb
                                UB = None
                                if TIMEMUTE >= 2: print("POSSIBLE PREV ROUTES: ",ALLT1SETS)
                                ROUTE = ROUTE1
                                req2 = rd.choice(T1)
                                TMPTUT = set(TUT)
                                TMPTUT.remove(req2)
                                T2 = tuple(sorted(TMPTUT))
                                req1 = req

                                if T2 in PrevTripCost:
                                    LB2, ROUTE2 = PrevTripCost[T2]
                                    req1 = invReqList[req]
                                    req2 = invReqList[req2]

                                    if len(ROUTE1[0]) == 1: RTT1 = [ (rqNum,0) if rqNum == 'd' else (invReqList[rqNum],0) for rqNum in ROUTE1]
                                    else: RTT1 = [(rqNum,tim) if rqNum == 'd' else (invReqList[rqNum],tim) for rqNum,tim in ROUTE1]
                                    if len(ROUTE2[0]) == 1: RTT2 = [(rqNum,0) if rqNum == 'd' else (invReqList[rqNum],0) for rqNum in ROUTE2]   
                                    else: RTT2 = [(rqNum,tim) if rqNum == 'd' else (invReqList[rqNum],tim) for rqNum,tim in ROUTE2]                             

                                    if UBRs == None: UBRs = computeUBroutes(RTT1,RTT2,req1,req2)
                                    else:
                                        tmpUBR = computeUBroutes(RTT1,RTT2,req1,req2)
                                        UBRs += tmpUBR
                                    if TIMEMUTE >=2:print("\tUB:: ", UBRs)
                                else:
                                    if TIMEMUTE >=2:print("\tT2 Not Found", T1, T2)
                            else:
                                if TIMEMUTE >=2:print("\t   WAS NOT IN PREV COST", T1,T2)
                                UB = None
                                LB = None
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

                            if TIMEMUTE >=2: print('\tbefore travel', time.clock()-BEGINTIME)
                            bol,cost,route = travel(drivers[i], reqL, LB = LB, UB = UB,RHO=RHO, UBROUTE = UBRs,INFEASIBLEUB=INFEASIBLEUB)
                            if TIMEMUTE >=2: print('\tafter travel', time.clock()-BEGINTIME,'\t',bol)
                            
##                            if TIMEMUTE !=0: print('\tbefore travel MILP', time.clock()-BEGINTIME)
##                            bol2,cost2,route2 = travelMILP(drivers[i],reqL)
##                            if TIMEMUTE !=0: print('\tafter travel MILP', time.clock()-BEGINTIME)
##                            print('\t',bol,bol2,cost,cost2)
##


                            if MUTE == 3: print('COST:\t', cost)
                            
                        else: #LPPRUN == 0
                            if TSize >= 5 and MUTE !=0: print(reqList)
                            if TIMEMUTE >=2: print('\t\tbefore travel\t', time.clock()-BEGINTIME)
                            bol, cost, route = travel(drivers[i], reqL,RHO=RHO)
                            if TIMEMUTE >=2: print('\t\tafter travel\t', time.clock()-BEGINTIME,'\t',bol)
                            
##                            if TIMEMUTE !=0: print('\tbefore travel MILP', time.clock()-BEGINTIME)
##                            bol2,cost2,route2 = travelMILP(drivers[i],reqL)
##                            if TIMEMUTE !=0: print('\tafter travel MILP', time.clock()-BEGINTIME)
##                            print('\t',bol,bol2,cost,cost2)


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
##                            print(i,Tr)
##                            print(cost)
                            if LBPRUN != 0: TripCost[TUT] = (cost,route)
            if MUTE != 0: print('COUNT:\t',COUNT)

    if TIMEMUTE >=1: print("FINISH PREPROCESSING OPTIMIZING %f" %(time.clock()-BEGINTIME))

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

    if TIMEMUTE >=1: print("START MATCHING %f" %(time.clock()-BEGINTIME))
    MILPSTARTTIME = time.clock()
    m.setParam('OutputFlag', MUTE)
    m.optimize()

    ENDTIME = time.clock()
    if TIMEMUTE >=1: print("Matching Time %f" %(ENDTIME - MILPSTARTTIME))
    print("\n\nRTV3\t Total Time %f" %(ENDTIME - BEGINTIME))


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
        print('Utility of ', i,': ',BUD[i],"  ", AUD[i])
    for j in range(R):
        objSW += UR[j]
        objEff += UR[j]
        print('Utility of ',j,': ',UR[j])
    print("\n\TIME: %f" %(ENDTIME - BEGINTIME))




    return m,x,objSW,objEff,ENDTIME-BEGINTIME
##    return m,x,OneDObjVal+m.ObjVal,ENDTIME-BEGINTIME
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
####        print(ori,des,tim,etim,cap)
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


    drivers = []
    reqs = []

    ##DRIVERS 7
    drivers.append(Driver( (0, 1) , (14, 16) , 0 , 60 , 0 , 4 , 1 , 6 , 158 , 1.2 ))
    drivers.append(Driver( (24, 26) , (13, 20) , 0 , 60 , 0 , 4 , 1 , 6 , 100 , 1.2 ))
    drivers.append(Driver( (2, 2) , (12, 15) , 0 , 60 , 0 , 4 , 1 , 6 , 226 , 1.2 ))
    drivers.append(Driver( (24, 26) , (18, 12) , 0 , 60 , 0 , 4 , 1 , 6 , 151 , 1.2 ))
    drivers.append(Driver( (1, 5) , (14, 13) , 0 , 60 , 0 , 4 , 1 , 6 , 120 , 1.2 ))
    drivers.append(Driver( (24, 29) , (14, 18) , 0 , 60 , 0 , 4 , 1 , 6 , 180 , 1.2 ))
    drivers.append(Driver( (3, 5) , (18, 13) , 0 , 60 , 0 , 4 , 1 , 6 , 217 , 1.2 ))
    ##REQUESTS 21
    reqs.append(Passenger( (4, 3) , (18, 15) , 2 , 35 , 9 , 1 , 3 , 120 , 120 ))
    reqs.append(Passenger( (29, 25) , (14, 15) , 6 , 50 , 18 , 1 , 3 , 89 , 89 ))
    reqs.append(Passenger( (4, 5) , (20, 18) , 3 , 52 , 17 , 1 , 3 , 148 , 148 ))
    reqs.append(Passenger( (28, 25) , (16, 16) , 9 , 38 , 16 , 1 , 3 , 72 , 72 ))
    reqs.append(Passenger( (3, 1) , (14, 20) , 3 , 40 , 10 , 1 , 3 , 148 , 148 ))
    reqs.append(Passenger( (29, 25) , (18, 14) , 3 , 45 , 16 , 1 , 3 , 116 , 116 ))
    reqs.append(Passenger( (0, 1) , (15, 14) , 3 , 45 , 14 , 1 , 3 , 115 , 115 ))
    reqs.append(Passenger( (28, 27) , (17, 17) , 9 , 24 , 9 , 1 , 3 , 89 , 89 ))
    reqs.append(Passenger( (0, 2) , (15, 17) , 1 , 33 , 6 , 1 , 3 , 158 , 158 ))
    reqs.append(Passenger( (27, 25) , (13, 20) , 6 , 27 , 9 , 1 , 3 , 94 , 94 ))
    reqs.append(Passenger( (4, 0) , (13, 17) , 0 , 31 , 5 , 1 , 3 , 108 , 108 ))
    reqs.append(Passenger( (28, 25) , (18, 18) , 13 , 28 , 14 , 1 , 3 , 67 , 67 ))
    reqs.append(Passenger( (1, 2) , (20, 17) , 1 , 44 , 10 , 1 , 3 , 146 , 146 ))
    reqs.append(Passenger( (28, 27) , (17, 16) , 2 , 45 , 15 , 1 , 3 , 116 , 116 ))
    reqs.append(Passenger( (0, 3) , (18, 16) , 4 , 33 , 7 , 1 , 3 , 125 , 125 ))
    reqs.append(Passenger( (24, 29) , (18, 12) , 8 , 38 , 13 , 1 , 3 , 116 , 116 ))
    reqs.append(Passenger( (2, 3) , (20, 12) , 2 , 52 , 16 , 1 , 3 , 120 , 120 ))
    reqs.append(Passenger( (28, 29) , (16, 12) , 6 , 32 , 8 , 1 , 3 , 116 , 116 ))
    reqs.append(Passenger( (2, 5) , (16, 16) , 1 , 35 , 9 , 1 , 3 , 88 , 88 ))
    reqs.append(Passenger( (26, 28) , (20, 17) , 9 , 22 , 9 , 1 , 3 , 85 , 85 ))
    reqs.append(Passenger( (1, 3) , (13, 13) , 4 , 35 , 11 , 1 , 3 , 101 , 101 ))
##
##
    print("Social Welfare")
    m,x,valSW_SW,valEFF_SW,runtime = MatchingRTV(drivers,reqs,RHO=None)
    print(valSW_SW,valEFF_SW)
    print()

##    print("Efficiency")
##    m2,x2,valSW_EFF,valEFF_EFF,runtime2 = MatchingRTV(drivers,reqs,RHO=0)
##    print(valSW_EFF,valEFF_EFF)
##    print()    
    
    

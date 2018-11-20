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
import heapq as hq
 

MUTE = 0

TIMEMUTE = 0

DPLBPRUN = 1
DPEVERYPRUN = 0
global BRUTECOUNT
global DPCOUNT
global MILPCOUNT

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


def travel(d, reqs, LB = None,  UB = None, RHO = None, UBROUTE = None, INFEASIBLEUB = 2e9):
    global BRUTECOUNT
    global DPCOUNT
    global MILPCOUNT
    R = len(reqs)

    BEGINTIME = time.clock()
    
    ind = [reqs[j].et for j in range(R)]
##    print("pvIND: ",ind)
    ind = np.asarray(ind)
    ind = np.argsort(ind).tolist()
##    print("IND: ",ind)

    invind = np.argsort(np.asarray(ind)).tolist()
    
    Reqs = np.array([i for i in range(R)])
    Reqs = Reqs[ind]
    Reqs = Reqs.tolist()

    UBroute = []
    if UBROUTE != None:
        if type(UBROUTE) != list:
            UBroute = [rrr if rrr == 'd' else invind[rrr] for rrr in UBROUTE]
        else:
            for UBR in UBROUTE:
                a = [rrr if rrr == 'd' else invind[rrr] for rrr in UBR]
                UBroute.append(a)
    else:
        UBRt = []
        travelUB = 2e9
    reqs = [reqs[j] for j in ind]
    

    
    
    BRUTECOUNT = 0
    DPCOUNT = 0
    MILPCOUNT = 0
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

    if LB == None:
        LB = -1
##    TotalDriVal = d.val + sum(d.rho*rq.val for rq in reqs)

    if RHO == None:
        RHO = d.rho


    def TrueCost(TreeCost, Rt, disCost):
##        print(Rt)
##        print("IN MILP")
        global MILPCOUNT
        MILPCOUNT+=1


##        print(d,Rt)
##        for r in reqs:
##            print(r)
##            print('\n')
        
        rtSchedule =None
        N = len(Rt)
        timCost = 0
        curDist = 0

        DORI = 0
        DDES = N-1
        DRIVER = R

        # Define Cdev Cdis

        # Define Tset?
        TSet=[]
        vtxLocation = []
        vtxEt = []
        vtxLt = []
        isDes = []
        oriInd = np.zeros(R+1, dtype='int')
        desInd = np.zeros(R+1, dtype='int')
        VISIT = np.zeros(R, dtype='int')
        for i in range(N):
            TSet.append({})
            vtxLocation.append(0)
            vtxEt.append(0)
            vtxLt.append(0)
            isDes.append(False)
            if i == 0:
                oriInd[R] = i
                TSet[i] = range(d.et,d.lt-Distance(d.ori,d.des)+1)
                vtxEt[i] = d.et
                vtxLt[i] = d.lt-Distance(d.ori,d.des)
                vtxLocation[i] = d.ori
                
            elif i == N-1:
                desInd[R] = i
                TSet[i] = range(d.et+Distance(d.ori,d.des),d.lt+1)
                vtxEt[i] = d.et+Distance(d.ori,d.des)
                vtxLt[i] = d.lt
                vtxLocation[  i] = d.des
                isDes[i] = True
                
            elif VISIT[Rt[i]] == 1: #Dstin
                desInd[Rt[i]] = i
                TSet[i] = range(reqs[Rt[i]].et+Distance(reqs[Rt[i]].ori,reqs[Rt[i]].des),reqs[Rt[i]].lt+1)
                vtxLocation[i] = reqs[Rt[i]].des
                VISIT[Rt[i]] = -1
                vtxEt[i] = reqs[Rt[i]].et+Distance(reqs[Rt[i]].ori,reqs[Rt[i]].des)
                vtxLt[i] = reqs[Rt[i]].lt
                isDes[i] = True
                
            elif VISIT[Rt[i]] == 0: #Origin
                oriInd[Rt[i]] = i
                TSet[i] = range(reqs[Rt[i]].et,reqs[Rt[i]].lt-Distance(reqs[Rt[i]].ori,reqs[Rt[i]].des)+1)
                vtxLocation[i] = reqs[Rt[i]].ori
                VISIT[Rt[i]] = 1
                vtxEt[i] = reqs[Rt[i]].et
                vtxLt[i] = reqs[Rt[i]].lt-Distance(reqs[Rt[i]].ori,reqs[Rt[i]].des)
                

        m = grb.Model("TripMILP")
        z = []
        Cdev = []
        for i in range(N):
            z.append({})
            Cdev.append({})
            for t in TSet[i]:
                if i  == 0:
                    Cdev[i][t] = d.cdev*abs(t-d.pt)
                elif i == N-1:
                    Cdev[i][t] = 0
                elif isDes[i]:
                    Cdev[i][t] = 0
                else:
                    Cdev[i][t] = reqs[Rt[i]].cdev*abs(t-reqs[Rt[i]].pt)
                z[i][t] = m.addVar(vtype=grb.GRB.BINARY)

        g = []
        Cdis = []
        for r in range(R+1):
            g.append({})
            Cdis.append({})
            for t in TSet[oriInd[r]]:
                Cdis[r][t] = {}
                g[r][t] = {}
                for tp in TSet[desInd[r]]:
                    g[r][t][tp] = m.addVar(vtype=grb.GRB.BINARY)
                    if tp < t:
                        Cdis[r][t][tp] = 3e9
                    elif r == R:
                        Cdis[r][t][tp] = d.cdet*(tp-t)
                    else:
                        Cdis[r][t][tp] = reqs[r].cdet*(tp-t)


        #Individually Rational



        for r in range(R):
            m.addConstr(reqs[r].val - grb.quicksum(Cdev[oriInd[r]][t]*z[oriInd[r]][t] for t in TSet[oriInd[r]])
                        - grb.quicksum(Cdis[r][t][tp]*g[r][t][tp] for t in TSet[oriInd[r]] for tp in TSet[desInd[r]])
                        >= reqs[r].val - reqs[r].lamb)

        m.addConstr(d.rho* grb.quicksum(reqs[r].val -grb.quicksum(Cdev[oriInd[r]][t]*z[oriInd[r]][t] for t in TSet[oriInd[r]])
                        - grb.quicksum(Cdis[r][t][tp]*g[r][t][tp] for t in TSet[oriInd[r]] for tp in TSet[desInd[r]]) for r in range(R))
                    +d.val - grb.quicksum(Cdev[DORI][t]*z[DORI][t] for t in TSet[DORI])-grb.quicksum(Cdis[DRIVER][t][tp]*g[DRIVER][t][tp]
                                                                                                      for t in TSet[DORI] for tp in TSet[DDES])
                    >= d.val - d.cdet*Distance(d.ori, d.des)
                    )


        # sum of z_vt = 1
        for v in range(N):
            m.addConstr(grb.quicksum(z[v][t] for t in TSet[v]) == 1)

        # sum of g_rtt' = 1
        # grtt' <= z_vt
        for r in range(R+1):
            m.addConstr(grb.quicksum(g[r][t][tp] for t in TSet[oriInd[r]] for tp in TSet[desInd[r]]) == 1)

            for t in TSet[oriInd[r]]:
                for tp in TSet[desInd[r]]:
                    m.addConstr(g[r][t][tp] <= z[oriInd[r]][t])
                    m.addConstr(g[r][t][tp] <= z[desInd[r]][tp])
####

        #z_vt \leq sum z_(v+1t')
        for v in range(N):
            for t in TSet[v]:
                if v == N-1: continue
                m.addConstr(z[v][t] <=
                            grb.quicksum(z[v+1][q] for q in range(max(vtxEt[v+1],t+Distance(vtxLocation[v],vtxLocation[v+1])),vtxLt[v+1]+1)))
##                if v == 0:
##                    m.addConstr(grb.quicksum(z[v][q] for q in range(d.et,t+1))
##                                <=
##                                grb.quicksum(z[v+1][qp] for qp in range(max(vtxEt[v+1],t+Distance(d.ori,vtxLocation[v+1])),vtxLt[v+1])))
##                elif v == N-2:                                                    
##                    m.addConstr(grb.quicksum(z[v][q] for q in range(vtxEt[v],t+1))
##                                <=
##                                grb.quicksum(z[v+1][qp] for qp in range(max(vtxEt[v+1],t+Distance(vtxLocation[v],vtxLocation[v+1])),vtxLt[v+1])))
##                elif v == N-1:
##                    continue
##                    
##                else:                                                    
##                    m.addConstr(grb.quicksum(z[v][q] for q in range(vtxEt[v],t+1))
##                                <=
##                                grb.quicksum(z[v+1][qp] for qp in range(max(vtxEt[v+1],t+Distance(vtxLocation[v],vtxLocation[v+1])),vtxLt[v+1])))



        for r in range(R+1):
            for t in TSet[oriInd[r]]:
                for tp in TSet[desInd[r]]:
                    if r == R:
                        if tp <= t+d.maxDev+Distance(d.ori,d.des): continue
                    else:
                        if tp <= t+reqs[r].maxDev+Distance(reqs[r].ori,reqs[r].des): continue
                    m.addConstr(g[r][t][tp] == 0)

        
        m.setObjective(grb.quicksum((1+RHO)*Cdev[v][t]*z[v][t] for v in range(1,N-1) for t in TSet[v])
                       +
                       grb.quicksum((1+RHO)*Cdis[r][t][tp]*g[r][t][tp]
                                    for r in range(R)
                                    for t in TSet[oriInd[r]]
                                    for tp in TSet[desInd[r]])
                       + grb.quicksum(Cdev[0][t]*z[0][t] for t in TSet[0])
                       + grb.quicksum(Cdev[N-1][t]*z[N-1][t] for t in TSet[N-1])
                       + grb.quicksum(Cdis[R][t][tp]*g[R][t][tp]
                                    for t in TSet[oriInd[R]]
                                    for tp in TSet[desInd[R]])
                       ,
                       grb.GRB.MINIMIZE)


        MUTE = 0
        m.setParam('OutputFlag', MUTE)
        m.optimize()


        if m.status != 2: # NOT OPTIMAL
##            print("INFEASIBLE")
            return None, None, None
##        print(m.Status)


        iniCost = sum(-RHO*reqq.val for reqq in reqs)
##        print('rho: ',RHO)
        rtRt=[]
        for v in range(N):
            for t in TSet[v]:
                if z[v][t].x >=1:
                    rtRt.append((Rt[v],t))
##                    print(z[v][t].x*Cdev[v][t])
        
##        return TreeCost+m.objVal,rtRt,0
##        print('solnqual: ',iniCost,m.ObjVal,iniCost+m.ObjVal)
        return iniCost+m.objVal,rtRt,0






    def TrueDP(TreeCost,Rt,disCost): #TripDP
        global DPCOUNT
##        print(Rt)
        rtSchedule = None
        DPCOUNT += 1
##        print(Rt)
        N = len(Rt)
        timCost = 0
        curDist = 0
        VISIT = 2*np.ones(R+1,dtype=np.int)
        for agnt in Rt:
            if agnt == 'd': VISIT[R] -= 1
            else: VISIT[agnt] -= 1
##        VISIT = np.zeros(R+1,dtype=np.int)
        CostT = [{} for COUNT in range(N)]
        #zCost = [{} for COUNT in range(N)]
        curPas = set()
##        curCost = max(0,TreeCost)
        for v in range(N-1,-1,-1):
##            print(CostT,disCost,curCost)
            if v == N-1 and Rt[-1] == 'd':
                VISIT[R] = 1
                tBound = d.lt
                for t in range(disCost-curDist, tBound+1):
                    CostT[v][t] = (0,[(Rt[v],t)])
                    #zCost[v][t] = 0
                prevTBound = tBound
                prevPd = d.des
                prevP = d
                pvet = d.et
                continue
            elif v == N-1:
                if VISIT[Rt[v]] == 0:
                    VISIT[Rt[v]] = 1
                    tBound = reqs[Rt[v]].lt
                    for t in range(disCost-curDist,tBound+1):
                        CostT[v][t] = (0,[(Rt[v],t)])
                    prevTBound = tBound
                    prevPd = reqs[Rt[v]].des
                    prevP = reqs[Rt[v]]
                    isDest = True
                    curPas.add(reqs[Rt[v]])
                    pvet = reqs[Rt[v]].et+DIST(reqs[Rt[v]])
                if VISIT[Rt[v]] == 1:
                    VISIT[Rt[v]] = 1
                    tBound = reqs[Rt[v]].lt-DIST(reqs[Rt[v]])
                    for t in range(disCost-curDist,tBound+1):
                        CostT[v][t] = (0,[(Rt[v],t)])                
                    prevTBound = tBound
                    prevPd = reqs[Rt[v]].ori
                    prevP = reqs[Rt[v]]
                    pvet = reqs[Rt[v]].et
                continue
            elif v == 0:
                VISIT[R] = -1
                tBound = d.lt-DIST(d)
                p = d
                pd = d.ori
                rtCost = 2e9
                #rtZCost = 2e9
                for t in range(disCost-curDist,tBound+1):
                    for tp in range(max(pvet,t+Distance(pd,prevPd)),prevTBound+1):
##                        print(t,tp,rtCost)
                        if CostT[1][tp][0]+p.cdev*abs(p.pt-t)+p.cdet*(tp-(t+Distance(pd,prevPd))) < rtCost:
##                            print(v+1,tp,CostT[1][tp])
                            rtCost = CostT[1][tp][0]+p.cdev*abs(p.pt-t)+p.cdet*(tp-(t+Distance(pd,prevPd)))
                            CostT[0][t]=(rtCost,[(Rt[0],t)]+CostT[1][tp][1])
                            #zCost[0][t] = p.cdev*abs(p.pt-t)+p.cdet*(tp-(t+Distance(pd,prevPd)))+zCost[1][tp]
                            rtSchedule = CostT[0][t][1]
                            #rtZCost = zCost[0][t]
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
                #minZ = 2e9
                minSch = []
                for tp in range(max(t+Distance(pd,prevPd),pvet),prevTBound+1):
                    sumDetCost = 0
                    #pominZ = 0
                    for rp in curPas:
                        sumDetCost += (1+RHO)*rp.cdet*(tp-(t+Distance(pd,prevPd)))
                        #pominZ += d.rho*rp.cdet*(tp-(t+Distance(pd,prevPd)))
                    sumDetCost += d.cdet*(tp-(t+Distance(pd,prevPd)))
                    #pominZ += d.cdet*(tp-(t+Distance(pd,prevPd)))
                    if CostT[v+1][tp][0]+sumDetCost < minC:
##                        print('v: ',v,'t: ',t,'v+1: ',v+1,'tp: ',tp,'COST: ', CostT[v+1][tp][0]+sumDetCost,
##                              'sumDetCost: ',sumDetCost,'PASS: ',len(curPas))
                        minC = CostT[v+1][tp][0]+sumDetCost
                        #minZ = zCost[v+1][tp]+pominZ
                        minSch = CostT[v+1][tp][1]
                if isDest:
                    CostT[v][t] = (0+minC,[(Rt[v],t)]+minSch)
                    #zCost[v][t] = minZ
                else:
                    CostT[v][t] = ((1+RHO)*p.cdev*abs(ppt-t)+minC,[(Rt[v],t)]+minSch)
                    #zCost[v][t] = minZ
##                    print(p.cdev*abs(ppt-t), ppt, t, p.ori)
            if not isDest and p in curPas:
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
        if MUTE != 0: print('RETURN SCHEDULE: ', rtSchedule)
##        print(TreeCost+timCost)
        return TreeCost+timCost,rtSchedule,0
##        return TreeCost+timCost,rtSchedule,rtZCost
##    print('s', 't', 'Loc', 'c', 'cap', 'GUB', 'RT')


    def bruteForceSearch(step,curTime,curLocation,curCost,curCap,cost,Rt,totalDist, driCost,minRt):
        global BRUTECOUNT
        global MILPCOUNT
##        print(BRUTECOUNT)
        BRUTECOUNT+=1
##        print('begin: ',Rt,cost,step)
##        print(totalDist, Rt)
##        if Rt == ['d',0,1]:
##            print(step,curTime,curLocation,curCost,curCap,cost,Rt)
##        print(step,curTime,curLocation,curCost,curCap,cost,Rt)
##        print(vis)
        retRt = Rt[:]

        if curCost >= travelUB-1e-5:
            return travelUB, UBRt
        if LB >= cost-1e-5:
            return 2e9,[]
##            print('LBRETURN: ',Rt,cost,step)
        if curCost >= INFEASIBLEUB-1e-5:
            return 2e9,[]
        if curCost >= cost-1e-5:
##            print('curReturn: ',Rt,cost,step,retRt)
            return cost, retRt
        mincost = cost
        if step > R*2: # We visited all requests go to driver's destination
##            print(step,curTime,curLocation,curCost,curCap,cost,Rt)
            curCost = curCost+d.cdet*Distance(curLocation,d.des)
            if curCost >= cost-1e-5:
                return cost, retRt
##            driCost = driCost+d.cdet*Distance(curLocation,d.des)
            totalDist = totalDist+Distance(curLocation,d.des)
            retRt = retRt+['d']
##            print(retRt)
            if DPLBPRUN != 0:
                lbDPCost, DPRT, DPZC = TrueDP(curCost,retRt, totalDist)
                if TIMEMUTE >= 2: print('\t\t\t\tDPCOST:  ', lbDPCost)
                if DPRT != None:
                    if lbDPCost >= travelUB-1e-5:
                        return travelUB, UBRt
                    if lbDPCost >= mincost-1e-5:
                        return mincost, minRt
                    # Check IR constraint of DP solution
                    AUD =0
                    UR = np.zeros(R)
                    UDVSIT = -1
                    URVSIT = -np.ones(R)
                    MaxDevBool = True
                    for (agnt, ti) in DPRT:
                        if agnt == 'd' and UDVSIT == -1:
                            AUD = d.val
                            AUD -= d.cdev*abs(ti-d.pt)
                        elif agnt == 'd':
                            AUD -= d.cdet*(ti-UDVSIT)
                            if ti-UDVSIT > d.maxDev:
                                maxDevBool = False
                                break
                        elif URVSIT[agnt] == -1:
                            UR[agnt] = reqs[agnt].val
                            UR[agnt] -= reqs[agnt].cdev*abs(ti-reqs[agnt].pt)
                            URVSIT[agnt] = ti
                        else:
                            UR[agnt] -= reqs[agnt].cdet*(ti-URVSIT[agnt])
                            if ti-URVSIT[agnt] > reqs[agnt].maxDev:
                                MaxDevBool = False
                                break
                            AUD += d.rho*UR[agnt]
                    if UR.all() >= 0 and AUD >= 0 and MaxDevBool == True:
                        return lbDPCost, DPRT
                           
                        
            if TIMEMUTE !=0: print('\t\t\t\t', MILPCOUNT,'before MILP', time.clock()-BEGINTIME)
            nowCost,nowSchedule,nowDriCost = TrueCost(curCost,retRt,totalDist)
            if TIMEMUTE !=0: print('\t\t\t\t  ', nowCost,retRt)
            if TIMEMUTE !=0: print('\t\t\t\t', MILPCOUNT, 'after MILP', time.clock()-BEGINTIME)
            if nowCost == None:
                return 2e9,[]
##            driCost += nowDriCost
##            if driCost > TotalDriVal: # driver's utility is negative
##                return 2e9,[]
            if mincost >= nowCost-1e-5:
                retSchedule = nowSchedule
                retRt = nowSchedule
                minCost = nowCost
            mincost = min(mincost,
                          nowCost)
            return mincost,retRt
        for i in range(R):
            if vis[i] == 1:
                if curTime+Distance(curLocation,reqs[i].des)> reqs[i].lt:
                    return 2e9, []
                if curCost+(d.cdet+reqs[i].cdet)*Distance(curLocation,reqs[i].des)+d.cdet*Distance(reqs[i].des,d.des)>=travelUB:
                    return travelUB,UBRt
                if curCost+(d.cdet+reqs[i].cdet)*Distance(curLocation,reqs[i].des)+d.cdet*Distance(reqs[i].des,d.des)>=cost:
                    return cost, minRt
            if vis[i] == 0:
                if curCost+(d.cdet+reqs[i].cdet)*Distance(curLocation,reqs[i].ori)\
                   +(d.cdet+reqs[i].cdet)*Distance(reqs[i].ori,reqs[i].des)\
                   +d.cdet*Distance(reqs[i].des,d.des)\
                   +reqs[i].cdev*(max((curTime+Distance(curLocation,reqs[i].ori))-reqs[i].pt,0))>=travelUB:
                    return travelUB,UBRt
                if curCost+(d.cdet+reqs[i].cdet)*Distance(curLocation,reqs[i].ori)\
                   +(d.cdet+reqs[i].cdet)*Distance(reqs[i].ori,reqs[i].des)\
                   +d.cdet*Distance(reqs[i].des,d.des)\
                   +reqs[i].cdev*(max((curTime+Distance(curLocation,reqs[i].ori))-reqs[i].pt,0))>=cost:
                    return cost,minRt

        if DPEVERYPRUN !=0 and R>1:
            lbDPCost, DPRT, DPZC = TrueDP(curCost,retRt, totalDist)
            if TIMEMUTE >= 2: print('\t\t\t\tDPEVERYCOST:  ', lbDPCost, travelUB, mincost)            
            if DPRT != None:
                if lbDPCost >= travelUB-1e-5:
                    return travelUB, UBRt
                if lbDPCost >= mincost-1e-5:
                    return mincost, minRt
                    
                   #+min(d.cdev,reqs[i].cdev)*(abs(reqs[i].pt-d.pt-(curTime+Distance(curLocation,reqs[i].ori))))
        if curTime+Distance(curLocation,d.des)>d.lt:
            return 2e9,[]
        if curCost+d.cdet*Distance(curLocation,d.des) >= travelUB:
            return travelUB,UBRt
        if curCost+d.cdet*Distance(curLocation,d.des) >= cost:
            return cost, minRt
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
##                curDriCost = driCost
                for rp in range(R):
                    if vis[rp] == -1 or vis[rp] == 0:continue
                    if rp == i: continue
                    curDetCost += (1+RHO)*reqs[rp].cdet*Distance(curLocation,reqs[i].ori)
##                    curDriCost += d.rho*reqs[rp].cdet*Distance(curLocation,reqs[i].ori)
                curDetCost += d.cdet*Distance(curLocation,reqs[i].ori)
                if curDetCost + max(curTime+Distance(curLocation,reqs[i].ori)-reqs[i].pt,0) >= mincost:
                    return mincost, minRt
##                curDriCost += d.cdet*Distance(curLocation,reqs[i].ori)

##                if curDriCost >= TotalDriVal:
##                    return 2e9,[]
######                totalDist += Distance(curLocation,reqs[i].ori)
                newCost, newRt = bruteForceSearch(step+1,pickupTime,reqs[i].ori,
                                             curDetCost #
                                             ,curCap-1,mincost,Rt+[i],totalDist + Distance(curLocation,reqs[i].ori),0,minRt)
                if newCost < mincost:
                    retRt = newRt[:]
                    mincost = newCost
                    minRt = newRt[:]
                vis[i]=0
            else:
                vis[i]=-1
                dropoffTime = curTime+Distance(curLocation,reqs[i].des)
##                print("here destin",i)
                curDetCost = curCost
##                curDriCost = driCost
                for rp in range(R):
                    if rp == i: curDetCost += (1+RHO)*reqs[rp].cdet*Distance(curLocation,reqs[i].des)
                    if vis[rp] == -1 or vis[rp] == 0: continue
                    curDetCost += (1+RHO)*reqs[rp].cdet*Distance(curLocation,reqs[i].des)
##                    curDriCost += d.rho*reqs[rp].cdet*Distance(curLocation,reqs[i].des)
                curDetCost += d.cdet*Distance(curLocation,reqs[i].des)
##                curDriCost += d.cdet*Distance(curLocation,reqs[i].des)
##                totalDist += Distance(curLocation,reqs[i].des)
                
##                if curDriCost >= TotalDriVal:
##                    return 2e9,[]
                
                newCost, newRt = bruteForceSearch(step+1,dropoffTime,reqs[i].des,
                                               curDetCost #+LAMBDA1*max(0,dropoffTime-(reqs[i].pt+DIST(reqs[i])))
                                               ,curCap+1,mincost,Rt+[i], totalDist+Distance(curLocation,reqs[i].des),0,minRt)
                if newCost < mincost:
                    retRt = newRt[:]
                    mincost = newCost
                    minRt = newRt[:]
                vis[i]=1
##        print('END: ',mincost,retRt)
        return mincost, retRt

    
##    def ATreeSearch():
##        Frontier = []
##        hq.heappush(Frontier,(sum(-RHO*reqq.val for reqq in reqs), -1, [d.ori], ['d'], 0,d.cap,d.et))
##
##        PUSHCOUNTER = 0
##        minCost = 2e9
##        minSchedule = None
##        totlalDist = 0
##        vist = np.zeros(R, dtype=np.int)
##        while(len(Frontier) > 0):
####            print('\nFRONTIER\n',Frontier,'\n')
##            wt, TIEBREAKINGCOUNTER, curLocRt,curRt,curDist, curCap,curTime= hq.heappop(Frontier)
####            print(curRt,curLocRt)
##            if wt > minCost:
##                break
##            if len(curRt) >= 2*R+1: # we are at the leaf node
##                nexLocRt = curLocRt + [d.des]
##                nexRt = curRt+['d']
##                nexWt = wt + d.cdet*(Distance(curLocRt[-1],d.des))
##
##                print('\t\t\t\t before MILP', time.clock()-BEGINTIME)
##                nexWt, nowSchedule,nowDriCost = TrueCost(nexWt,nexRt,curDist)
##                
##                print('\t\t\t\t after MILP', time.clock()-BEGINTIME)
##
##                if nexWt == None:
##                    continue
##                if minCost > nexWt:
##                    minCost = nexWt
##                    minSchedule = nowSchedule
##                continue
##            vist = np.zeros(R,dtype=np.int)
##            for j in curRt:
##                if j == 'd':continue
##                vist[j]+= 1
####            print(vist)
##            SKIP=0
##            for i in range(R):
##                if vist[i] == 1:
##                    if curTime+Distance(curLocRt[-1],reqs[i].des)> reqs[i].lt:
##                        SKIP = -1
##                    if wt+Distance(curLocRt[-1],reqs[i].des)+Distance(reqs[i].des,d.des)>=cost:
##                        SKIP = -1
##            if SKIP == -1: continue
##            if curTime+Distance(curLocRt[-1],d.des)>d.lt: continue
##            if wt+Distance(curLocRt[-1],d.des) >= cost: continue
##            for i in range(R):
##                nexWt = wt
##                if vist[i] == 2:
##                    continue
##                if vist[i] == 0: # picking him up
##                    vist[i] = 1
##                    pickupTime = max(curTime+Distance(curLocRt[-1],reqs[i].ori),reqs[i].et)
##                    if pickupTime + DIST(reqs[i]) > reqs[i].lt and pickupTime+DIST(reqs[i])+Distance(reqs[i].des,d.des) > d.lt:
##                        vist[i] = 0
##                        continue
##                    if curCap == 0:
##                        vist[i] = 0
##                        continue
##                    for rp in range(R):
##                        if vis[rp] == -1 or vis[rp] ==0: continue
##                        if rp == i: continue
##                        nexWt += (1+RHO)*reqs[rp].cdet*Distance(curLocRt[-1],reqs[i].ori)
##                    nexWt += d.cdet*Distance(curLocRt[-1],reqs[i].ori)
##                    nexLocRt = curLocRt+[reqs[i].ori]
##                    nexRt = curRt + [i]
##                    nexDist = curDist+Distance(curLocRt[-1],reqs[i].ori)
##                    nexTime = max(curTime+Distance(curLocRt[-1],reqs[i].ori),pickupTime)
##                    if MUTE != 0: print(nexWt,nexRt,nexDist,curCap,nexTime)
##                    PUSHCOUNTER+=1
##                    hq.heappush(Frontier,(nexWt, PUSHCOUNTER, nexLocRt,nexRt,nexDist,curCap-1,nexTime))
##                    vist[i]=0
##                else: #vist[i] == 1
##                    vist[i] = 2
##                    dropoffTime = curTime+Distance(curLocRt[-1],reqs[i].des)
##                    for rp in range(R):
##                        if rp == i: nexWt += (1+RHO)*reqs[rp].cdet*Distance(curLocRt[-1],reqs[i].des)
##                        if vis[rp] == -1 or vis[rp] == 0: continue
##                        nexWt += (1+RHO)*reqs[rp].cdet*Distance(curLocRt[-1],reqs[i].des)
##                    nexWt += d.cdet*Distance(curLocRt[-1],reqs[i].des)
##                    nexLocRt = curLocRt+[reqs[i].des]
##                    nexRt = curRt + [i]
##                    nexDist = curDist+Distance(curLocRt[-1],reqs[i].des)
##                    nexTime = max(curTime+Distance(curLocRt[-1],reqs[i].des),dropoffTime)
##                    if MUTE != 0: print(nexWt,nexRt,nexDist,curCap,nexTime)
##                    PUSHCOUNTER+=1
##                    hq.heappush(Frontier,(nexWt,PUSHCOUNTER,nexLocRt,nexRt,nexDist,curCap+1,nexTime))
##                    vist[i]=1
        return minCost,minSchedule                            


    initCost = sum(-RHO*reqq.val for reqq in reqs)                    
    UBCost = 2e9
    UBRt = []
    if UBROUTE !=None:
        if type(UBroute) != list:
            if DPLBPRUN != 0:
                UBRCost = 0
                UBRDist = 0
                UR = np.zeros(R)
                UDVSIT = -1
                URVSIT = -np.ones(R)
                curLoc = d.ori
                for agnt in UBroute:
                    if agnt == 'd' and UDVSIT == -1:
                        UBRDist = 0
                        UBRCost = initCost
                        curLoc = d.ori
                        UDVSIT = 0
                    elif agnt == 'd':
                        UBRDist  += Distance(curLoc,d.des)
                        UBRCost += d.cdet*Distance(curLoc,d.des)
                        curLoc = d.des
                        
                    elif URVSIT[agnt] == -1:
                        UBRDist += Distance(curLoc, reqs[agnt].ori)
                        for rrr in range(R):
                            if URVSIT[rrr] == 0:
                                UBRCost += (1+RHO)*reqs[rrr].cdet*Distance(curLoc, reqs[agnt].ori)
                        UBRCost += d.cdet*Distance(curLoc,reqs[agnt].ori)
                        curLoc = reqs[agnt].ori
                        URVSIT[agnt] = 0
                    else:
                        UBRDist += Distance(curLoc, reqs[agnt].des)
                        for rrr in range(R):
                            if URVSIT[rrr] == 0:
                                UBRCost += (1+RHO)*reqs[rrr].cdet*Distance(curLoc, reqs[agnt].des)
                        UBRCost += d.cdet*Distance(curLoc,reqs[agnt].des)
                        curLoc = reqs[agnt].des
                        URVSIT[agnt] = 1
                lbDPCost, DPRT, DPZC = TrueDP(UBRCost,UBR, UBRDist)
                if DPRT == None:
                    pass
                # Check IR constraint of DP solution
                AUD =0
                UR = np.zeros(R)
                UDVSIT = -1
                URVSIT = -np.ones(R)
                maxDevBool = True
                for (agnt, ti) in DPRT:
                    if agnt == 'd' and UDVSIT == -1:
                        AUD = drivers[i].val
                        AUD -= d.cdev*abs(ti-d.pt)
                        UDVSIT = ti
                    elif agnt == 'd':
                        AUD -= d.cdet*(ti-UDVSIT)
                        if ti-UDVSIT > d.maxDev:
                            maxDevBool = False
                            break
                    elif URVSIT[agnt] == -1:
                        UR[agnt] = reqs[agnt].val
                        UR[agnt] -= reqs[agnt].cdev*abs(ti-reqs[agnt].pt)
                        URVSIT[agnt] = ti
                    else:
                        UR[agnt] -= reqs[agnt].cdet*(ti-URVSIT[agnt])
                        AUD += d.rho*UR[agnt]
                        if ti-URVSIT[agnt] > reqs[agnt].maxDev:
                            maxDevBool = False
                            break
                if UR.all() >= 0 and AUD >= 0 and maxDevBool == True:
                    UBCost = lbDPCost
                    UBRt = DPRT
                    UBSchedule = DPRT
                else:
                    UBCost,UBSchedule,UBDriCost = TrueCost(0,UBroute,0)        
            else: UBCost,UBSchedule,UBDriCost = TrueCost(0,UBroute,0)
        else: #list
            CNT = 0
            rd.shuffle(UBroute)
            for UBR in UBroute:
                if DPLBPRUN != 0:
                    UBRCost = 0
                    UBRDist = 0
                    UR = np.zeros(R)
                    UDVSIT = -1
                    URVSIT = -np.ones(R)
                    curLoc = d.ori
                    for agnt in UBR:
                        if agnt == 'd' and UDVSIT == -1:
                            UBRDist = 0
                            UBRCost = initCost
                            curLoc = d.ori
                            UDVSIT = 0
                        elif agnt == 'd':
                            UBRDist  += Distance(curLoc,d.des)
                            UBRCost += d.cdet*Distance(curLoc,d.des)
                            curLoc = d.des
                            
                        elif URVSIT[agnt] == -1:
                            UBRDist += Distance(curLoc, reqs[agnt].ori)
                            for rrr in range(R):
                                if URVSIT[rrr] == 0:
                                    UBRCost += (1+RHO)*reqs[rrr].cdet*Distance(curLoc, reqs[agnt].ori)
                            UBRCost += d.cdet*Distance(curLoc,reqs[agnt].ori)
                            curLoc = reqs[agnt].ori
                            URVSIT[agnt] = 0
                        else:
                            UBRDist += Distance(curLoc, reqs[agnt].des)
                            for rrr in range(R):
                                if URVSIT[rrr] == 0:
                                    UBRCost += (1+RHO)*reqs[rrr].cdet*Distance(curLoc, reqs[agnt].des)
                            UBRCost += d.cdet*Distance(curLoc,reqs[agnt].des)
                            curLoc = reqs[agnt].des
                            URVSIT[agnt] = 1  
                    lbDPCost, DPRT, DPZC = TrueDP(UBRCost,UBR, UBRDist)
                    if TIMEMUTE >= 2: print('\t\t\t\tDPCOST:  ', lbDPCost)
                    if DPRT == None:
                        continue
                    # Check IR constraint of DP solution
                    AUD =0
                    UR = np.zeros(R)
                    UDVSIT = -1
                    URVSIT = -np.ones(R)
                    maxDevBool = True
                    for (agnt, ti) in DPRT:
                        if agnt == 'd' and UDVSIT == -1:
                            AUD = d.val
                            AUD -= d.cdev*abs(ti-d.pt)
                            UDVSIT = ti
                        elif agnt == 'd':
                            AUD -= d.cdet*(ti-UDVSIT)
                            if ti-UDVSIT > d.maxDev:
                                maxDevBool = False
                                break
                        elif URVSIT[agnt] == -1:
                            UR[agnt] = reqs[agnt].val
                            UR[agnt] -= reqs[agnt].cdev*abs(ti-reqs[agnt].pt)
                            URVSIT[agnt] = ti
                        else:
                            UR[agnt] -= reqs[agnt].cdet*(ti-URVSIT[agnt])
                            AUD += d.rho*UR[agnt]
                            if ti-URVSIT[agnt] > reqs[agnt].maxDev:
                                maxDevBool = True
                                break
                    if UR.all() >= 0 and AUD >= 0:
                        UBCost = lbDPCost
                        UBRt = DPRT
                        UBSchedule = DPRT
                        break
                
                CNT+=1
                if len(reqs) > d.cap and CNT >=3:
                    break
                elif CNT >= 5:
                    break
                if TIMEMUTE != 0:print('\t\t\t',UBR)
                UBCost, UBSchedule, UBDriCost = TrueCost(0,UBR,0)
                if TIMEMUTE != 0:print('\t\t\t', UBCost)
                if UBCost != None:
                    UBRt = UBSchedule
                    if TIMEMUTE != 0: print('\t\t\t', UBRt)
                    break
        travelUB = UBCost
        if travelUB == None: travelUB = int(2e9)
    elif UB != None:
        travelUB = min(UB,UBCost)
    else: travelUB = int(2e9)

    if UB == None: UB=2e9
    
    
    if TIMEMUTE != 0: print('\t\t\t before tree', time.clock()-BEGINTIME)
    cost,Rt= bruteForceSearch(1,d.et,d.ori,initCost,d.cap,UB,['d'],0,0,[])
    if TIMEMUTE != 0: print('\t\t\t after tree', time.clock()-BEGINTIME)

    if TIMEMUTE != 0:
        if cost>=1e9: print('\t\t\t',0,None)
        else: print('\t\t\t',cost,Rt)    
##    
##    if TIMEMUTE != 0: print('\t\t\t before Atree', time.clock()-BEGINTIME)
##    cost2,Rt2 = ATreeSearch()
##    if TIMEMUTE != 0: print('\t\t\t after Atree', time.clock()-BEGINTIME)
##
##    if TIMEMUTE != 0: print('\t\t\t', cost,cost2,'\n\t\t\t',Rt,'\n\t\t\t',Rt2)
####    

##
    if MUTE != 0:
        print("RECURSIVE COUNT: ",BRUTECOUNT)
        print("DP COUNT: ", DPCOUNT)
        print("COST   ",cost)
    
    if cost >=1e9:
        return False,0,None

    if cost == UBCost:
        return True, UBCost, UBSchedule

    if len(Rt) <= 2*len(reqs):
        return False,0,None
    
    for i in range(len(Rt)):
        if Rt[i][0] == 'd':
            continue
        else:
            Rt[i] = (ind[Rt[i][0]],Rt[i][1])
        
##    print(Rt, cost)
    return True, cost, Rt
                            


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
##
##    
##
##    d = drivers[0]
##    indd = [0,1,5,10,15,16]
##    reqss = [reqs[i] for i in indd]
##
##
##    drivers = []
##    reqs = []

    
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
##    d = drivers[0]
##    indd = [2,7,11,13,16,18]
##    reqss = [reqs[i] for i in indd]




    drivers = []
    reqs = []
    
    drivers.append(Driver( (3, 0) , (18, 0) , 0 , 60 , 0 , 4 , 1 , 6 , 180 , 1.2 ))

    reqs.append(Passenger( (3, 0) , (18, 0) , 6 , 34 , 12 , 1 , 3 , 72 , 72 ))
    reqs.append(Passenger( (3, 0) , (12, 0) , 12 , 36 , 19 , 1 , 3 , 44 , 44 ))
    reqs.append(Passenger( (5, 0) , (17, 0) , 1 , 37 , 13 , 1 , 3 , 57 , 57 ))    
    reqs.append(Passenger( (2, 0) , (13, 0) , 6 , 42 , 18 , 1 , 3 , 60 , 60 ))
    d = drivers[0]


    Rt3 = [('d', 12), (0, 12), (1, 12), (2, 14), (1, 21), (2, 26), (0, 27), ('d', 27)]
    C3 = 149.6

    Rt2 = [('d', 17), (3, 18), (0, 19), (1, 19), (1, 28), (3, 29), (0, 34), ('d', 34)]
    C2 = 154.2

    Rt1 = [('d', 10), (3, 11), (0, 12), (2, 14), (3, 22), (2, 26), (0, 27), ('d', 27)]
    C1 = 153.6

    Rt0 = [('d', 17), (3, 18), (1, 19), (2, 21), (1, 28), (3, 29), (2, 33), ('d', 34)]
    C0 = 154.6


    OPT = [('d', 10), (3, 11), (0, 12), (1, 12), (2, 14), (1, 21), (3, 22), (2, 26), (0, 27), ('d', 27)]
    COPT = 175.6

    RT1 = Rt3
    RT2 = Rt2

    UBRS = [[]]
    i1= 0
    i2 = 0
    r1 = 3
    r2 = 2
    visited = np.zeros(len(RT1)+2)
    for i in range(len(RT1)):
##        print(RT1[i1][0],RT2[i2][0])
##        print(UBRS)
        
        if i == 0 or i == len(RT1)-1:
            print("1here")
            for UBR in UBRS:
                UBR.append('d')
            i1+=1
            i2+=1
        elif RT1[i1][0] == 'd':
            print("dhere")
            while(RT2[i2][0] != 'd'):
                for UBR in UBRS:
                    UBR.append(RT2[i2][0])
                i2+=1
        elif RT2[i2][0] == 'd':
            print("dhere")
            while(RT1[i1][0] != 'd'):
                for UBR in UBRS:
                    UBR.append(RT1[i1][0])
                i1+=1
        elif RT1[i1][0] == RT2[i2][0]:
            print("2here")
            for UBR in UBRS:
                UBR.append(RT1[i1][0])
            visited[RT1[i1][0]]+=1
            i1+=1
            i2+=1
            continue
        elif RT1[i1][0] == RT2[i2+1][0] and RT1[i1+1][0] == RT2[i2][0]:
            print("3here")
            if RT1[i1][1] <= RT2[i2][1]:
                for UBR in UBRS:
                    UBR.append(RT1[i1][0])
                    UBR.append(RT2[i2][0])
                visited[RT1[i1][0]]+=1
                visited[RT1[i2][0]]+=1                
                i1+=2
                i2+=2
            else:
                for UBR in UBRS:
                    UBR.append(RT2[i2][0])
                    UBR.append(RT1[i1][0])
                visited[RT1[i1][0]]+=1
                visited[RT1[i2][0]]+=1 
                i1+=2
                i2+=2
            continue
        elif RT1[i1][0] == RT2[i2+1][0] and RT2[i2][0] == r1:
            print("4here")
            for UBR in UBRS:
                UBR.append(RT2[i2][0])
                UBR.append(RT1[i1][0])
            visited[RT1[i1][0]]+=1
            visited[RT1[i2][0]]+=1 
            i1+=1
            i2+=2
            continue
        elif RT1[i1+1][0] == RT2[i2][0] and RT1[i1][0] == r2:
            print("5here")
            for UBR in UBRS:
                UBR.append(RT1[i1][0])
                UBR.append(RT2[i2][0])
            visited[RT1[i1][0]]+=1
            visited[RT2[i2][0]]+=1 
            i1+=2
            i2+=1
            continue
        else:
            print("ELSE")
            UBR2S = []
            for UBR in UBRS:
                UBR2 = UBR.copy()
                if visited[RT1[i1][0]] <= 1:
                    UBR.append(RT1[i1][0])
                if visited[RT1[i2][0]] <= 1:
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
                    
                

                        
    print(UBRS)

##    DrawInstance(d,reqs)
    
    b,v, Rt = travel(d,reqs,UBROUTE = UBRS)
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

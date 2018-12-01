import numpy as np
import networkx as nx
import time
import random as rd
import gurobipy as grb
from NewFeasibleBruthWTime import travel
from NewFeasibleBruth import Distance
from types import SimpleNamespace
import logging

DEFAULT_PARAMS = SimpleNamespace()
DEFAULT_PARAMS.LAMBDA = 1
DEFAULT_PARAMS.BIGNUMBER = 5
DEFAULT_PARAMS.LBPRUN = 1 # Set to 1 to do LB Pruning
DEFAULT_PARAMS.RVPRUN = 1 # Set to 1 to do RV Pruning
DEFAULT_PARAMS.GreedySWITCH = 0 # Set to 1 to do Greedy
DEFAULT_PARAMS.PREPROCESSING = 0 # Set to 1 to do PREPROCESSING [Decomposition]
DEFAULT_PARAMS.SIMDPRUN = 1 # set to 1 to do similar Drivers pruning
DEFAULT_PARAMS.CONTINUOUS = 0
DEFAULT_PARAMS.TimeSwitch = 1 # set to 1 to do timelimit
DEFAULT_PARAMS.TimeLIMIT = 60*60*2
DEFAULT_PARAMS.RHO = 1.2 # May want to change this

logger = logging.getLogger('RidesharingProblem')


class RidesharingProblem(object):
    def __init__(self, params=DEFAULT_PARAMS):
        """
        :param params: namespace containing parameters
        """
        self.requests = None
        self.drivers = None
        self.COST = None
        self.preprocessed = False
        self.R, self.D, self.T = None, None, None

        self.params = params

    def setData(self, drivers, requests):
        self.preprocessed = False

        self.requests, self.drivers = requests, drivers

        self.R = len(self.requests)
        self.D = len(self.drivers)
        self.T = min(self.drivers[i].et for i in range(self.D))
        self.T = max(self.drivers[i].lt for i in range(self.D)) - self.T

    def preprocess(self):
        '''
        Computes costs, rtv
        '''

        self.preprocessed = True
        BEGINTIME = time.clock()

        # Set aliases
        drivers, requests = self.drivers, self.requests

        R, D, T = self.R, self.D, self.T

        COST = []
        SCHEDULE = []
        for i in range(D):
            COST.append({})
            SCHEDULE.append({})

        # Compute RV Graph
        feasibleMat = np.zeros((D, R), dtype=int)

        RV = nx.Graph()
        for j in range(R):
            J = requests[j]
            for jp in range(R):
                if j == jp:
                    continue

                JP = requests[jp]
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
                        RV.add_edge(j, jp,weight=prevCost)
                        continue

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
                        RV.add_edge(j, jp, weight=cost)
                        continue

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
                        RV.add_edge(j, jp, weight=cost)
                        continue

                # TODO: how do we describe this segment?
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
                        continue

            # What is this?
            for i in range(D):
                d = drivers[i]
                if self.params.RVPRUN == 1:
                    if J.lt < drivers[i].et or J.et > drivers[i].lt: continue
                    cost = 0
                    Curtim = d.et
                    Curtim = max(Curtim+Distance(d.ori,J.ori),J.et)
                    Curtim = Curtim + Distance(J.ori,J.des)
                    if Curtim >= J.lt: continue
                    if Curtim+Distance(J.des,d.des) >= d.lt: continue

                    # v_d + rho_d*(v_r - mincost_r) - mincost_d -minDevCost_{dr}< v_d - d_cdet*dist_d
                    if d.cdet*(Distance(d.ori,d.des))+d.rho*(J.val-J.cdet*Distance(J.ori,J.des)) \
                            -d.cdet*(Distance(J.ori,J.des)) \
                            -d.cdet*(Distance(d.ori,J.ori)+Distance(J.des,d.des)) \
                            -min(d.cdev,J.cdev*d.rho)*abs(J.pt-d.pt-Distance(d.ori,J.ori))< -1e-5: #driver cannot be IR
                        continue

                logger.info('   ---- before travel', time.clock()-BEGINTIME)
                bol, cost, route = travel(drivers[i],[J], RHO=self.params.RHO)
                logger.info('\t', bol, cost, route, i, j)
                logger.info('   ---- after travel', time.clock()-BEGINTIME)
                ##            if TIMEMUTE !=0: print('\tbefore travel MILP', time.clock()-BEGINTIME)
                ##            bol2,cost2,route2 = travelMILP(drivers[i],[J])
                ##            if TIMEMUTE !=0: print('\tafter travel MILP', time.clock()-BEGINTIME)
                ##            print('\t',bol,bol2,cost,cost2)
                if bol == True:
                    feasibleMat[i][j] = 1
                    RV.add_edge('d'+str(i), j, weight=cost)
                    COST[i][tuple({j})] = cost
                    for schInd in range(len(route)):
                        if route[schInd][0] == 'd':
                            continue
                        route[schInd] = (j, route[schInd][1])
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

        logger.info(feasibleMat)
        logger.info("Finished RV Time: %f" %(time.clock()-BEGINTIME))
        ##    print(FeasibleMat)

        ######################################################333#############################
        ### ORDER MATTER ###
        DEPSILON = 5
        SimilarD = {}
        for d1 in range(D):
            for d2 in range(d1):
                if d1 == d2: continue
                D1 = drivers[d1]
                D2 = drivers[d2]

                if Distance(D1.ori, D2.ori) <= DEPSILON and Distance(D1.des, D2.des) <= DEPSILON:
                    if d2 in SimilarD:
                        SimilarD[d2].add(d1)
                    else:
                        SimilarD[d2] = set([d1])
        logger.info(SimilarD)


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

        ############# EXTRACTED METHOD FROM HOONS CODE ############
        # TODO: remove unnecessary parameters, refactor
        COUNT, RTV, TripCosts, Trips, feaReqs = self.constructRTV(BEGINTIME,
                                                                  COST,
                                                                  D,
                                                                  R,
                                                                  RV,
                                                                  SCHEDULE,
                                                                  drivers,
                                                                  requests)


        # TRIP OF SIZE 2 to R
        self.constructRTV2toR(BEGINTIME, COST, COUNT, D, R, RTV, SCHEDULE,
                              SimilarD, TripCosts, Trips, drivers, feaReqs,
                              requests)
        ############# End extracted portion ############


        logger.info("FINISH PREPROCESSING OPTIMIZING %f" %(time.clock()-BEGINTIME))

        ## ADD EMPTY SET
        for i in range(D):
            dIndex = 'd'+str(i)
            RTV.add_edge('EMPTY'+str(i), dIndex)
            COST[i]['EMPTY'+str(i)] = drivers[i].cdet*Distance(drivers[i].ori, drivers[i].des)
            SCHEDULE[i]['EMPTY'+str(i)] = [('d',drivers[i].pt),
                                           ('d',drivers[i].pt+Distance(drivers[i].ori, drivers[i].des))]
            ##        COST[i]['EMPTY'+str(i)] = 0 # By convension we only care about detour and deviation cost

        logger.info('================COST==============')
        logger.info(COST)
        self.COST, self.RTV, self.SCHEDULE = COST, RTV, SCHEDULE

    def constructRTV2toR(self, BEGINTIME, COST, COUNT, D, R, RTV, SCHEDULE, SimilarD, TripCosts, Trips, drivers,
                         feaReqs, reqs):
        for i in range(D):
            Trip = Trips[i]
            TripCost = TripCosts[i]
            feaReq = feaReqs[i]
            DRIVER = 'd' + str(i)
            logger.info("\nRestart Driver %d\n" % (i))

            for TSize in range(2, R):
                INFEASIBELTRIP = set()
                if self.params.TimeSwitch != 0 and time.clock() - BEGINTIME > self.params.TimeLIMIT:
                    break

                Trip.append(set())
                if self.params.LBPRUN != 0:
                    PrevTripCost = TripCost
                    logger.info("    PREV TRIP LENGTH",
                                                        len(PrevTripCost))  # Dict of (SET : cost, Schedule)
                    TripCost = {}
                logger.info("  There %d candidate trips" % (COUNT))
                logger.info(
                    "  There are %d trips of size %d\t%f" % (len(Trip[TSize - 1]), TSize, time.clock() - BEGINTIME))
                logger.info("\n  Now Computing Tsize %d:" % (TSize + 1))
                logger.info("TRIP LIST: ", len(Trip[TSize - 1]))
                if len(Trip[TSize - 1]) == 0: break
                COUNT = 0
                TravelCOUNT = 0
                for T1 in Trip[TSize - 1]:

                    if COUNT == 0:
                        logger.info('\t\t', T1)

                    if self.params.TimeSwitch != 0 and time.clock() - BEGINTIME > self.params.TimeLIMIT:
                        break
                    ##                for T2 in Trip[TSize-1]:
                    ##                    TUT = set(T1).union(set(T2))
                    for req in feaReq:
                        logger.info('\tbefore clique', time.clock() - BEGINTIME)
                        TUT = set(T1).union({req})
                        if len(TUT) != TSize + 1: continue
                        if tuple(sorted(TUT)) in Trip[TSize]: continue
                        if tuple(sorted(TUT)) in INFEASIBELTRIP: continue
                        logger.info(tuple(sorted(TUT)), Trip[TSize])
                        reqList = sorted(list(TUT))

                        invReqList = {}
                        for ct in range(len(reqList)):
                            invReqList[reqList[ct]] = ct

                        ALLT1SETS = []
                        INFEASIBLEUB = 2e9
                        SKIP = 0
                        if self.params.LBPRUN != 0 and T1 in PrevTripCost:
                            T1LB, T1RT = PrevTripCost[tuple(sorted(T1))]
                            INFEASIBELUB = min(INFEASIBLEUB, T1LB + reqs[req].lamb)

                            if len(T1RT[0]) == 1:
                                T1RTT = [(rqNum, 0) if rqNum == 'd' else (invReqList[rqNum], 0) for rqNum in T1RT]
                            else:
                                T1RTT = [(rqNum, tim) if rqNum == 'd' else (invReqList[rqNum], tim) for rqNum, tim in
                                         T1RT]
                            ALLT1SETS.append(T1RTT)

                        for rr in T1:
                            TUT.remove(rr)
                            if tuple(sorted(TUT)) not in Trip[TSize - 1]:
                                TUT.add(rr)
                                SKIP = -1
                                break
                            if self.params.LBPRUN != 0:
                                if tuple(sorted(TUT)) in PrevTripCost:
                                    TUTLB, TUTRT = PrevTripCost[tuple(sorted(TUT))]
                                    INFEASIBLEUB = min(INFEASIBLEUB, TUTLB + reqs[rr].lamb)

                                    if len(TUTRT[0]) == 1:
                                        TUTRTT = [(rqNum, 0) if rqNum == 'd' else (invReqList[rqNum], 0) for rqNum in
                                                  TUTRT]
                                    else:
                                        TUTRTT = [(rqNum, tim) if rqNum == 'd' else (invReqList[rqNum], tim) for
                                                  rqNum, tim in TUTRT]
                                    ALLT1SETS.append(TUTRTT)

                            TUT.add(rr)
                        if SKIP == -1:
                            INFEASIBELTRIP.add(tuple(sorted(TUT)))
                            ##                        if TIMEMUTE >=1: print('\t',TUT)
                            SKIP = 0
                            logger.info('\tafter clique', time.clock() - BEGINTIME)
                            continue

                        if SKIP == 0:
                            INFEASIBELTRIP.add(tuple(sorted(TUT)))  ## ALREADY CONSIDERED NOW NO NEED TO CONSIDER
                            reqList = sorted(list(TUT))
                            logger.info('passed: ', tuple(sorted(TUT)))
                            COUNT += 1

                            ##                        invReqList = {}
                            ##                        for ct in range(len(reqList)):
                            ##                            invReqList[reqList[ct]] = ct

                            reqL = [reqs[JS] for JS in reqList]
                            UBRs = None
                            if self.params.SIMDPRUN != 0:
                                if i in SimilarD:
                                    UBRs = []
                                    for d1 in SimilarD[i]:
                                        if tuple(sorted(TUT)) in COST[d1]:
                                            simDcost = COST[d1][tuple(sorted(TUT))]
                                            simDsch = SCHEDULE[d1][tuple(sorted(TUT))]
                                            if len(simDsch[0]) == 1:
                                                SIMDRT = [(rqNum, 0) if rqNum == 'd' else (invReqList[rqNum], 0) for
                                                          rqNum in simDsch]
                                            else:
                                                SIMDRT = [(rqNum, tim) if rqNum == 'd' else (invReqList[rqNum], tim) for
                                                          rqNum in simDsch]
                                            if SIMDRT not in UBRs: UBRs += SIMDRT
                                else:
                                    UBRs = None

                            if self.params.LBPRUN != 0:
                                TUT = tuple(sorted(TUT))
                                T2 = None
                                if T1 in PrevTripCost:
                                    LB, ROUTE1 = PrevTripCost[T1]
                                    ##                                LB += reqs[req].lamb
                                    ##                                INFEASIBLEUB = LB+reqs[req].lamb
                                    UB = None
                                    logger.info("POSSIBLE PREV ROUTES: ", ALLT1SETS)
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

                                        if len(ROUTE1[0]) == 1:
                                            RTT1 = [(rqNum, 0) if rqNum == 'd' else (invReqList[rqNum], 0) for rqNum in
                                                    ROUTE1]
                                        else:
                                            RTT1 = [(rqNum, tim) if rqNum == 'd' else (invReqList[rqNum], tim) for
                                                    rqNum, tim in ROUTE1]
                                        if len(ROUTE2[0]) == 1:
                                            RTT2 = [(rqNum, 0) if rqNum == 'd' else (invReqList[rqNum], 0) for rqNum in
                                                    ROUTE2]
                                        else:
                                            RTT2 = [(rqNum, tim) if rqNum == 'd' else (invReqList[rqNum], tim) for
                                                    rqNum, tim in ROUTE2]

                                        if UBRs == None:
                                            UBRs = self.computeUBroutes(RTT1, RTT2, req1, req2)
                                        else:
                                            tmpUBR = self.computeUBroutes(RTT1, RTT2, req1, req2)
                                            UBRs += tmpUBR
                                        logger.info("\tUB:: ", UBRs)
                                    else:
                                        logger.info("\tT2 Not Found", T1, T2)
                                else:
                                    logger.info("\t   WAS NOT IN PREV COST", T1, T2)
                                    UB = None
                                    LB = None
                                    ROUTE = None

                                grdCost = None
                                if ROUTE != None and self.params.GreedySWITCH != 0:
                                    ROUTE = [i if i != 'd' else len(ROUTE) // 2 for i in ROUTE]
                                    pvCost = float('inf')
                                    pvRt = None
                                    totalBol = False
                                    reqL1 = [reqs[HRK] for HRK in T1]
                                    for rtI in range(len(ROUTE) + 1):
                                        for rtJ in range(rtI, len(ROUTE) + 1):
                                            curROUTE = ROUTE[:rtI] + [len(reqL1)] + ROUTE[rtI:rtJ] + [
                                                len(reqL1)] + ROUTE[rtJ:]
                                            ##                                    print(ROUTE, curROUTE)
                                            bol, cost = self.params.Greedy(reqL1 + [reqs[req]] + [drivers[i]], curROUTE,
                                                                           cap=drivers[i].cap)
                                            if bol == True and cost < pvCost:
                                                pvCost = cost
                                                totalBol = True
                                                pvRt = curROUTE
                                    grdCost = pvCost

                                logger.info('\tbefore travel', time.clock() - BEGINTIME)
                                bol, cost, route = travel(drivers[i], reqL, LB=LB, UB=UB, RHO=self.params.RHO,
                                                          UBROUTE=UBRs, INFEASIBLEUB=INFEASIBLEUB)
                                logger.info('\tafter travel', time.clock() - BEGINTIME, '\t',
                                                                    bol)

                                ##                            if TIMEMUTE !=0: print('\tbefore travel MILP', time.clock()-BEGINTIME)
                                ##                            bol2,cost2,route2 = travelMILP(drivers[i],reqL)
                                ##                            if TIMEMUTE !=0: print('\tafter travel MILP', time.clock()-BEGINTIME)
                                ##                            print('\t',bol,bol2,cost,cost2)
                                ##

                                logger.info('COST:\t', cost)

                            else:  # LPPRUN == 0
                                if TSize >= 5:
                                    logger.info(reqList)
                                logger.info('\t\tbefore travel\t', time.clock() - BEGINTIME)
                                bol, cost, route = travel(drivers[i], reqL, RHO=self.params.RHO)
                                logger.info('\t\tafter travel\t', time.clock() - BEGINTIME,
                                                                    '\t', bol)

                            ##                            if TIMEMUTE !=0: print('\tbefore travel MILP', time.clock()-BEGINTIME)
                            ##                            bol2,cost2,route2 = travelMILP(drivers[i],reqL)
                            ##                            if TIMEMUTE !=0: print('\tafter travel MILP', time.clock()-BEGINTIME)
                            ##                            print('\t',bol,bol2,cost,cost2)

                            if bol == True:
                                Tr = tuple(sorted(TUT))
                                Trip[TSize].add(Tr)
                                for r in Tr:
                                    RTV.add_edge(r, Tr)
                                RTV.add_edge(Tr, DRIVER)
                                COST[i][Tr] = cost
                                for schInd in range(len(route)):
                                    if route[schInd][0] == 'd':
                                        continue
                                    route[schInd] = (reqList[route[schInd][0]], route[schInd][1])
                                SCHEDULE[i][Tr] = route
                                ##                            print(i,Tr)
                                ##                            print(cost)
                                if self.params.LBPRUN != 0: TripCost[TUT] = (cost, route)
                logger.info('COUNT:\t', COUNT)

    def constructRTV(self, BEGINTIME, COST, D, R, RV, SCHEDULE, drivers, reqs):
        # RV Graph
        RTV = nx.DiGraph()
        Trips = []
        TripCosts = []
        feaReqs = []
        for r in range(R):
            RTV.add_node(r)
        for i in range(D):
            Trips.append([])
            for UNNECESSARY in range(2):
                Trips[i].append(set())
            feaReqs.append(set())
            TripCosts.append([])
            DRIVER = 'd' + str(i)
            RTV.add_node(DRIVER)
            for d, r in RV.edges(DRIVER):
                Tr = {r}
                feaReqs[i].add(r)
                Tr = tuple(Tr)
                Trips[i][0].add(Tr)
                RTV.add_edge(r, Tr)
                RTV.add_edge(Tr, DRIVER)
        # RTV Graph
        ##Trip = []
        ##    print(RV.edges())
        for i in range(D):
            COUNT = 0
            feaReq = feaReqs[i]
            Trip = Trips[i]
            logger.info("\n\nRTV: Driver %d starting Time: %f" % (i, time.clock() - BEGINTIME))
            ##        for UNNECESSARY in range(2):
            ##            Trip.append(set())

            DRIVER = 'd' + str(i)
            logger.info(" Number of feasibe requests: %d" % (len(RV.edges(DRIVER))))

            ##        if TIMEMUTE >=1: print('StartRTV: ', time.clock()-BEGINTIME)

            logger.info("  Now computeing Tsize 2:")
            if self.params.LBPRUN != 0: TripCost = {}
            for T1 in Trip[0]:
                for T2 in Trip[0]:
                    ##                print("ERERERER")
                    r1 = T1[0]
                    r2 = T2[0]

                    if r1 == r2: continue
                    if (r1, r2) not in RV.edges(): continue

                    if (r1, r2) in Trip[1]: continue
                    if (r2, r1) in Trip[1]: continue
                    reqList = [reqs[r1], reqs[r2]]
                    reqIndex = [r1, r2]
                    INFEASIBLEUB = min(COST[i][tuple({r1})] + reqs[r2].lamb, COST[i][tuple({r2})] + reqs[r1].lamb)
                    ##                print(reqList)
                    logger.info('\tbefore travel', time.clock() - BEGINTIME)
                    COUNT += 1
                    bol, cost, route = travel(drivers[i], reqList, RHO=self.params.RHO, INFEASIBLEUB=INFEASIBLEUB)
                    logger.info('\tafter travel', time.clock() - BEGINTIME)
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

                        Tr = tuple(sorted((r1, r2)))
                        Trip[1].add(Tr)
                        RTV.add_edge(r1, Tr)
                        RTV.add_edge(r2, Tr)
                        RTV.add_edge(Tr, DRIVER)
                        COST[i][Tr] = pvCost
                        for schInd in range(len(route)):
                            if route[schInd][0] == 'd':
                                continue
                            route[schInd] = (reqIndex[route[schInd][0]], route[schInd][1])
                        SCHEDULE[i][Tr] = route
                        if self.params.LBPRUN != 0: TripCost[Tr] = (pvCost, route)

            ######                    print(i,Tr)
            ######                    print(pvCost)
            ######                    if LBPRUN!=0:
            ######                        pvRt = [i-1 if i!=0 else 'd' for i in pvRt]
            ######                        TripCost[Tr] = (pvCost,pvRt)

            TripCosts[i] = TripCost
            feaReqs[i] = feaReq
            logger.info("  Finished pair (2) \t\t%f" % (time.clock() - BEGINTIME))
        return COUNT, RTV, TripCosts, Trips, feaReqs

    def computeUBroutes(self, RT1, RT2, r1, r2, MUTEWITHINFUNCTION = 0):
        """
        given two schedules, create more likely schedule
        :param RT2:
        :param r1:
        :param r2:
        :param MUTEWITHINFUNCTION:
        :return:
        """

        logger.info('RT1 = ',RT1)
        logger.info('RT2 = ', RT2)
        logger.info('r1 = ',r1,'\nr2 = ',r2)

        THRESHOLD = 10
        UBRS = [[]]
        i1= 0
        i2 = 0
        i = 0
        visited = np.zeros(self.R+2)
        while(i < (len(RT1))):
            logger.info("\n")
            logger.info(UBRS[0])
            logger.info(i,i1,i2)
            logger.info(RT1[i1][0], RT2[i2][0])
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
                logger.info('IN THE VISITED -1')
                i1+=1
                if RT1[i1][0] == 'd': break
            while(visited[RT2[i2][0]]) > 1:
                logger.info('IN THE VISITED -2')
                i2+=1
                if RT2[i2][0] == 'd': break
            if RT1[i1][0] == 'd' or RT2[i2][0] =='d': continue
            if RT1[i1][0] == RT2[i2][0]:
                logger.info('EQUAL')
                for UBR in UBRS:
                    UBR.append(RT1[i1][0])
                visited[RT1[i1][0]]+=1
                i1+=1
                i2+=1
                continue
            elif RT1[i1][0] == RT2[i2+1][0] and RT1[i1+1][0] == RT2[i2][0]:
                logger.info('DIAG EQUAL')
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
                logger.info('-1')
                for UBR in UBRS:
                    UBR.append(RT2[i2][0])
                    UBR.append(RT1[i1][0])
                visited[RT1[i1][0]]+=1
                visited[RT2[i2][0]]+=1
                i1+=1
                i2+=2
                continue
            elif RT1[i1+1][0] == RT2[i2][0] and RT1[i1][0] == r2:
                logger.info('-2')
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
                logger.info("HERE AT THE END")
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
                logger.error("\n\nERROR\n")
                logger.error(RT1,RT2,r1,r2)
                logger.error(UBR)
                time.sleep(2)
        return UBRS


    def solve(self, COST=None):
        '''
        :param COST: (optional) modified costs. By default we will use those used in the precomputation
        :return:
        '''
        # Set aliases
        drivers, reqs = self.drivers, self.requests
        D, R = self.D, self.R
        RTV, SCHEDULE = self.RTV, self.SCHEDULE
        if COST is None: COST = self.COST

        if not self.preprocessed:
            raise Exception('Have not preprocessed cost matrix!')

        #######################
        # Just the MILP part #

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
                if self.params.CONTINUOUS == 0: x[v][a] = m.addVar(vtype=grb.GRB.BINARY)
                else: x[v][a] = m.addVar(vtype = 'c', lb= 0, ub = 1)

        # SET variable y for each request
        if self.params.CONTINUOUS == 0: y = m.addVars(R, vtype = grb.GRB.BINARY)
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

        # Added by CK
        BEGINTIME = time.clock()

        logger.info("START MATCHING %f" %(time.clock()-BEGINTIME))
        MILPSTARTTIME = time.clock()
        m.setParam('OutputFlag', 0)
        m.optimize()

        ENDTIME = time.clock()
        logger.info("Matching Time %f" %(ENDTIME - MILPSTARTTIME))
        logger.info("\n\nRTV3\t Total Time %f" %(ENDTIME - BEGINTIME))


        AUD = np.zeros(D) # altruistic Utility
        BUD = np.zeros(D) # Base utility
        UR = np.zeros(R)
        UDVSIT = -np.ones(D)
        URVSIT = -np.ones(R)
        for i in range(D):
            for s in x[i]:
                if x[i][s].x > 0:
                    # CK: print(i,s,COST[i][s],x[i][s].x)
                    # CK: print('  ', SCHEDULE[i][s])
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
        for j in range(R):
            objSW += UR[j]
            objEff += UR[j]

        # CK change: return value (1.0 or 0.0) instead of gurobi variable
        for z in x:
            for k, v in z.items():
                z[k] = v.getAttr('X')
        return m, x, objSW, objEff, ENDTIME-BEGINTIME


if __name__ == '__main__':
    from generate import DataGenerator

    generator = DataGenerator(n=10, DD=40, PRECISION=100, CAP=4, rhoo=1.2)


    print('Generating data...')
    drivers, requests = generator.generate()

    print('Creating ridesharing problem...')
    problem = RidesharingProblem()
    problem.setData(drivers, requests)

    print('Preprocessing ridesharing problem...')
    problem.preprocess()

    print('====================')
    print('Costs per driver')
    for j in problem.COST:
        print(j)
    print('====================')

    m, x, objSW, objEff, time = problem.solve()
    print(m.getAttr('x'))
    for i in range(40):
        print(x[i])
    print(objSW)

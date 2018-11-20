import numpy as np
import networkx as nx
import pylab
import time
import random as rd
import gurobipy as grb
from Classes import Driver
from Classes import Passenger
##from NewFeasible import travel
from itertools import combinations as COMB
import math
##import NewMatchingRTV as nrtv
##import NewMatchingRTV2 as nrtv2
import NewMatchingRTV3 as nrtv3
from NewFeasibleBruthWTime import Distance
import twoMILP as mlp
import twoMILPPrun as mlp2
import DecompMILPPrun as mlp3

def main(n=5, DD = 2, PRECISION = 30, CAP = 4,rhoo=0):
    R = n
    D = DD
    reqs = []
    drivers = []
    T = Distance((0,0), (0.7*PRECISION, 0.7*PRECISION))+PRECISION
    
    LAMBDA = 1
    BIGNUMBER = 5
    MUTE = 1

    timeF = open("data\SW22SWvsEffEFF.txt", 'a+')
    timeF.seek(0)    

    BEGINTIME = time.clock()
##    print("DRIVERS",D)	
##    for i in range(D):
##    ##    ori = (int(rd.uniform(0,1)*PRECISION),int(rd.uniform(0,1)*PRECISION))
##    ##    des = (int(rd.uniform(0,1)*PRECISION),int(rd.uniform(0,1)*PRECISION))
##        if i%2 == 0:
##            ori = (int(rd.uniform(0,0.2)*PRECISION),int(rd.uniform(0,0.2)*PRECISION))
##        if i%2 == 1:
##            ori = (int(rd.uniform(0.8,1)*PRECISION),int(rd.uniform(0.8,1)*PRECISION))
##        des = (int(rd.uniform(0.4,0.7)*PRECISION),int(rd.uniform(0.4,0.7)*PRECISION))
##
##        
##        if i%2 == 0:
##            ori = (int(rd.uniform(0,0.2)*PRECISION),int(0*PRECISION))
##        if i%2 == 1:
##            ori = (int(rd.uniform(0.8,1)*PRECISION),int(0*PRECISION))
##        des = (int(rd.uniform(0.4,0.7)*PRECISION),int(0*PRECISION))
##
##        
##        tim = 0
##        etim = T
##        ptim = 0
##        cdev = 1
##        cdet = 6
##        cap = CAP
##        val = int(Distance(ori,des)*cdet*(1.25+rd.uniform(0,1)))
##        rho = 0
##        
##        drivers.append(Driver(ori,des,tim,etim,ptim,cap,cdev,cdet,val,rho))
##        print('drivers.append(Driver(',ori,",",des,",",tim,",",etim,","
##              ,ptim,",",cap,",",cdev,",",cdet,",",val,",",rho,'))')
##
##    print("REQUESTS",R)                   
##    for i in range(R): 
##        if i%2 == 0:
##            ori = (int(rd.uniform(0,0.2)*PRECISION),int(rd.uniform(0,0.2)*PRECISION))
##        if i%2 == 1:
##            ori = (int(rd.uniform(0.8,1)*PRECISION),int(rd.uniform(0.8,1)*PRECISION))
##        des = (int(rd.uniform(0.4,0.7)*PRECISION),int(rd.uniform(0.4,0.7)*PRECISION))
##
##        
##        if i%2 == 0:
##            ori = (int(rd.uniform(0,0.2)*PRECISION),int(0*PRECISION))
##        if i%2 == 1:
##            ori = (int(rd.uniform(0.8,1)*PRECISION),int(0*PRECISION))
##        des = (int(rd.uniform(0.4,0.7)*PRECISION),int(0*PRECISION))
##        
##        tim = int(rd.uniform(0,max(T/2-2-Distance(ori,des),1)))
##        delta = int(rd.uniform(0,1)*PRECISION)
##        etim = min(tim+delta+Distance(ori,des),T-1)
##        ptim = tim+delta//2
##        cdev = 1
##        cdet = 3
##        val = int(Distance(ori,des)*cdet*(1.5+rd.uniform(0,1)))
##        lamb = val
##        reqs.append(Passenger(ori,des,tim,etim,ptim,cdev,cdet,val,lamb))
##        print('reqs.append(Passenger(',ori,",",des,",",tim,",",etim,",",ptim,
##              ",",cdev,",",cdet,",",val,",",lamb,'))')

    drivers = []
    reqs = []
##    DRIVERS 7
    drivers.append(Driver( (3, 0) , (18, 0) , 0 , 60 , 0 , 4 , 1 , 6 , 180 , 0 ))
##    drivers.append(Driver( (25, 0) , (13, 0) , 0 , 60 , 0 , 4 , 1 , 6 , 91 , 0 ))
##    drivers.append(Driver( (3, 0) , (15, 0) , 0 , 60 , 0 , 4 , 1 , 6 , 150 , 0 ))
##    drivers.append(Driver( (24, 0) , (20, 0) , 0 , 60 , 0 , 4 , 1 , 6 , 39 , 0 ))
##    drivers.append(Driver( (2, 0) , (15, 0) , 0 , 60 , 0 , 4 , 1 , 6 , 158 , 0 ))
##    drivers.append(Driver( (27, 0) , (14, 0) , 0 , 60 , 0 , 4 , 1 , 6 , 158 , 0 ))
##    drivers.append(Driver( (1, 0) , (20, 0) , 0 , 60 , 0 , 4 , 1 , 6 , 181 , 0 ))
##    REQUESTS 21
    reqs.append(Passenger( (3, 0) , (18, 0) , 6 , 34 , 12 , 1 , 3 , 72 , 72 ))
    reqs.append(Passenger( (24, 0) , (19, 0) , 4 , 26 , 12 , 1 , 3 , 24 , 24 ))
    reqs.append(Passenger( (1, 0) , (17, 0) , 9 , 35 , 14 , 1 , 3 , 72 , 72 ))
    reqs.append(Passenger( (27, 0) , (12, 0) , 7 , 31 , 11 , 1 , 3 , 86 , 86 ))
    reqs.append(Passenger( (3, 0) , (12, 0) , 12 , 36 , 19 , 1 , 3 , 44 , 44 ))
    reqs.append(Passenger( (27, 0) , (12, 0) , 6 , 49 , 20 , 1 , 3 , 109 , 109 ))
    reqs.append(Passenger( (5, 0) , (17, 0) , 1 , 37 , 13 , 1 , 3 , 57 , 57 ))
    reqs.append(Passenger( (26, 0) , (16, 0) , 0 , 30 , 10 , 1 , 3 , 60 , 60 ))
    reqs.append(Passenger( (1, 0) , (20, 0) , 2 , 43 , 13 , 1 , 3 , 101 , 101 ))
    reqs.append(Passenger( (24, 0) , (13, 0) , 10 , 22 , 10 , 1 , 3 , 63 , 63 ))
    reqs.append(Passenger( (2, 0) , (13, 0) , 6 , 42 , 18 , 1 , 3 , 60 , 60 ))
##    reqs.append(Passenger( (24, 0) , (12, 0) , 6 , 44 , 19 , 1 , 3 , 88 , 88 ))
##    reqs.append(Passenger( (0, 0) , (19, 0) , 7 , 43 , 15 , 1 , 3 , 139 , 139 ))
##    reqs.append(Passenger( (25, 0) , (13, 0) , 11 , 47 , 23 , 1 , 3 , 88 , 88 ))
##    reqs.append(Passenger( (2, 0) , (13, 0) , 9 , 21 , 9 , 1 , 3 , 56 , 56 ))
##    reqs.append(Passenger( (26, 0) , (14, 0) , 11 , 36 , 17 , 1 , 3 , 61 , 61 ))
##    reqs.append(Passenger( (4, 0) , (19, 0) , 6 , 26 , 8 , 1 , 3 , 70 , 70 ))
##    reqs.append(Passenger( (26, 0) , (18, 0) , 12 , 21 , 12 , 1 , 3 , 58 , 58 ))
##    reqs.append(Passenger( (0, 0) , (18, 0) , 9 , 31 , 11 , 1 , 3 , 129 , 129 ))
##    reqs.append(Passenger( (25, 0) , (18, 0) , 9 , 32 , 17 , 1 , 3 , 39 , 39 ))
##    reqs.append(Passenger( (1, 0) , (17, 0) , 0 , 36 , 10 , 1 , 3 , 106 , 106 ))
        
    newRho = 1.2
    D = len(drivers)
    R = len(reqs)
    for i in range(D):
        drivers[i].rho = newRho
        
    m,x,valSW_SW,valEFF_SW,runtime = nrtv3.MatchingRTV(drivers,reqs,RHO=None)
    print(valSW_SW,valEFF_SW)
    print()

    m2,x2,valSW_EFF,valEFF_EFF,runtime2  = 0,0,0,0,0
##    m2,x2,valSW_EFF,valEFF_EFF,runtime2 = nrtv3.MatchingRTV(drivers,reqs,RHO=0)
##    print(valSW_EFF,valEFF_EFF)
##    print()
##    timeF.write("%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n" %(D, R,newRho,valSW_SW,valEFF_SW,valSW_EFF,valEFF_EFF,runtime,runtime2))
##    timeF.seek(0)
##
##    timeF.close()

    return valSW_SW,valEFF_SW, valSW_EFF,valEFF_EFF, runtime, runtime2

##timeF = open("SWSWvsEffEFF.txt", 'a+')
##timeF.seek(0)
DD = 3
##for j in range(10):
##    if j%2 == 0:
##        DD += 1
##    for i in range(5,15):
##        valSW_SW,valEFF_SW, valSW_EFF,valEFF_EFF, r1, r2 = main(i, DD)
####timeF.close()


R = 21
D = 7
valSW_SW,valEFF_SW, valSW_EFF,valEFF_EFF, r1, r2 = main(R, D)

##DD = 2
##n = 4
##val, runtime = main(n,DD)
    


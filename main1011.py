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

def main(n=5, DD = 2, PRECISION = 30, CAP = 4):
    R = n
    D = DD
    reqs = []
    drivers = []
    T = Distance((0,0), (0.7*PRECISION, 0.7*PRECISION))+PRECISION
    
    LAMBDA = 1
    BIGNUMBER = 5
    MUTE = 1

    BEGINTIME = time.clock()
    print("DRIVERS",D)	
    for i in range(D):
    ##    ori = (int(rd.uniform(0,1)*PRECISION),int(rd.uniform(0,1)*PRECISION))
    ##    des = (int(rd.uniform(0,1)*PRECISION),int(rd.uniform(0,1)*PRECISION))
        if i%2 == 0:
            ori = (int(rd.uniform(0,0.2)*PRECISION),int(rd.uniform(0,0.2)*PRECISION))
        if i%2 == 1:
            ori = (int(rd.uniform(0.8,1)*PRECISION),int(rd.uniform(0.8,1)*PRECISION))
        des = (int(rd.uniform(0.4,0.7)*PRECISION),int(rd.uniform(0.4,0.7)*PRECISION))

        
        if i%2 == 0:
            ori = (int(rd.uniform(0,0.2)*PRECISION),int(0*PRECISION))
        if i%2 == 1:
            ori = (int(rd.uniform(0.8,1)*PRECISION),int(0*PRECISION))
        des = (int(rd.uniform(0.4,0.7)*PRECISION),int(0*PRECISION))

        
        tim = 0
        etim = T
        ptim = 0
        cdev = 1
        cdet = 6
        cap = CAP
        val = int(Distance(ori,des)*(cdet+rd.uniform(0,1)))
        rho = rd.uniform(0,1)
        
        drivers.append(Driver(ori,des,tim,etim,ptim,cap,cdev,cdet,val,rho))
        print('drivers.append(Driver(',ori,",",des,",",tim,",",etim,","
              ,ptim,",",cap,",",cdev,",",cdet,",",val,",",rho,'))')

##    drivers.append(Driver( (0, 0) , (18, 0) , 0 , 60 , 0, 4,1,3 ))
##    drivers.append(Driver( (29, 0) , (17, 0) , 0 , 60 , 0, 4,1,3 ))
##
    print("REQUESTS",R)                   
    for i in range(R): 
        if i%2 == 0:
            ori = (int(rd.uniform(0,0.2)*PRECISION),int(rd.uniform(0,0.2)*PRECISION))
        if i%2 == 1:
            ori = (int(rd.uniform(0.8,1)*PRECISION),int(rd.uniform(0.8,1)*PRECISION))
        des = (int(rd.uniform(0.4,0.7)*PRECISION),int(rd.uniform(0.4,0.7)*PRECISION))

        
        if i%2 == 0:
            ori = (int(rd.uniform(0,0.2)*PRECISION),int(0*PRECISION))
        if i%2 == 1:
            ori = (int(rd.uniform(0.8,1)*PRECISION),int(0*PRECISION))
        des = (int(rd.uniform(0.4,0.7)*PRECISION),int(0*PRECISION))
        
        tim = int(rd.uniform(0,max(T/2-2-Distance(ori,des),1)))
        delta = int(rd.uniform(0,1)*PRECISION)
        etim = min(tim+delta+Distance(ori,des),T-1)
        ptim = tim+delta//2
        cdev = 1
        cdet = 3
        val = int(Distance(ori,des)*cdet*(2+rd.uniform(0,1)))
        lamb = val
        reqs.append(Passenger(ori,des,tim,etim,ptim,cdev,cdet,val,lamb))
        print('reqs.append(Passenger(',ori,",",des,",",tim,",",etim,",",ptim,
              ",",cdev,",",cdet,",",val,",",lamb,'))')
##
####    reqs.append(Passenger( (1, 0) , (20, 0) , 1 , 34 , 7 ,1,3))
####    reqs.append(Passenger( (29, 0) , (20, 0) , 10 , 19 , 10,1,3 ))
####    reqs.append(Passenger( (2, 0) , (17, 0) , 10 , 50 , 11 ,1,3 ))
####    reqs.append(Passenger( (25, 0) , (14, 0) , 11 , 42 , 12 ,1,3 ))
####    reqs.append(Passenger( (2, 0) , (19, 0) , 0 , 35 , 6,1,3 ))



##    drivers.append(Driver( (0, 0) , (14, 0) , 0 , 60 , 0 , 4 , 1  , 3 , 42 , 0 ))
##    drivers.append(Driver( (26, 0) , (15, 12) , 0 , 60 , 0 , 4 , 1  , 3 , 40 , 0 ))
##    
##    reqs.append(Passenger( (0, 0) , (13, 0) , 0 , 44 , 0 , 1 , 3 , 99 , 99 ))
##    reqs.append(Passenger( (25, 0) , (15, 0) , 0 , 19 , 0 , 1 , 3 , 58 , 58 ))
##    reqs.append(Passenger( (0, 0) , (12, 0) , 0 , 44 , 0 , 1 , 3 , 66 , 66 ))
##    reqs.append(Passenger( (26, 0) , (15, 0) , 10 , 42 , 20 , 1 , 3 , 65 , 65 ))
##    reqs.append(Passenger( (0, 0) , (11, 0) , 0 , 33 , 0 , 1 , 3 , 55 , 55 ))    
####    

##    DRIVERS 2
##    drivers.append(Driver( (3, 0) , (12, 13) , 0 , 60 , 0 , 4 , 1 , 13 , 219 , 0.27952716267378686 ))
##    drivers.append(Driver( (24, 0) , (15, 16) , 0 , 60 , 0 , 4 , 1 , 13 , 254 , 0.9380406449994505 ))
####    REQUESTS 4
##    reqs.append(Passenger( (3, 0) , (13, 0) , 16 , 33 , 19 , 1 , 3 , 89 , 89 ))
##    reqs.append(Passenger( (29, 0) , (13, 0) , 10 , 47 , 20 , 1 , 3 , 130 , 130 ))
##    reqs.append(Passenger( (4, 0) , (14, 0) , 9 , 41 , 20 , 1 , 3 , 75 , 75 ))
##    reqs.append(Passenger( (27, 0) , (15, 0) , 12 , 28 , 14 , 1 , 3 , 91 , 91 ))

    print("\nFIRST\n")



##    drivers = []
##    reqs = []
###    #DRIVERS 4
##    drivers.append(Driver( (3, 0) , (13, 0) , 0 , 60 , 0 , 4 , 1 , 6 , 61 , 0.7294316032858155 ))
##    drivers.append(Driver( (28, 0) , (13, 0) , 0 , 60 , 0 , 4 , 1 , 6 , 104 , 0.7188046054668554 ))
##    drivers.append(Driver( (2, 0) , (12, 0) , 0 , 60 , 0 , 4 , 1 , 6 , 64 , 0.14795648659223104 ))
##    drivers.append(Driver( (27, 0) , (13, 0) , 0 , 60 , 0 , 4 , 1 , 6 , 91 , 0.7490288719236787 ))
##    #REQUESTS 8
##    reqs.append(Passenger( (4, 0) , (13, 0) , 8 , 37 , 18 , 1 , 3 , 63 , 63 ))
##    reqs.append(Passenger( (26, 0) , (18, 0) , 16 , 44 , 26 , 1 , 3 , 50 , 50 ))
##    reqs.append(Passenger( (3, 0) , (13, 0) , 13 , 37 , 20 , 1 , 3 , 62 , 62 ))
##    reqs.append(Passenger( (26, 0) , (14, 0) , 11 , 37 , 18 , 1 , 3 , 86 , 86 ))
##    reqs.append(Passenger( (2, 0) , (20, 0) , 0 , 28 , 5 , 1 , 3 , 157 , 157 ))
##    reqs.append(Passenger( (28, 0) , (18, 0) , 13 , 32 , 17 , 1 , 3 , 69 , 69 ))
##    reqs.append(Passenger( (3, 0) , (18, 0) , 11 , 34 , 15 , 1 , 3 , 106 , 106 ))
##    reqs.append(Passenger( (25, 0) , (13, 0) , 5 , 17 , 5 , 1 , 3 , 85 , 85 ))
##



    m,x,valSW,runtime = nrtv3.MatchingRTV(drivers,reqs,RHO=None)
    print(valSW)
    print()

    m2,x2,valEFF,runtime2 = nrtv3.MatchingRTV(drivers,reqs,RHO=0)
    print(valEFF)
    print()

    return valSW, valEFF, runtime, runtime2

timeF = open("data\SWvsEFF.txt", 'a+')
timeF.seek(0)
DD = 3
for j in range(10):
    if j%2 == 0:
        DD += 1
    for i in range(5,15):
        vSW, vEFF, r1, r2 = main(i, DD)
        timeF.write("%d\t%d\t%f\t%f\t%f\t%f\n" %(DD, i,vSW,vEFF,r1,r2))
        timeF.seek(0)
timeF.close()

##DD = 2
##n = 4
##val, runtime = main(n,DD)
    


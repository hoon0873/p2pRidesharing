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


    MAXDEV = 10

##    timeF = open("data\SW22SWvsEffEFF.txt", 'a+')
##    timeF.seek(0)    

    BEGINTIME = time.clock()
    print("#DRIVERS",D)	
    for i in range(D):
    ##    ori = (int(rd.uniform(0,1)*PRECISION),int(rd.uniform(0,1)*PRECISION))
    ##    des = (int(rd.uniform(0,1)*PRECISION),int(rd.uniform(0,1)*PRECISION))
        if i%2 == 0:
            ori = (int(rd.uniform(0,0.2)*PRECISION),int(rd.uniform(0,0.2)*PRECISION))
        if i%2 == 1:
            ori = (int(rd.uniform(0.8,1)*PRECISION),int(rd.uniform(0.8,1)*PRECISION))
        des = (int(rd.uniform(0.4,0.7)*PRECISION),int(rd.uniform(0.4,0.7)*PRECISION))

        
        tim = 0
        etim = T
        ptim = 0
        cdev = 1
        cdet = 6
        cap = CAP
        val = int(Distance(ori,des)*cdet*(1.25+rd.uniform(0,1)))
        rho = rhoo
        maxdev = MAXDEV
        
        drivers.append(Driver(ori,des,tim,etim,ptim,cap,cdev,cdet,val,rho,maxdev))
        print('drivers.append(Driver(',ori,",",des,",",tim,",",etim,","
              ,ptim,",",cap,",",cdev,",",cdet,",",val,",",rho,",",maxdev,'))')

    print("#REQUESTS",R)                   
    for i in range(R): 
        if i%2 == 0:
            ori = (int(rd.uniform(0,0.2)*PRECISION),int(rd.uniform(0,0.2)*PRECISION))
        if i%2 == 1:
            ori = (int(rd.uniform(0.8,1)*PRECISION),int(rd.uniform(0.8,1)*PRECISION))
        des = (int(rd.uniform(0.4,0.7)*PRECISION),int(rd.uniform(0.4,0.7)*PRECISION))
        
        tim = int(rd.uniform(0,max(T/2-2-Distance(ori,des),1)))
        delta = int(rd.uniform(0,1)*PRECISION)
        etim = min(tim+delta+Distance(ori,des),T-1)
        ptim = tim+delta//2
        cdev = 1
        cdet = 3
        val = int(Distance(ori,des)*cdet*(1.5+rd.uniform(0,1)))
        lamb = val
        maxdev = MAXDEV
        
        reqs.append(Passenger(ori,des,tim,etim,ptim,cdev,cdet,val,lamb,maxdev))
        print('reqs.append(Passenger(',ori,",",des,",",tim,",",etim,",",ptim,
              ",",cdev,",",cdet,",",val,",",lamb,",",maxdev,'))')

    newRho = rhoo
    for i in range(D):
        drivers[i].rho = newRho
        
    m,x,valSW_SW,valEFF_SW,runtime = nrtv3.MatchingRTV(drivers,reqs,RHO=None)
    print(valSW_SW,valEFF_SW)
    print()

    m2,x2,valSW_EFF,valEFF_EFF,runtime2 = nrtv3.MatchingRTV(drivers,reqs,RHO=0)
    print(valSW_EFF,valEFF_EFF)
    print()
##    timeF.write("%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n" %(D, R,newRho,valSW_SW,valEFF_SW,valSW_EFF,valEFF_EFF,runtime,runtime2))
##    timeF.seek(0)

    return valSW_SW,valEFF_SW, valSW_EFF,valEFF_EFF, runtime, runtime2

timeF = open("data\Scalability.txt", 'a+')
timeF.seek(0)
DD = 3
newRho = 1.2
for D in range(1,101):
    R = 50
    for REPEAT in range(3):
        valSW_SW,valEFF_SW, valSW_EFF,valEFF_EFF, runtime, runtime2 = main(R, D, rhoo=newRho)
        timeF.write("%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n" %(D, R,newRho,valSW_SW,valEFF_SW,valSW_EFF,valEFF_EFF,runtime,runtime2))
        timeF.seek(0)
timeF.close()

##DD = 2
##n = 4
##val, runtime = main(n,DD)
    

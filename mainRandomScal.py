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
    T = int(1.2*Distance((0,0), (0.5*PRECISION, 0.5*PRECISION))+PRECISION)
    
    LAMBDA = 1
    BIGNUMBER = 5
    MUTE = 1

    DCDEV = 1
    DCDET = 6
    RCDEV = 1
    RCDET = 3
    MAXDEV = 10

##    timeF = open("data\SW22SWvsEffEFF.txt", 'a+')
##    timeF.seek(0)    

    BEGINTIME = time.clock()
    print("#DRIVERS",D)	
    for i in range(D):
        NEIGHBORHOOD = (int(rd.uniform(0.45,0.55)*PRECISION),int(rd.uniform(0.45,0.55)*PRECISION))
        ori = (int(rd.uniform(0,1)*PRECISION),int(rd.uniform(0,1)*PRECISION))
        des = (int(rd.uniform(0,1)*PRECISION),int(rd.uniform(0,1)*PRECISION))

        randCoin = rd.uniform(0,1)
        if randCoin >= 0.525:
            ori = NEIGHBORHOOD
        elif randCoin >= 0.075:
            des = NEIGHBORHOOD
        
####        ori = (int(rd.uniform(0,1.0)*PRECISION),int(rd.uniform(0,1.0)*PRECISION))          
##        des = (int(0.5*PRECISION),int(0.5*PRECISION))

        
        tim = int(rd.uniform(0,max(T/2-2-Distance(ori,des),1)))
        delta = int(rd.uniform(0,1)*PRECISION)
        etim = min(tim+delta+Distance(ori,des),T-1)
        ptim = tim+delta//2
        cdev = DCDEV
        cdet = DCDET
        cap = CAP
        val = int(Distance(ori,des)*cdet*(1.25+rd.uniform(0,1)))
        rho = rhoo
        maxdev = MAXDEV
    
        drivers.append(Driver(ori,des,tim,etim,ptim,cap,cdev,cdet,val,rho,maxdev))
        print('drivers.append(Driver(',ori,",",des,",",tim,",",etim,","
              ,ptim,",",cap,",",cdev,",",cdet,",",val,",",rho,",",maxdev,'))')

    print("#REQUESTS",R)                   
    for i in range(R): 
        NEIGHBORHOOD = (int(rd.uniform(0.45,0.55)*PRECISION),int(rd.uniform(0.45,0.55)*PRECISION))
        ori = (int(rd.uniform(0,1.0)*PRECISION),int(rd.uniform(0,1.0)*PRECISION))  
        des = (int(rd.uniform(0,1)*PRECISION),int(rd.uniform(0,1)*PRECISION))

        randCoin = rd.uniform(0,1)
        if randCoin >= 0.525:
            ori = NEIGHBORHOOD
        elif randCoin >= 0.075:
            des = NEIGHBORHOOD     
##        des = (int(0.5*PRECISION),int(0.5*PRECISION))
        
        tim = int(rd.uniform(0,max(T/2-2-Distance(ori,des),1)))
        delta = int(rd.uniform(0,1)*PRECISION)
        etim = min(tim+delta+Distance(ori,des),T-1)
        ptim = tim+delta//2
        cdev = RCDEV
        cdet = RCDET
        val = int(Distance(ori,des)*cdet*(1.5+rd.uniform(0,1)))
        lamb = val
        maxdev = MAXDEV
        reqs.append(Passenger(ori,des,tim,etim,ptim,cdev,cdet,val,lamb,maxdev))
        print('reqs.append(Passenger(',ori,",",des,",",tim,",",etim,",",ptim,
              ",",cdev,",",cdet,",",val,",",lamb,",",maxdev,'))')


        
    m,x,valSW_SW,valEFF_SW,runtime = nrtv3.MatchingRTV(drivers,reqs,RHO=None)
    print(valSW_SW,valEFF_SW)
    print()

    m2,x2,valSW_EFF,valEFF_EFF,runtime2 = nrtv3.MatchingRTV(drivers,reqs,RHO=0)
    print(valSW_EFF,valEFF_EFF)
    print()
##    timeF.write("%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n" %(D, R,newRho,valSW_SW,valEFF_SW,valSW_EFF,valEFF_EFF,runtime,runtime2))
##    timeF.seek(0)

    return valSW_SW,valEFF_SW, valSW_EFF,valEFF_EFF, runtime, runtime2

timeF = open("data\ScalabilityRandom.txt", 'a+')
timeF.seek(0)
newRho = 1.2
for D in range(1,101):
    R = 3*D
    for REPEAT in range(3):
        valSW_SW,valEFF_SW, valSW_EFF,valEFF_EFF, runtime, runtime2 = main(R, D, rhoo=newRho)
        timeF.write("%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n" %(D, R,newRho,valSW_SW,valEFF_SW,valSW_EFF,valEFF_EFF,runtime,runtime2))
        timeF.seek(0)
timeF.close()

##DD = 2
##n = 4
##val, runtime = main(n,DD)
    

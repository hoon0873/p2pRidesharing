import numpy as np
import networkx as nx
import pylab
import time
import random as rd
import gurobipy as grb
from Classes import Driver
from Classes import Passenger
from NewFeasible import travel
from itertools import combinations as COMB
import math
import NewMatchingRTV as nrtv
import NewMatchingRTV2 as nrtv2
import NewMatchingRTV3 as nrtv3
from NewFeasible import Distance
import twoMILP as mlp
import twoMILPPrun as mlp2
import DecompMILPPrun as mlp3

def main(n=20, DD = 2, PRECISION = 30, CAP = 4):
    R = n
    D = DD
    reqs = []
    drivers = []
    ##T = 150
    T = Distance((0,0), (0.7*PRECISION, 0.7*PRECISION))+PRECISION
##    T = Distance((0,0),(0.7*PRECISION,0.7*PRECISION))+5
    LAMBDA = 1
    BIGNUMBER = 5
    MUTE = 1

    BEGINTIME = time.clock()
    print("DRIVERS",D)	
    for i in range(D):
    ##    ori = (int(rd.uniform(0,1)*PRECISION),int(rd.uniform(0,1)*PRECISION))
    ##    des = (int(rd.uniform(0,1)*PRECISION),int(rd.uniform(0,1)*PRECISION))
        if i%2 == 0:
##        if i == 0:
            ori = (int(rd.uniform(0,0.2)*PRECISION),int(rd.uniform(0,0.2)*PRECISION))
        if i%2 == 1:
##        else:
            ori = (int(rd.uniform(0.8,1)*PRECISION),int(rd.uniform(0.8,1)*PRECISION))
        des = (int(rd.uniform(0.4,0.7)*PRECISION),int(rd.uniform(0.4,0.7)*PRECISION))
##        if i == 0:
##            ori = (int(rd.uniform(0,0.2)*PRECISION),0)
####        if i%2 == 1:
##        else:
##            ori = (int(rd.uniform(0.8,1)*PRECISION),0)
##        des = (int(rd.uniform(0.4,0.7)*PRECISION),0)        
        
    ##    tim = int(rd.uniform(0,max(T/2-2-Distance(ori,des),1)))
    ####    delta = int(rd.uniform(0.8,1)*PRECISION)
    ##    delta = int(rd.uniform(0.5,1)*T)    
    ##    etim = min(tim+delta+Distance(ori,des),T-1)
        tim = 0
        etim = T
    ##    delta = int(rd.uniform(0,1)*PRECISION)
    ##    etim = min(tim+delta+Distance(ori,des),T-1)
        cap = CAP
        drivers.append(Driver(ori,des,tim,etim,cap))
        print('drivers.append(Driver(',ori,",",des,",",tim,",",etim,",",cap,'))')

    print("REQUESTS",R)                   
    for i in range(R): 
    ##    ori = (int(rd.uniform(0,1)*PRECISION),int(rd.uniform(0,1)*PRECISION))
##    ##    des = (int(rd.uniform(0,1)*PRECISION),int(rd.uniform(0,1)*PRECISION))
        if i%2 == 0:
            ori = (int(rd.uniform(0,0.2)*PRECISION),int(rd.uniform(0,0.2)*PRECISION))
        if i%2 == 1:
            ori = (int(rd.uniform(0.8,1)*PRECISION),int(rd.uniform(0.8,1)*PRECISION))
        des = (int(rd.uniform(0.4,0.7)*PRECISION),int(rd.uniform(0.4,0.7)*PRECISION))
##        if i%2 == 0:
##            ori = (int(rd.uniform(0,0.2)*PRECISION),0)
##        if i%2 == 1:
##            ori = (int(rd.uniform(0.8,1)*PRECISION),0)
##        ori = (int(rd.uniform(0,1)*PRECISION),0)
##        des = (int(rd.uniform(0,1)*PRECISION),0)        
        
        tim = int(rd.uniform(0,max(T/2-2-Distance(ori,des),1)))
        delta = int(rd.uniform(0,1)*PRECISION)
        etim = min(tim+delta+Distance(ori,des),T-1)
        ptim = tim+delta//2
    ##    tim = 0
    ##    etim = T-1
        reqs.append(Passenger(ori,des,tim,etim,ptim))
        print('reqs.append(Passenger(',ori,",",des,",",tim,",",etim,",",ptim,'))')


    

##    drivers = []
##    reqs =[]
##
##    drivers.append(Driver((2,0),(11,0),0,30,4))
##
##    reqs.append(Passenger((0,0),(10,0),0,20,5))
##    reqs.append(Passenger((1,0),(9,0),3,20,4))
##    reqs.append(Passenger((3,0),(8,0),2,20,3))
##    reqs.append(Passenger((4,0),(7,0),5,25,2))
##    reqs.append(Passenger((5,0),(6,0),14,15,1))

    print("\nFIRST\n")
##    m,x,val,runtime = nrtv.MatchingRTV(drivers,reqs)
    m,x,val,runtime = nrtv3.MatchingRTV(drivers,reqs)
##    m,x,val,runtime = nrtv2.MatchingRTV(drivers,reqs)
##    m,x,val,runtime, G0, c0 = mlp2.MILP(drivers,reqs)


    print("\nSECOND\n")
##    m1,x1,val1,runtime1 = nrtv2.MatchingRTV(drivers,reqs)
##    P,c = 0,0

##    val1,runtime1= mlp3.Decomp(drivers,reqs)
##    P,c = 0,0

    m1,x1,val1,runtime1,P,c = mlp2.MILP(drivers,reqs)
    
##    m1,x1,val1,runtime1,P,c = 0,0,0,0,0,0

    
    print(val, val1)
    print(runtime,runtime1)
    return runtime, runtime1, val,val1,P,c
    return runtime, 0, val, 0

timeF = open("data\FeasibleTime.txt", 'a+')
timeF.seek(0)
DD = 2
for j in range(10):
    if j%2 == 0:
        DD += 1
    for i in range(5,15):
        r1, r2, v1,v2,P,c = main(i, DD)
        timeF.write("%d\t%d\t%f\t%f\t%f\t%f\n" %(DD, i,r1,v1,r2,v2))
        timeF.seek(0)
timeF.close()
    

